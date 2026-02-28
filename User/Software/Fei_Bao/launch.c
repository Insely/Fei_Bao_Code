#include "launch.h"
#include "motor.h"  // 添加 motor.h 头文件
#include "math.h"   // 添加 math.h 头文件，用于 fabs

#define test_p 0
#define MIN_ERROR 5 //自瞄允许偏差

void Fei_Bao_motor_init(void){

    DJIMotor_init(DJI_M3508,YAW_ROOT);
    DJIMotor_init(DJI_M2006,TRIGGER);

    LZMotor_position_init(MID_MOTO);
    LZMotor_position_init(RIGHT_MOTO);
    LZMotor_position_init(LEFT_MOTO);

}


void Auto_mode(void) {

}


void Manual_mode(void){

    static int n=0;
    static int x=1;
    static double p_angle=0.0;
    static long point_angle=0;

    static uint8_t shoot_state=1;
    static uint8_t reset_state=0;

    if(RC_data.online!=-1&&RC_data.rc.s[0]!=240){ 
    //储能电机10010L控制
    if(RC_data.rc.ch[1]>200){
        p_angle=100;
        sten_moto_ctrl(p_angle,2);
    }
    else if(RC_data.rc.ch[1]<-200){
        p_angle=-100;
        sten_moto_ctrl(p_angle,2);
    } 
    else{
        sten_moto_ctrl(p_angle,0);
        p_angle=0.0;
    }

    //YAW轴电机控制
    if (RC_data.rc.s[2]==1024.0f){
        vision(200,400,50);
    }
    else{
        Shoot_set_yaw_root_velocity(RC_data.rc.ch[3]/2);
    }
   
    // if(RC_data.rc.s[3]==240) n+=1;
    // if(RC_data.rc.s[1]==1807 && n==x) {
    //     save_moto_zero(); 
    //     x+=1;
    // }
    //位置-速度双环控制 2006编码器一圈大概为13000
    //板机复位
    if(RC_data.rc.s[1]==240){
        if(shoot_state==0){
        point_angle+=2000;     
        Shoot_set_sten_trigger_position(point_angle);
        reset_state=0;
        shoot_state=1;
        }
    }
    //板机发射
    else if(RC_data.rc.s[1]==1807){
        if(reset_state==0){
        point_angle+=11000;
        Shoot_set_sten_trigger_position(point_angle);
        reset_state=1;
        shoot_state=0;
        }
    }
    //速度环控制
    // if(RC_data.rc.s[3]>1800){
    // if(trigger_state==0){
    // start_time=Get_sys_time_s();
    // trigger_state=1;
    // }
    // else{
    // run_time=Get_sys_time_s()-start_time;
    // }
    // if(run_time<0.17){
    // Shoot_set_sten_trigger_velocity(400);
    // }
    // else if(run_time>=0.17){
    // Shoot_set_sten_trigger_velocity(0);
    // }
    // }
    // else if(RC_data.rc.s[3]<300){
    //     trigger_state=0;
    // }

    // if(RC_data.rc.ch[2]>700||RC_data.rc.ch[2]<-700){
    // Shoot_set_sten_trigger_velocity(RC_data.rc.ch[2]/3);
    // }
    // else{
    // Shoot_set_sten_trigger_velocity(0);   
    // }

    }
}

//镖架制导
void vision(int MIN_SPEED,int MAX_SPEED,int ERROR_SPEED){
          float yaw_err = vision_data.yaw_error;
      float abs_err = yaw_err > 0 ? yaw_err : -yaw_err;
      float speed = 0;

      if (abs_err >= MIN_ERROR) {
            // 最低为MIN_SPEED的速度，偏移量范围每多出100速度就增加ERROR_SPEED
            speed = MIN_SPEED + (int)(abs_err / 100.0f) * ERROR_SPEED;
            // 速度上限最多增加到MAX_SPEED
            if (speed > MAX_SPEED) speed = MAX_SPEED;
      }

      if(yaw_err >= 0){
            Shoot_set_yaw_root_velocity(-speed);
      }
      else {
            Shoot_set_yaw_root_velocity(speed);
      }
}

typedef enum {
    FLING_IDLE,
    FLING_RESET_MOTORS, // 所有电机位置归零
    FLING_STATE_1,      // PIN1舵机设定角度为60度
    FLING_STATE_2,      // 两侧电机位置负3
    FLING_STATE_3,      // 中间电机位置-6
    FLING_STATE_4,      // 两侧电机位置3
    FLING_STATE_5,      // PIN1舵机设定角度为120度
    FLING_STATE_6,      // 两侧电机位置0
    FLING_STATE_7,      // 中间电机位置0
    FLING_STATE_8,      // 两侧电机位置3
    FLING_STATE_9,      // PIN1舵机设定角度为60度
    FLING_STATE_10,     // 两侧电机位置0
    FLING_STATE_11,     // 中间电机位置6
    FLING_STATE_12,     // 两侧电机位置3
    FLING_STATE_13,     // PIN1舵机设定角度为120度
    FLING_STATE_14,     // 两侧电机位置0
    FLING_STATE_15,     // 中间电机位置0
    FLING_STATE_16,     // 两侧电机位置3
    FLING_STATE_17,     // PIN1舵机设定角度为60度
    FLING_DONE          // 结束
} FlingState_t;

#if (test_p==1)

static FlingState_t fling_state = FLING_IDLE;
static uint32_t fling_state_start_time = 0;
static const uint32_t SERVO_DELAY_MS = 1000; // 舵机延时
// static const uint32_t MOTOR_MOVE_DELAY_MS = 500; // 电机移动延时 (不再使用，改为角度检测)
static const float MOTOR_ANGLE_TOLERANCE = 0.1f; // 电机角度容差，单位：rad

#define GET_LZ_MOTOR_ANGLE(motor_id) (LZ_Motors[motor_id / QUANTITY_OF_LZMOTOR][motor_id % QUANTITY_OF_LZMOTOR].state.angle)

void test(void) {
    uint32_t current_time = HAL_GetTick();

    switch (fling_state) {
        case FLING_IDLE:
            // 初始状态，等待开始或切换到复位
            // 用户可能通过某种外部信号触发开始，这里我们假设它被调用时就是开始
            fling_state = FLING_RESET_MOTORS;
            fling_state_start_time = current_time;
            break;

        case FLING_RESET_MOTORS:
            // 首先全部电机位置归零
            LZMotor_set_pos_param(LEFT_MOTO, 0, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 0, 5);
            LZMotor_set_pos_param(MID_MOTO, 0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_1;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_1: // PIN1舵机设定角度为60度
            set_servo_angle(PWM_PIN_1, 60);
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS) {
                fling_state = FLING_STATE_2;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_2: // 两侧电机位置负3
            LZMotor_set_pos_param(LEFT_MOTO, -3, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 3, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (-3.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 3.0f) < MOTOR_ANGLE_TOLERANCE) { // 角度判断也取反
                fling_state = FLING_STATE_3;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_3: // 中间电机位置-6完成
            LZMotor_set_pos_param(MID_MOTO, -6, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - (-6.0f)) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_4;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_4: // 两侧电机位置3
            LZMotor_set_pos_param(LEFT_MOTO, 3, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, -3, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 3.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (-3.0f)) < MOTOR_ANGLE_TOLERANCE) { // 角度判断也取反
                fling_state = FLING_STATE_5;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_5: // PIN1舵机设定角度为120度
            set_servo_angle(PWM_PIN_1, 120);
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS) {
                fling_state = FLING_STATE_6;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_6: // 两侧电机位置0
            LZMotor_set_pos_param(LEFT_MOTO, 0, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_7;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_7: // 中间电机位置0
            LZMotor_set_pos_param(MID_MOTO, 0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_8;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_8: // 两侧电机位置3
            LZMotor_set_pos_param(LEFT_MOTO, 3, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, -3, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 3.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (-3.0f)) < MOTOR_ANGLE_TOLERANCE) { // 角度判断也取反
                fling_state = FLING_STATE_9;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_9: // PIN1舵机设定角度为60度
            set_servo_angle(PWM_PIN_1, 60);
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS) {
                fling_state = FLING_STATE_10;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_10: // 两侧电机位置0
            LZMotor_set_pos_param(LEFT_MOTO, 0, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_11;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_11: // 中间电机位置6
            LZMotor_set_pos_param(MID_MOTO, 6, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - 6.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_12;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_12: // 两侧电机位置3
            LZMotor_set_pos_param(LEFT_MOTO, 3, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, -3, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 3.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (-3.0f)) < MOTOR_ANGLE_TOLERANCE) { // 角度判断也取反
                fling_state = FLING_STATE_13;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_13: // 舵机设定角度为120度
            set_servo_angle(PWM_PIN_1, 120);
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS) {
                fling_state = FLING_STATE_14;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_14: // 两侧电机位置0
            LZMotor_set_pos_param(LEFT_MOTO, 0, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_15;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_15: // 中间电机位置0
            LZMotor_set_pos_param(MID_MOTO, 0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - 0.0f) < MOTOR_ANGLE_TOLERANCE) {
                fling_state = FLING_STATE_16;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_16: // 两侧电机位置3
            LZMotor_set_pos_param(LEFT_MOTO, 3, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, -3, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 3.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (-3.0f)) < MOTOR_ANGLE_TOLERANCE) { // 角度判断也取反
                fling_state = FLING_STATE_17;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_STATE_17: // 舵机设定角度为60度
            set_servo_angle(PWM_PIN_1, 60);
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS) {
                fling_state = FLING_DONE;
                fling_state_start_time = current_time;
            }
            break;

        case FLING_DONE:
            // 自动化流程结束，可以保持电机和舵机在最终状态，或者回到FLING_IDLE等待下次触发
            // 如果需要循环，可以在这里将fling_state设为FLING_IDLE或FLING_RESET_MOTORS
            break;
    }
}
#endif

#if (test_p==0)


void test(void){
 
   set_servo_angle(PWM_PIN_1, 0);
   set_servo_angle(PWM_PIN_2, 90);
   set_servo_angle(PWM_PIN_3, 180);
   set_servo_angle(PWM_PIN_4, 180);
   

}

#endif