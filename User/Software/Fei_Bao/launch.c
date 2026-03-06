#include "launch.h"
#include "need_h.h"
#include "reload.h"
#include "motor.h" // 引用 motor.h 头文件
#include "math.h"  // 引用 math.h 头文件，使用 fabs

#define test_p 1
#define MIN_ERROR 5 // 自瞄允许偏差

/* shared state variables defined once for the entire program */
long point_angle = 0;
uint8_t shoot_state = 1;
uint8_t reset_state = 0;

void Fei_Bao_motor_init(void)
{

    DJIMotor_init(DJI_M3508, YAW_ROOT);
    DJIMotor_init(DJI_M2006, TRIGGER);

    LZMotor_position_init(MID_MOTO);
    LZMotor_position_init(RIGHT_MOTO);
    LZMotor_position_init(LEFT_MOTO);
}

void Manual_mode(void)
{

    static int n = 0;
    static int x = 1;

    if (RC_data.online != -1 && RC_data.rc.s[0] != 240)
    {
        // 储能电机10010L控制
        if (RC_data.rc.ch[1] > 200)
        {
            sten_moto_spctrl(2);
        }
        else if (RC_data.rc.ch[1] < -200)
        {
            sten_moto_spctrl(-2);
        }
        else
        {
            sten_moto_spctrl(0);
        }

        // 位置-速度双环控制 2006编码器一圈大概为13000
        // 板机复位
        // if (RC_data.rc.s[3] == 240)
        // {
        //     Trigger_up();
        // }
        // 板机发射
        if (RC_data.rc.s[3] == 1807)
        {
            Trigger_down();
        }
    }
    // YAW轴电机控制
    if (RC_data.rc.s[1] == 1807.0f)
    {
        vision(200, 400, 50);
    }
    else
    {
        Shoot_set_yaw_root_velocity(RC_data.rc.ch[3] / 2);
    }
}

// 镖架制导
void vision(int MIN_SPEED, int MAX_SPEED, int ERROR_SPEED)
{
    float yaw_err = vision_data.yaw_error;
    float abs_err = yaw_err > 0 ? yaw_err : -yaw_err;
    float speed = 0;

    if (abs_err >= MIN_ERROR)
    {
        // 最低为MIN_SPEED的速度，偏移量范围每多出100速度就增加ERROR_SPEED
        speed = MIN_SPEED + (int)(abs_err / 100.0f) * ERROR_SPEED;
        // 速度上限最多增加到MAX_SPEED
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
    }

    if (yaw_err >= 0)
    {
        Shoot_set_yaw_root_velocity(-speed);
    }
    else
    {
        Shoot_set_yaw_root_velocity(speed);
    }
}

#if (test_p == 1)

// ######################### 调参区 #########################//

double cnt_angle = 0.0000;
static int zero = 0;
static uint32_t fling_state_start_time = 0;

FlingState_t fling_state = FLING_IDLE;

const uint32_t SERVO_DELAY_MS = 1000;  // 发射延时
const uint32_t DOWN_AGAIN_MS = 200;    // 再次下降延时
const uint32_t TRIGGER_DELAY_MS = 500; // 板机扣动延时

const double SMALL_P = -4.000;              // 近程发射
const double BIG_P = -5.000;                // 远程发射
const double LOW_SPEED = 1;                 // 提前减速区间
const double STEN_MOTOR_TOLERANCE = 0.1000; // 10010L储能容差

const double MOTOR_ANGLE_TOLERANCE = 0.1000; // 灵足电机角度容差，单位：rad

#define GET_LZ_MOTOR_ANGLE(motor_id) (LZ_Motors[motor_id / QUANTITY_OF_LZMOTOR][motor_id % QUANTITY_OF_LZMOTOR].state.angle)

// ######################### 调参区 #########################//
void test(void)
{
}

void Auto_mode(void)
{
    uint32_t current_time = HAL_GetTick();
    if (zero == 0) // 找最近零点
    {
        find_zero();
        if (DM_Motor_data[0][0].motor_data.para.pos >= 0.99 && DM_Motor_data[0][0].motor_data.para.pos <= 1.01 && DM_Motor_data[0][0].motor_data.para.pos != 0.00000000)
        {
            fling_state = FLING_IDLE;
            find_zero();
            zero = 1;
        }
    }
    else if (zero == 1) // 多圈处理
    {
        // 定义状态变量（需在循环外定义或使用static）
        static double last_raw_pos = 0.0000;
        static int32_t round_count = 0;

        // 1. 获取当前原始位置
        float current_raw_pos = DM_Motor_data[0][0].motor_data.para.pos;

        // 2. 检测数值跳变
        // 判定阈值通常取全量程的一半（即 12.5）
        if (current_raw_pos - last_raw_pos < -12.5f)
        {
            // 正向转过临界点：数值从 12.5 跳到 -12.5
            round_count++;
        }
        else if (current_raw_pos - last_raw_pos > 12.5f)
        {
            // 反向转过临界点：数值从 -12.5 跳到 12.5
            round_count--;
        }

        // 3. 计算多圈累计角度
        // 累计角度 = 圈数 * 全量程 + 当前原始位置
        cnt_angle = (float)round_count * 25.00000 + current_raw_pos;

        // 4. 更新上一时刻位置，用于下次计算
        last_raw_pos = current_raw_pos;
    }

    if (zero == 1)
    {
        switch (fling_state)
        {
        case FLING_IDLE:
            // 初始状态，等待开始或切换到复位
            // 用户可能通过某种外部信号触发开始，这里我们假设它被调用时就是开始
            fling_state = Reset_Motor;
            fling_state_start_time = current_time;
            break;

        case Reset_Motor:

            // 首先全部电机位置归零
            sten_moto_spctrl(0);
            LZMotor_set_pos_param(LEFT_MOTO, 1, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 1, 5);
            LZMotor_set_pos_param(MID_MOTO, 1, 5);
            C_dart();
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 1.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 1.0f) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - 1.0f) < MOTOR_ANGLE_TOLERANCE)
            {
                fling_state = Fir_Ready;
                fling_state_start_time = current_time;
            }
            break;

        case Fir_Ready: // 第一次储能

            small_position_ready(Fir_Fire);

            break;

        case Fir_Fire: // 扣下扳机

            if (RC_data.rc.s[3] == 240 || RC_data.rc.s[3] == 0)
            {
                Trigger_down();
                fling_state_start_time = current_time;
            }
            else
            {
                if (current_time - fling_state_start_time >= TRIGGER_DELAY_MS) // 等待板机发射完成
                {
                    fling_state = Fir_Back;
                    fling_state_start_time = current_time;
                }
            }
            break;

        case Fir_Back:

            back(Sec_Ready); // 第一次回勾

            break;

        case Sec_Ready: // 第二次储能

            small_position_ready(Fir_Down);

            break;

        case Fir_Down: // 第一次灵足电机放下

            LZMotor_set_pos_param(LEFT_MOTO, -4.3f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 6.3f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (-4.3f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (6.3f)) < MOTOR_ANGLE_TOLERANCE)

            { // 角度判断也取反
                fling_state = Fir_Push;
                fling_state_start_time = current_time;
            }
            break;

        case Fir_Push: // 第一次夹爪松开

            P_dart();
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS)
            {
                fling_state = Fir_up;
                fling_state_start_time = current_time;
            }
            break;

        case Fir_up: // 第一次灵足电机上升

            LZMotor_set_pos_param(LEFT_MOTO, 1.0f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 1.0f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = Sec_Fire;
                fling_state_start_time = current_time;
            }
            break;

        case Sec_Fire: // 第二次开火

            if (RC_data.rc.s[3] == 240 || RC_data.rc.s[3] == 0)
            {
                Trigger_down(); // 扣下扳机
                fling_state_start_time = current_time;
            }
            else
            {
                if (current_time - fling_state_start_time >= TRIGGER_DELAY_MS) // 等待板机发射完成
                {
                    fling_state = Sec_Back;
                    fling_state_start_time = current_time;
                }
            }
            break;

        case Sec_Back: // 第二次回勾

            back(Thr_Ready);

            break;

        case Thr_Ready: // 第三次储能

            small_position_ready(L_Go);

            break;

        case L_Go: // X轴左移
            P_dart();
            LZMotor_set_pos_param(MID_MOTO, -3.2f, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - (-3.2f)) < MOTOR_ANGLE_TOLERANCE)
            {
                fling_state = L_Down;
                fling_state_start_time = current_time;
            }
            break;

        case L_Down: // 左侧下降

            LZMotor_set_pos_param(LEFT_MOTO, 0.0f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 2.0f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (0.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (2.0f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = L_Catch;
                fling_state_start_time = current_time;
            }
            break;

        case L_Catch: // 左侧夹镖
            C_dart();
            if (current_time - fling_state_start_time >= DOWN_AGAIN_MS)
            {
                fling_state = L_Down_Again;
                fling_state_start_time = current_time;
            }
            break;

        case L_Down_Again:
            LZMotor_set_pos_param(LEFT_MOTO, -0.9, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 2.9f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (-0.9f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (2.9f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = L_Up;
                fling_state_start_time = current_time;
            }
            break;

        case L_Up: // 左侧上升

            LZMotor_set_pos_param(LEFT_MOTO, 1.00, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 1.00, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - 1.00) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - 1.00) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = L_Back;
                fling_state_start_time = current_time;
            }
            break;

        case L_Back: // X轴左侧回归
            LZMotor_set_pos_param(MID_MOTO, 1.0, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - (-1.0f)) < MOTOR_ANGLE_TOLERANCE)
            {
                fling_state = Sec_Down;
                fling_state_start_time = current_time;
            }
            break;

        case Sec_Down: // 第二次下降

            LZMotor_set_pos_param(LEFT_MOTO, -4.3f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 6.3f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (-4.3f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (6.3f)) < MOTOR_ANGLE_TOLERANCE)

            { // 角度判断也取反
                fling_state = Sec_Push;
                fling_state_start_time = current_time;
            }
            break;

        case Sec_Push:

            P_dart();
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS)
            {
                fling_state = Sec_Up;
                fling_state_start_time = current_time;
            }
            break;

        case Sec_Up:

            LZMotor_set_pos_param(LEFT_MOTO, 1.0f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 1.0f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = Thr_Fire;
                fling_state_start_time = current_time;
            }

            break;

        case Thr_Fire: // 第三次开火

            if (RC_data.rc.s[3] == 240 || RC_data.rc.s[3] == 0)
            {
                Trigger_down(); // 扣下扳机
                fling_state_start_time = current_time;
            }
            else
            {
                if (current_time - fling_state_start_time >= TRIGGER_DELAY_MS) // 等待板机发射完成
                {
                    fling_state = Thr_Back;
                    fling_state_start_time = current_time;
                }
            }
            break;

        case Thr_Back:

            back(Fou_Ready);

            break;

        case Fou_Ready:

            small_position_ready(R_Go);
            break;

        case R_Go:

            P_dart();
            LZMotor_set_pos_param(MID_MOTO, 5.2f, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - (5.2f)) < MOTOR_ANGLE_TOLERANCE)
            {
                fling_state = R_Down;
                fling_state_start_time = current_time;
            }
            break;

        case R_Down:

            LZMotor_set_pos_param(LEFT_MOTO, 0.0f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 2.0f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (0.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (2.0f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = R_Catch;
                fling_state_start_time = current_time;
            }
            break;

        case R_Catch:

            C_dart();
            if (current_time - fling_state_start_time >= DOWN_AGAIN_MS)
            {
                fling_state = R_Down_Again;
                fling_state_start_time = current_time;
            }
            break;

        case R_Down_Again:
            LZMotor_set_pos_param(LEFT_MOTO, -0.9, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 2.9f, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (-0.9f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (2.9f)) < MOTOR_ANGLE_TOLERANCE)
            {
                fling_state = R_Up;
                fling_state_start_time = current_time;
            }
            break;

        case R_Up:
            LZMotor_set_pos_param(LEFT_MOTO, 1.0f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 1.0f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = R_Back;
                fling_state_start_time = current_time;
            }

        case R_Back:
            LZMotor_set_pos_param(MID_MOTO, 1.0f, 5);
            if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE)
            {
                fling_state = Thr_Down;
                fling_state_start_time = current_time;
            }
            break;

        case Thr_Down:
            LZMotor_set_pos_param(LEFT_MOTO, -4.3f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 6.3f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (-4.3f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (6.3f)) < MOTOR_ANGLE_TOLERANCE)

            { // 角度判断也取反
                fling_state = Thr_Push;
                fling_state_start_time = current_time;
            }
            break;
        case Thr_Push:

            P_dart();
            if (current_time - fling_state_start_time >= SERVO_DELAY_MS)
            {
                fling_state = Thr_Up;
                fling_state_start_time = current_time;
            }
            break;

        case Thr_Up:
            LZMotor_set_pos_param(LEFT_MOTO, 1.0f, 5);
            LZMotor_set_pos_param(RIGHT_MOTO, 1.0f, 5); // 右侧电机角度取反
            if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE &&
                fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (1.0f)) < MOTOR_ANGLE_TOLERANCE)
            { // 角度判断也取反
                fling_state = Fou_Fire;
                fling_state_start_time = current_time;
            }
        case Fou_Fire:
            if (RC_data.rc.s[3] == 240 || RC_data.rc.s[3] == 0)
            {
                Trigger_down(); // 扣下扳机
                fling_state_start_time = current_time;
            }
            else
            {
                if (current_time - fling_state_start_time >= TRIGGER_DELAY_MS) // 等待板机发射完成
                {
                    fling_state = Non_Task;
                    fling_state_start_time = current_time;
                }
            }
            break;

        case Fou_Back:

            back(Non_Task);

            break;
        }
    }
}

#endif

#if (test_p == 0)

double cnt_angle = 0.0000;
static int zero = 0;

void test(void)
{

    //    set_servo_angle(PWM_PIN_1, 0);
    //    set_servo_angle(PWM_PIN_2, 90);
    //    set_servo_angle(PWM_PIN_3, 180);
    //    set_servo_angle(PWM_PIN_4, 180);
    if (zero == 0)
    {
        find_zero();
        if (DM_Motor_data[0][0].motor_data.para.pos >= -0.01 && DM_Motor_data[0][0].motor_data.para.pos <= 0.01 && DM_Motor_data[0][0].motor_data.para.pos != 0.0000000)
        {

            zero = 1;
        }
    }
    else if (zero == 1) // 多圈处理
    {
        // 定义状态变量（需在循环外定义或使用static）
        static double last_raw_pos = 0.0000;
        static int32_t round_count = 0;

        // 1. 获取当前原始位置
        float current_raw_pos = DM_Motor_data[0][0].motor_data.para.pos;

        // 2. 检测数值跳变
        // 判定阈值通常取全量程的一半（即 12.5）
        if (current_raw_pos - last_raw_pos < -12.5f)
        {
            // 正向转过临界点：数值从 12.5 跳到 -12.5
            round_count++;
        }
        else if (current_raw_pos - last_raw_pos > 12.5f)
        {
            // 反向转过临界点：数值从 -12.5 跳到 12.5
            round_count--;
        }

        // 3. 计算多圈累计角度
        // 累计角度 = 圈数 * 全量程 + 当前原始位置
        cnt_angle = (float)round_count * 25.00000 + current_raw_pos;

        // 4. 更新上一时刻位置，用于下次计算
        last_raw_pos = current_raw_pos;
    }

    if (zero == 1)
    {

        p_ctrl(6.25, 2, 0.03);
    }
}

#endif

#if (test_p == 2)
void Auto_mode(void)
{
}

void test(void)
{

    set_servo_angle(PWM_PIN_1, 130);

    set_servo_angle(PWM_PIN_2, 80);
}

#endif