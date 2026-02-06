#include "launch.h"

#define MIN_ERROR 5 //自瞄允许偏差


void Fei_Bao_motor_init(void){

    DJIMotor_init(DJI_M3508,YAW_ROOT);
    DJIMotor_init(DJI_M2006,TRIGGER);

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
