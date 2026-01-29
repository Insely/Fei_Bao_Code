#include "launch.h"

void Fei_Bao_motor_init(void){

    DJIMotor_init(DJI_M3508,YAW_ROOT);
    DJIMotor_init(DJI_M2006,TRIGGER);

}



void Manual_mode(void){

    static int n=0;
    static int x=1;
    static double p_angle=0.0;
    static uint8_t Moto_state=0;
    static long point_angle=0;

    static uint8_t shoot_state=1;
    static uint8_t reset_state=0;

    if(RC_data.online!=-1&&RC_data.rc.s[0]!=240){ 
    //储能电机10010L控制
    if(RC_data.rc.ch[1]>200){
        p_angle=100;
        Moto_state=0;
    }
    else if(RC_data.rc.ch[1]<-200){
        p_angle=-100;
        Moto_state=0;
    } 
    else{
        if (Moto_state!=1){
            save_pos_zero(&hfdcan1, STEN_MOTO, POS_MODE);
            Moto_state=1;
        }
        p_angle=0.0;
    }
    sten_moto_ctrl(p_angle,4);
    
    // if(RC_data.rc.s[3]==240) n+=1;
    // if(RC_data.rc.s[1]==1807 && n==x) {
    //     save_moto_zero(); 
    //     x+=1;
    // }
    //位置-速度双环控制 2006编码器一圈大概为13000
    //板机复位
    if(RC_data.rc.s[3]==240){
        if(shoot_state==0){
        point_angle+=6500;     
        Shoot_set_sten_trigger_position(point_angle);
        reset_state=0;
        shoot_state=1;
        }
    }
    //板机发射
    else if(RC_data.rc.s[3]==1807){
        if(reset_state==0){
        point_angle+=6500;
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

    //YAW轴电机控制
    Shoot_set_yaw_root_velocity(RC_data.rc.ch[3]/4);
    }
}
