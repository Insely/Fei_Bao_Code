#include "launch.h"



void Manual_mode(void){

    static int n=0;
    static int x=1;
    static double p_angle=0.0;
    static uint8_t Moto_state=0;
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
    sten_moto_ctrl(p_angle,2);

    if(RC_data.rc.s[1] >1800.0f){

        Trigger_down();
          
      }
    else if(RC_data.rc.s[1]<250.0f){

        Trigger_up(); 

    }

    if(RC_data.rc.s[3]<250.0f) n+=1;
    if(RC_data.rc.s[1] >1800.0f && n==x) {
        save_moto_zero(); 
        x+=1;
    }
}
