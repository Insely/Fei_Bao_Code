#include "reload.h"


void Trigger_down(void){//扳机扣下
  set_servo_angle(PWM_PIN_4, 110);
  set_servo_angle(PWM_PIN_3, 70);
}


void Trigger_up(void){//扳机复位
  set_servo_angle(PWM_PIN_4,90);
  set_servo_angle(PWM_PIN_3,90);
}

void sten_moto_ctrl(float angle,float val){ //设定储能电机J10010L的 角度 以及 速度限幅【位置速度模式】

        pos_ctrl(&hfdcan1, STEN_MOTO,angle,val);    //发多个包，减少丢包产生的延迟
        pos_ctrl(&hfdcan1, STEN_MOTO,angle,val);
        pos_ctrl(&hfdcan1, STEN_MOTO,angle,val);
        pos_ctrl(&hfdcan1, STEN_MOTO,angle,val);
        pos_ctrl(&hfdcan1, STEN_MOTO,angle,val);
        pos_ctrl(&hfdcan1, STEN_MOTO,angle,val);    
}

void save_moto_zero(){
  save_pos_zero(&hfdcan1, STEN_MOTO, POS_MODE);
}