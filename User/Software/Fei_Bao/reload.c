#include "reload.h"


void Trigger_down(void){//扳机扣下
  set_servo_angle(PWM_PIN_4, 0);//0
  set_servo_angle(PWM_PIN_3, 30);//30
}


void Trigger_up(void){//扳机复位
  set_servo_angle(PWM_PIN_4,30);//30
  set_servo_angle(PWM_PIN_3, 0);//0
}
