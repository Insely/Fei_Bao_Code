#ifndef __PWM_CONTROL_H__
#define __PWM_CONTROL_H__

#include "stdint.h"

#define PWM_PIN_1 1 //舵机通道1/2/3/4
#define PWM_PIN_2 2
#define PWM_PIN_3 3
#define PWM_PIN_4 4

void PWM_control_init(void);
void set_servo_angle(uint8_t channel, float angle); //设置180舵机
void set_360_servo_ctrl(uint8_t channel, int ctrl_val); // 360速度舵机控制

#endif
