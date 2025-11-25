/*
 * @Date: 2025-09-21 18:40:10
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-04 08:52:50
 * @FilePath: \Season-26-Code\User\Software\Gimbal\Gimbal_Direct.h
 */
#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)

#ifndef __GIMBAL_DIRECT_H__
#define __GIMBAL_DIRECT_H__

#include "CAN_receive_send.h"

#include "motor.h"

/*函数*/
#define GIMBALMotor_init(type, id)                        DMMotor_init(type, id) 
#define GIMBALMotor_set(id, pos, vel, tor, kp, kd)        DMMotor_set(id, pos, vel, tor, kp, kd)
#define GIMBALMotor_get_data(id)                          DMMotor_get_data(id)
#define GIMBALMotor_setzero(angle, id)                    DMMotor_setzero(angle, id)

/*电机参数*/
#define GIMBALMOTOR_MAX_CURRENT MAX_CURRENT


void Gimbal_init();
void Gimbal_control();

#endif 
#endif
