/*
 * @Date: 2025-11-29
 * @LastEditors: Insel
 * @Brief: 飞镖相关电机配置
 *
 */

// ######################### 定义区 #########################//

#ifndef FEI_BAO_PARAM_H
#define FEI_BAO_PARAM_H

#include "robot_param.h"

#define STEN_MOTO 0x01 // DM10010L储能电机

#define YAW_ROOT CAN_2_1 // Yaw轴控制电机
#define TRIGGER CAN_2_2  // 板机控制电机

#define MID_MOTO LZ_CAN_3_1   // 装填中间的电机
#define LEFT_MOTO LZ_CAN_3_2  // 装填左侧的电机
#define RIGHT_MOTO LZ_CAN_3_3 // 装填右侧的电机

// ######################### 定义区 #########################//




// ######################### 调参区 #########################//

#define PARAM_SERVO_DELAY_MS 300    // 放镖舵机等待延时 (ms)
#define PARAM_DOWN_AGAIN_MS 200     // 夹镖后再次下压延时 (ms)
#define PARAM_TRIGGER_DELAY_MS 1000 // 板机扣动等待延时 (ms)

#define PARAM_PULL_POSITION -6.9000 // 放镖位置 (rad)
#define PARAM_WAIT_POSITION -7.4000 // 等待装填上升位置，储能电机下拉 (rad)

#define PARAM_FIR_SHOOT_POSITION -6.7000 // 第一发发射位置 (rad)
#define PARAM_SEC_SHOOT_POSITION -6.7000 // 第二发发射位置 (rad)
#define PARAM_THR_SHOOT_POSITION -6.7000 // 第三发发射位置 (rad)
#define PARAM_FOU_SHOOT_POSITION -6.7000 // 第四发发射位置 (rad)

#define PARAM_STEN_DECEL_ZONE 1         // 储能电机提前减速区间 (rad)
#define PARAM_STEN_TOLERANCE 0.0100     // 储能电机到位容差 (rad)
#define PARAM_LZ_ANGLE_TOLERANCE 0.1000 // 灵足电机到位容差 (rad)

#define PARAM_BACK_POSITION        1.5    // 储能电机回勾位置 (rad)

// 灵足双轴运动参数 (LZ_double_motor_ctrl 的 pos 偏移值)
#define PARAM_LZ_HOME_OFFSET       0.0f  // 归原位偏移
#define PARAM_LZ_CATCH_POS         0.7f  // 夹镖偏移
#define PARAM_LZ_CLAMP_POS         2.0f  // 夹镖后下压偏移
#define PARAM_LZ_PUSH_POS          4.7f  // 推镖下压偏移
#define PARAM_LZ_HALF_DOWN_POS     3.5f  // 推镖半降位置

// 中间平移电机参数 (MID_MOTO 绝对位置)
#define PARAM_MID_SLIDE_LEFT_POS  -3.0f  // MID电机左移取镖位置
#define PARAM_MID_SLIDE_RIGHT_POS  4.5f  // MID电机右移取镖位置
#define PARAM_MID_HOME_POS         1.0f  // MID电机归中位置

// 左右电机归位参数
#define PARAM_LR_HOME_POS          1.0f  // 左右电机归位位置

// ######################### 调参区 #########################//

#endif