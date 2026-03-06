#ifndef LAUNCH_H
#define LAUNCH_H

#include "fei_bao_types.h"

void vision(int MIN_SPEED, int MAX_SPEED, int ERROR_SPEED);
void Manual_mode(void);
void Auto_mode(void);
void Fei_Bao_motor_init(void);
void test(void);

extern double cnt_angle;
extern long point_angle;

extern uint8_t shoot_state;
extern uint8_t reset_state;

extern FlingState_t fling_state;

extern const uint32_t SERVO_DELAY_MS;      // 舵机延时
extern const uint32_t TRIGGER_DELAY_MS;    // 拨弹轮等待延时
extern const uint32_t DOWN_AGAIN_MS;              // 再次下降延时
extern const double SMALL_P;               // 近距发射
extern const double BIG_P;                 // 远距发射
extern const double LOW_SPEED;             // 前进低速速度
extern const double STEN_MOTOR_TOLERANCE;  // 10010L误差容差
extern const double MOTOR_ANGLE_TOLERANCE; // 电机角度误差容差，单位rad

#endif
