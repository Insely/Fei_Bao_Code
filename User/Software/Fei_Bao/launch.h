#ifndef LAUNCH_H
#define LAUNCH_H

#include "fei_bao_types.h"

#define GET_LZ_MOTOR_ANGLE(motor_id) (LZ_Motors[motor_id / QUANTITY_OF_LZMOTOR][motor_id % QUANTITY_OF_LZMOTOR].state.angle)

void vision(int MIN_SPEED, int MAX_SPEED, int ERROR_SPEED);
void Manual_mode(void);
void Auto_mode(void);
void Fei_Bao_motor_init(void);
void test(void);

extern double cnt_angle;
extern long point_angle;

extern uint8_t shoot_state;
extern uint8_t reset_state;

extern EnergyFSM_t energy_fsm;
extern ReloadFSM_t reload_fsm;
extern FireFSM_t   fire_fsm;

extern uint32_t current_time;
extern uint32_t SERVO_DELAY_MS;   // 鑸垫満寤舵椂
extern uint32_t TRIGGER_DELAY_MS; // 鎷ㄥ脊杞?绛夊緟寤舵??
extern uint32_t DOWN_AGAIN_MS;    // 鍐嶆?′笅闄嶅欢鏃?
extern double PULL_P;             // 鏀鹃晼浣嶇疆
extern double WAIT_P;
extern double Fir_P;                 // 第一发发射位置
extern double Sec_P;                 // 第二发发射位置
extern double Thr_P;                 // 第三发发射位置
extern double Fou_P;                 // 第四发发射位置
extern double LOW_SPEED;
extern double STEN_MOTOR_TOLERANCE;
extern double MOTOR_ANGLE_TOLERANCE;
extern double BACK_P;

extern float LZ_HOME;
extern float LZ_CLAMP_POS;
extern float LZ_PUSH_POS;
extern float LZ_HALF_DOWN_POS;

extern float MID_SLIDE_LEFT;
extern float MID_SLIDE_RIGHT;
extern float MID_HOME;
extern float LR_HOME;

#endif
