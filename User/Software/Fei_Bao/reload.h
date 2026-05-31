#ifndef RELOAD_H
#define RELOAD_H

#include "fei_bao_types.h"

void Trigger_up(void);
void Trigger_down(void);
void save_moto_zero();
void sten_moto_ctrl(float angle, float val);
void sten_moto_spctrl(float val);
void find_zero(void);
void p_ctrl(float target_angle, float speed, float deadzone);
void P_dart(void);
void C_dart(void);
void vision(int MIN_SPEED, int MAX_SPEED, int ERROR_SPEED);

uint8_t LZ_double_motor_ctrl(float pos);
uint8_t position_ready(double target);
uint8_t back();

void reset_all_fsm(void);
void yaw_control(void);
void multi_turn_update(void);
void fire_round_handler(Energy_State next_energy, ReloadState_t next_reload);

#endif
