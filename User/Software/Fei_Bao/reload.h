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

void small_position_ready(FlingState_t next_mode);
void back(FlingState_t next_mode);

#endif
