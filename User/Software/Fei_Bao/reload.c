#include "reload.h"
#include "launch.h"
#include "need_h.h"
void Trigger_down(void)
{
  // 板机发射

  if (reset_state == 0)
  {
    point_angle += 13000;
    Shoot_set_sten_trigger_position(point_angle);
    reset_state = 1;
    shoot_state = 0;
  }
}

void Trigger_up(void)
{ // 扳机复位
  // 位置-速度双环控制 2006编码器一圈大概为13000
  // 板机复位

  if (shoot_state == 0)
  {
    point_angle += 2000;
    Shoot_set_sten_trigger_position(point_angle);
    reset_state = 0;
    shoot_state = 1;
  }
}

void sten_moto_ctrl(float angle, float val)
{ // 设定储能电机J10010L的 角度 以及 速度限幅【位置速度模式】

  pos_ctrl(&hfdcan1, STEN_MOTO, angle, val); // 发多个包，减少丢包产生的延迟
  pos_ctrl(&hfdcan1, STEN_MOTO, angle, val);
  pos_ctrl(&hfdcan1, STEN_MOTO, angle, val);
  pos_ctrl(&hfdcan1, STEN_MOTO, angle, val);
  pos_ctrl(&hfdcan1, STEN_MOTO, angle, val);
  pos_ctrl(&hfdcan1, STEN_MOTO, angle, val);
}

void sten_moto_spctrl(float val)
{

  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  osDelay(1);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  osDelay(1);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  osDelay(1);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  osDelay(1);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
  spd_ctrl(&hfdcan1, STEN_MOTO, val);
}

void find_zero(void)
{

  if (DM_Motor_data[0][0].motor_data.para.pos < 1.01)
  {
    sten_moto_spctrl(1);
  }
  else if (DM_Motor_data[0][0].motor_data.para.pos > 0.99)
  {
    sten_moto_spctrl(-1);
  }
  else
  {
    sten_moto_spctrl(0);
  }
}

/**
 * @brief 恒速控制电机到达指定的多圈累计角度
 * * @param target_angle  目标角度 (float): 你希望电机停下的多圈累计位置
 * @param speed         旋转速度 (float): 运行时的恒定速度值 (应为正数)
 * @param deadzone      停止死区 (float): 允许的误差范围。例如 0.05 代表距离目标正负 0.05 以内就停止
 */
void p_ctrl(float target_angle, float speed, float deadzone)
{
  // 1. 计算当前位置与目标位置的偏差
  // 注意：这里的 cnt_angle 是你第一段代码计算出的全局多圈角度
  float error = target_angle - cnt_angle;

  // 2. 逻辑判断
  if (error > deadzone)
  {
    // 情况 A: 当前位置在目标左侧，且距离超过死区 -> 正向全速旋转
    sten_moto_spctrl(speed);
  }
  else if (error < -deadzone)
  {
    // 情况 B: 当前位置在目标右侧，且距离超过死区 -> 反向全速旋转
    // 使用 -speed 确保方向正确
    sten_moto_spctrl(-speed);
  }
  else
  {
    // 情况 C: 进入死区范围 -> 立即停止
    // 防止电机在目标点附近因为微小偏差来回震荡
    sten_moto_spctrl(0.0f);
  }
}

void save_moto_zero()
{

  save_pos_zero(&hfdcan1, STEN_MOTO, POS_MODE);
}

void small_position_ready(FlingState_t next_mode)
{
  if (cnt_angle > SMALL_P + LOW_SPEED)
  {
    sten_moto_spctrl(-5); // 在远离区间高速
  }
  else
  {
    sten_moto_spctrl(-2);                                 // 在靠近区间低速
    if (fabs(cnt_angle - SMALL_P) < STEN_MOTOR_TOLERANCE) // 位置偏差小于容差
    {
      sten_moto_spctrl(0);
      fling_state = next_mode;
    }
  }
}

void back(FlingState_t next_mode)
{
  if (cnt_angle < 1.1 - LOW_SPEED)
  {
    sten_moto_spctrl(5); // 在远离区间高速
  }
  else
  {
    sten_moto_spctrl(2);                            // 在靠近区间低速
    if (fabs(cnt_angle - 1.1) < STEN_MOTOR_TOLERANCE) // 位置偏差小于容差
    {
      sten_moto_spctrl(0);
      fling_state = next_mode;
    }
  }
}

void P_dart(void)
{
  set_servo_angle(PWM_PIN_1, 120);
}
void C_dart(void)
{
  set_servo_angle(PWM_PIN_1, 80);
}
