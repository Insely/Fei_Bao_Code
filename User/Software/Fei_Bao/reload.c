#include "reload.h"
#include "launch.h"
#include "need_h.h"

#define MIN_ERROR 1 // 自瞄允许偏差

// YAW轴限位
#define YAW_ANGLE_MAX     10000   // YAW轴正向限位 (编码器值)
#define YAW_ANGLE_MIN    -20000   // YAW轴反向限位 (编码器值)
#define YAW_LIMIT_SPEED   500     // YAW轴限位回退速度

// 遥控器
#define SW_POS_UP         1807    // 拨杆上位值
#define SW_THRESHOLD      1024.0f // 拨杆阈值

// 视觉跟踪
#define VISION_MIN_SPD    230     // 最低跟踪速度
#define VISION_MAX_SPD    400     // 最高跟踪速度
#define VISION_ERR_SPD    50      // 误差步进速度

// DM电机多圈参数
#define DM_FULL_RANGE     25.0f   // DM电机单圈全量程
#define DM_HALF_RANGE     12.5f   // DM电机半量程 (跳变检测阈值)



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
    point_angle += 0;
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
  // 拉弓保护：多圈累计角度超过 -5.5 时禁止继续负向运动
  if (cnt_angle < -7.1 && val < 0)
    val = 0;

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
  float pos = DM_Motor_data[0][0].motor_data.para.pos;
  if (pos >= 0.23 && pos <= 0.30)
  {
    sten_moto_spctrl(0);
  }
  else if (pos < 0.23)
  {
    sten_moto_spctrl(1);
  }
  else
  {
    sten_moto_spctrl(-1);
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


uint8_t position_ready(double target)
{
  float error = target - cnt_angle;
  float speed;

  if (fabs(error) > LOW_SPEED)
  {
    speed = (error > 0) ? 7 : -7;  // 高速向目标移动
  }
  else
  {
    speed = (error > 0) ? 1 : -1;  // 低速向目标移动
    if (fabs(error) < STEN_MOTOR_TOLERANCE) // 位置偏差小于容差
    {
      sten_moto_spctrl(0);
      return 1; // 到位
    }
  }
  energy_fsm.start_time = current_time;
  sten_moto_spctrl(speed);
  return 0; // 未到位
}


uint8_t LZ_double_motor_ctrl(float pos)
{

  float L_pos = (-pos) + (PARAM_LR_HOME_POS);
  float R_pos = (pos) + (PARAM_LR_HOME_POS);

  LZMotor_set_pos_param(LEFT_MOTO, L_pos, 5);
  LZMotor_set_pos_param(RIGHT_MOTO, R_pos, 5);
  if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - (L_pos)) < MOTOR_ANGLE_TOLERANCE &&
      fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - (R_pos)) < MOTOR_ANGLE_TOLERANCE)
  {
    return 1; // 到位
  }
  reload_fsm.start_time = current_time;
  return 0; // 未到位
}



uint8_t back()
{
  return position_ready(BACK_P);
}

void P_dart(void)
{
  set_servo_angle(PWM_PIN_1, 88);
}
void C_dart(void)
{
  set_servo_angle(PWM_PIN_1, 70);
}

// 镖架制导
void vision(int MIN_SPEED, int MAX_SPEED, int ERROR_SPEED)
{
    float yaw_err = vision_data.yaw_error;
    float abs_err = yaw_err > 0 ? yaw_err : -yaw_err;
    float speed = 0;

    if (abs_err >= MIN_ERROR)
    {
        // 最低为MIN_SPEED的速度，偏移量范围每多出75速度就增加ERROR_SPEED
        speed = MIN_SPEED + (int)(abs_err / 75.0f) * ERROR_SPEED;
        // 速度上限最多增加到MAX_SPEED
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
    }

    if (yaw_err >= 0)
    {
        Shoot_set_yaw_root_velocity(-speed);
    }
    else
    {
        Shoot_set_yaw_root_velocity(speed);
    }
}

/**
 * @brief 复位三个状态机到初始状态
 */
void reset_all_fsm(void)
{
    energy_fsm.state      = FLING_IDLE;
    energy_fsm.start_time = current_time;
    reload_fsm.state      = reload_non_task;
    reload_fsm.start_time = current_time;
    fire_fsm.state        = Stop_Fire;
    fire_fsm.start_time   = current_time;
}

/**
 * @brief YAW轴电机控制 (手动/自动模式共用)
 *        S1上位=视觉跟踪，S4高位=遥控器+限位保护，其他=直通遥控器
 */
void yaw_control(void)
{
    if (RC_data.rc.s[1] == SW_POS_UP)
    {
        vision(VISION_MIN_SPD, VISION_MAX_SPD, VISION_ERR_SPD);
    }
    else if (RC_data.rc.s[4] >= SW_THRESHOLD)
    {
        int32_t yaw_angle = DJIMotor_data[1][0].angle_cnt;
        if (yaw_angle <= YAW_ANGLE_MAX && yaw_angle >= YAW_ANGLE_MIN)
            Shoot_set_yaw_root_velocity(RC_data.rc.ch[3]);
        else if (yaw_angle >= YAW_ANGLE_MAX)
            Shoot_set_yaw_root_velocity(-YAW_LIMIT_SPEED);
        else
            Shoot_set_yaw_root_velocity(YAW_LIMIT_SPEED);
    }
    else
    {
        Shoot_set_yaw_root_velocity(RC_data.rc.ch[3]);
    }
}

/**
 * @brief DM电机多圈角度累计 (跳变检测法)
 */
void multi_turn_update(void)
{
    static double last_raw_pos = 0.0;
    static int32_t round_count = 0;

    float raw = DM_Motor_data[0][0].motor_data.para.pos;

    if (raw - last_raw_pos < -DM_HALF_RANGE)
        round_count++;
    else if (raw - last_raw_pos > DM_HALF_RANGE)
        round_count--;

    cnt_angle = (double)round_count * DM_FULL_RANGE + raw;
    last_raw_pos = raw;
}

/**
 * @brief 火控通用处理：等待扳机扣下 → 延时 → 转移到下一组状态
 * @param next_energy  开火完成后储能状态机的下一个状态
 * @param next_reload  开火完成后装填状态机的下一个状态 (reload_non_task 表示不改变)
 */
void fire_round_handler(Energy_State next_energy, ReloadState_t next_reload)
{
    if (reset_state == 0 && shoot_state == 1)
    {
        // 扳机未扣下，持续刷新时间基准
        fire_fsm.start_time = current_time;
    }
    else
    {
        // 扳机已扣下，等待延时完成
        if (current_time - fire_fsm.start_time >= TRIGGER_DELAY_MS)
        {
            fire_fsm.state   = Stop_Fire;
            energy_fsm.state = next_energy;
            if (next_reload != reload_non_task)
                reload_fsm.state = next_reload;
            fire_fsm.start_time = current_time;
        }
    }
}

