#include "launch.h"
#include "need_h.h"
#include "reload.h"
#include "motor.h"
#include "math.h"
#include "referee_system.h"

#define test_p 1 // 1:正常模式  2:夹爪上电重装模式

/* ==================== 常量定义 ==================== */

// DM电机找零参数
#define DM_ZERO_LOW       0.95f   // DM电机找零位置下限
#define DM_ZERO_HIGH      1.05f   // DM电机找零位置上限

// 遥控器
#define STICK_DEADZONE    200     // 摇杆死区
#define SW_POS_DOWN       240     // 拨杆下位值
#define SW_POS_UP         1807    // 拨杆上位值

/* ==================== 全局状态变量 ==================== */

long point_angle = 0;
uint8_t shoot_state = 1;
uint8_t reset_state = 0;

double cnt_angle = 0.0;
static int zero = 0;
uint32_t current_time;

// 三个状态机上下文
EnergyFSM_t energy_fsm = {FLING_IDLE, 0, 0.0};
ReloadFSM_t reload_fsm = {reload_non_task, 0};
FireFSM_t   fire_fsm   = {Stop_Fire, 0};

/* ==================== 调参区 ==================== */

uint32_t SERVO_DELAY_MS = PARAM_SERVO_DELAY_MS;     // 放镖延时
uint32_t DOWN_AGAIN_MS = PARAM_DOWN_AGAIN_MS;       // 再次下降延时
uint32_t TRIGGER_DELAY_MS = PARAM_TRIGGER_DELAY_MS; // 扳机扣动延时

double PULL_P = PARAM_PULL_POSITION; // 上镖位置
double WAIT_P = PARAM_WAIT_POSITION; // 等待上移位置

double Fir_P = PARAM_FIR_SHOOT_POSITION; // 第一发发射位置
double Sec_P = PARAM_SEC_SHOOT_POSITION; // 第二发发射位置
double Thr_P = PARAM_THR_SHOOT_POSITION; // 第三发发射位置
double Fou_P = PARAM_FOU_SHOOT_POSITION; // 第四发发射位置

double LOW_SPEED = PARAM_STEN_DECEL_ZONE;           // 提前减速区间
double STEN_MOTOR_TOLERANCE = PARAM_STEN_TOLERANCE;  // 10010L储能容差

double MOTOR_ANGLE_TOLERANCE = PARAM_LZ_ANGLE_TOLERANCE; // 灵足电机角度容差

double BACK_P = PARAM_BACK_POSITION; // 储能电机回勾位置

float LZ_HOME = PARAM_LZ_HOME_OFFSET;            // 灵足归原位偏移
float LZ_CATCH_POS = PARAM_LZ_CATCH_POS;         // 灵足夹镖偏移
float LZ_CLAMP_POS = PARAM_LZ_CLAMP_POS;         // 灵足夹镖下压偏移
float LZ_PUSH_POS = PARAM_LZ_PUSH_POS;           // 灵足推镖下压偏移
float LZ_HALF_DOWN_POS = PARAM_LZ_HALF_DOWN_POS; // 灵足推镖半降位置

float MID_SLIDE_LEFT = PARAM_MID_SLIDE_LEFT_POS;   // MID电机左移位置
float MID_SLIDE_RIGHT = PARAM_MID_SLIDE_RIGHT_POS; // MID电机右移位置
float MID_HOME = PARAM_MID_HOME_POS;               // MID电机归中位置
float LR_HOME = PARAM_LR_HOME_POS;                 // 左右电机归位位置

/* ==================== 电机初始化 ==================== */

void Fei_Bao_motor_init(void)
{
    DJIMotor_init(DJI_M3508, YAW_ROOT);
    DJIMotor_init(DJI_M2006, TRIGGER);

    LZMotor_position_init(MID_MOTO);
    LZMotor_position_init(RIGHT_MOTO);
    LZMotor_position_init(LEFT_MOTO);
}

/* ==================== 手动模式 ==================== */

void Manual_mode(void)
{
    switch (game_status.game_progress)
    {
    case 3:
        reset_all_fsm();
        break;
    default:
        break;
    }

    if (RC_data.online != -1 && RC_data.rc.s[0] != SW_POS_DOWN)
    {
        // 储能电机手动控制
        if (RC_data.rc.ch[1] > STICK_DEADZONE)
            sten_moto_spctrl(2);
        else if (RC_data.rc.ch[1] < -STICK_DEADZONE)
            sten_moto_spctrl(-2);
        else
            sten_moto_spctrl(0);

        // 扳机控制
        if (RC_data.rc.s[3] == SW_POS_DOWN)
            Trigger_up();
        if (RC_data.rc.s[3] == SW_POS_UP)
            Trigger_down();

        // YAW轴
        yaw_control();
    }
}

/* ==================== 自动模式 ==================== */

#if (test_p == 1)

void test(void) {}

void Auto_mode(void)
{
    // YAW轴控制
    yaw_control();

    // 扳机控制
    if (RC_data.rc.s[3] == SW_POS_DOWN && dart_client_cmd.dart_launch_opening_status == 0)
        Trigger_up();
    if (RC_data.rc.s[3] == SW_POS_UP && dart_client_cmd.dart_launch_opening_status == 0 && fire_fsm.state != Stop_Fire)
        Trigger_down();

    current_time = HAL_GetTick();

    /* ---------- 找零 & 多圈处理 ---------- */
    if (zero == 0)
    {
        find_zero();
        float pos = DM_Motor_data[0][0].motor_data.para.pos;
        if (pos >= DM_ZERO_LOW && pos <= DM_ZERO_HIGH && pos != 0.0f)
        {
            energy_fsm.state = FLING_IDLE;
            find_zero();
            zero = 1;
        }
    }
    else if (zero == 1)
    {
        multi_turn_update();
    }

    if (zero != 1)
        return; // 未完成找零，不运行状态机

    /* ---------- 比赛阶段检查 ---------- */
    switch (game_status.game_progress)
    {
    case 3:
        reset_all_fsm();
        break;
    default:
        break;
    }

    /* ================== 储能状态机 ================== */
    switch (energy_fsm.state)
    {
    case FLING_IDLE:
        energy_fsm.state      = Reset_Motor;
        energy_fsm.start_time = current_time;
        break;

    case Reset_Motor:
        sten_moto_spctrl(0);
        LZMotor_set_pos_param(LEFT_MOTO, LR_HOME, 5);
        LZMotor_set_pos_param(RIGHT_MOTO, LR_HOME, 5);
        LZMotor_set_pos_param(MID_MOTO, MID_HOME, 5);
        C_dart();
        if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - LR_HOME) < MOTOR_ANGLE_TOLERANCE &&
            fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - LR_HOME) < MOTOR_ANGLE_TOLERANCE &&
            fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - MID_HOME) < MOTOR_ANGLE_TOLERANCE)
        {
            energy_fsm.state      = Fir_Ready;
            energy_fsm.start_time = current_time;
        }
        break;

    case Fir_Ready: // 第一次储能
        if (position_ready(PULL_P))
            energy_fsm.state = Fir_Shoot_Ready;
        break;

    case Fir_Shoot_Ready:
        if (position_ready(Fir_P))
            fire_fsm.state = Fir_Fire;
        break;

    case Fir_Back:
        if (back() && reset_state == 0 && shoot_state == 1)
            energy_fsm.state = Sec_Ready;
        break;

    case Sec_Ready: // 等待放镖 + 等待装填半降完成
        if (position_ready(PULL_P) && reload_fsm.state == Fir_Half_Down_Finish)
        {
            reload_fsm.state     = Fir_Down;
            energy_fsm.keep_pos  = cnt_angle;
            energy_fsm.state     = Keep_position;
        }
        break;

    case Fir_Wait: // 储能电机下拉，等待灵足上升
        if (position_ready(WAIT_P) && reload_fsm.state == reload_non_task)
        {
            reload_fsm.state     = Fir_up;
            energy_fsm.keep_pos  = cnt_angle;
            energy_fsm.state     = Keep_position;
        }
        break;

    case Sec_Shoot_Ready:
        if (position_ready(Sec_P))
            fire_fsm.state = Sec_Fire;
        break;

    case Sec_Back:
        if (back() && reset_state == 0 && shoot_state == 1)
            energy_fsm.state = Thr_Ready;
        break;

    case Thr_Ready: // 第三次储能，等待左侧装填完成
        if (position_ready(PULL_P) && reload_fsm.state == Sec_Half_Down_Finish)
        {
            reload_fsm.state     = Sec_Down;
            energy_fsm.keep_pos  = cnt_angle;
            energy_fsm.state     = Keep_position;
        }
        break;

    case Sec_Wait:
        if (position_ready(WAIT_P) && reload_fsm.state == reload_non_task)
        {
            reload_fsm.state     = Sec_Up;
            energy_fsm.keep_pos  = cnt_angle;
            energy_fsm.state     = Keep_position;
        }
        break;

    case Thr_Shoot_Ready:
        if (position_ready(Thr_P))
            fire_fsm.state = Thr_Fire;
        break;

    case Thr_Back:
        if (back() && reset_state == 0 && shoot_state == 1)
            energy_fsm.state = Fou_Ready;
        break;

    case Fou_Ready: // 第四次储能，等待右侧装填完成
        if (position_ready(PULL_P) && reload_fsm.state == Thr_Half_Down_Finish)
        {
            reload_fsm.state     = Thr_Down;
            energy_fsm.keep_pos  = cnt_angle;
            energy_fsm.state     = Keep_position;
        }
        break;

    case Thr_Wait:
        if (position_ready(WAIT_P) && reload_fsm.state == reload_non_task)
        {
            reload_fsm.state     = Thr_Up;
            energy_fsm.keep_pos  = cnt_angle;
            energy_fsm.state     = Keep_position;
        }
        break;

    case Fou_Shoot_Ready:
        if (position_ready(Fou_P))
            fire_fsm.state = Fou_Fire;
        break;

    case Fou_Back:
        if (back() && reset_state == 0 && shoot_state == 1)
        {
            energy_fsm.keep_pos = cnt_angle;
            energy_fsm.state    = Keep_position;
        }
        break;

    case Keep_position:
        position_ready(energy_fsm.keep_pos);
        break;

    case energy_non_task:
        break;

    default:
        // 异常状态，回到空闲
        energy_fsm.state = FLING_IDLE;
        break;
    }

    /* ================== 装填状态机 ================== */
    switch (reload_fsm.state)
    {
    case reload_non_task:
        break;

    /* --- 第一次装填 --- */
    case Fir_Half_Down:
        if (LZ_double_motor_ctrl(LZ_HALF_DOWN_POS))
            reload_fsm.state = Fir_Half_Down_Finish;
        break;

    case Fir_Half_Down_Finish:
        /* 等待储能状态机汇合 (Sec_Ready 检测此状态) */
        break;

    case Fir_Down:
        if (LZ_double_motor_ctrl(LZ_PUSH_POS))
            reload_fsm.state = Fir_Push;
        break;

    case Fir_Push:
        P_dart();
        if (current_time - reload_fsm.start_time >= SERVO_DELAY_MS)
        {
            energy_fsm.state      = Fir_Wait;
            reload_fsm.state      = reload_non_task;
            reload_fsm.start_time = current_time;
        }
        break;

    case Fir_up:
        if (LZ_double_motor_ctrl(LZ_HOME))
        {
            if (current_time - reload_fsm.start_time >= TRIGGER_DELAY_MS)
            {
                energy_fsm.state      = Sec_Shoot_Ready;
                reload_fsm.state      = reload_non_task;
                reload_fsm.start_time = current_time;
            }
        }
        break;

    /* --- 左侧取镖 (第二发装填) --- */
    case L_Go:
        P_dart();
        LZMotor_set_pos_param(MID_MOTO, MID_SLIDE_LEFT, 5);
        if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - MID_SLIDE_LEFT) < MOTOR_ANGLE_TOLERANCE)
        {
            reload_fsm.state      = L_Down;
            reload_fsm.start_time = current_time;
        }
        break;

    case L_Down:
        if (LZ_double_motor_ctrl(LZ_CATCH_POS))
            reload_fsm.state = L_Catch;
        break;

    case L_Catch:
        C_dart();
        if (current_time - reload_fsm.start_time >= DOWN_AGAIN_MS)
        {
            reload_fsm.state      = L_Down_Again;
            reload_fsm.start_time = current_time;
        }
        break;

    case L_Down_Again:
        if (LZ_double_motor_ctrl(LZ_CLAMP_POS))
            reload_fsm.state = L_Up;
        break;

    case L_Up:
        if (LZ_double_motor_ctrl(LZ_HOME))
            reload_fsm.state = L_Back;
        break;

    case L_Back:
        LZMotor_set_pos_param(MID_MOTO, MID_HOME, 5);
        if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - MID_HOME) < MOTOR_ANGLE_TOLERANCE)
        {
            reload_fsm.state      = Sec_Half_Down;
            reload_fsm.start_time = current_time;
        }
        break;

    /* --- 第二次装填 --- */
    case Sec_Half_Down:
        if (LZ_double_motor_ctrl(LZ_HALF_DOWN_POS))
            reload_fsm.state = Sec_Half_Down_Finish;
        break;

    case Sec_Half_Down_Finish:
        /* 等待储能状态机汇合 (Thr_Ready 检测此状态) */
        break;

    case Sec_Down:
        if (LZ_double_motor_ctrl(LZ_PUSH_POS))
            reload_fsm.state = Sec_Push;
        break;

    case Sec_Push:
        P_dart();
        if (current_time - reload_fsm.start_time >= SERVO_DELAY_MS)
        {
            energy_fsm.state      = Sec_Wait;
            reload_fsm.state      = reload_non_task;
            reload_fsm.start_time = current_time;
        }
        break;

    case Sec_Up:
        if (LZ_double_motor_ctrl(LZ_HOME))
        {
            energy_fsm.state = Thr_Shoot_Ready;
            reload_fsm.state = reload_non_task;
        }
        break;

    /* --- 右侧取镖 (第三发装填) --- */
    case R_Go:
        P_dart();
        LZMotor_set_pos_param(MID_MOTO, MID_SLIDE_RIGHT, 5);
        if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - MID_SLIDE_RIGHT) < MOTOR_ANGLE_TOLERANCE)
        {
            reload_fsm.state      = R_Down;
            reload_fsm.start_time = current_time;
        }
        break;

    case R_Down:
        if (LZ_double_motor_ctrl(LZ_CATCH_POS))
            reload_fsm.state = R_Catch;
        break;

    case R_Catch:
        C_dart();
        if (current_time - reload_fsm.start_time >= DOWN_AGAIN_MS)
        {
            reload_fsm.state      = R_Down_Again;
            reload_fsm.start_time = current_time;
        }
        break;

    case R_Down_Again:
        if (LZ_double_motor_ctrl(LZ_CLAMP_POS))
            reload_fsm.state = R_Up;
        break;

    case R_Up:
        if (LZ_double_motor_ctrl(LZ_HOME))
            reload_fsm.state = R_Back;
        break;

    case R_Back:
        LZMotor_set_pos_param(MID_MOTO, MID_HOME, 5);
        if (fabs(GET_LZ_MOTOR_ANGLE(MID_MOTO) - MID_HOME) < MOTOR_ANGLE_TOLERANCE)
        {
            reload_fsm.state      = Thr_Half_Down;
            reload_fsm.start_time = current_time;
        }
        break;

    /* --- 第三次装填 --- */
    case Thr_Half_Down:
        if (LZ_double_motor_ctrl(LZ_HALF_DOWN_POS))
            reload_fsm.state = Thr_Half_Down_Finish;
        break;

    case Thr_Half_Down_Finish:
        /* 等待储能状态机汇合 (Fou_Ready 检测此状态) */
        break;

    case Thr_Down:
        if (LZ_double_motor_ctrl(LZ_PUSH_POS))
            reload_fsm.state = Thr_Push;
        break;

    case Thr_Push:
        P_dart();
        if (current_time - reload_fsm.start_time >= SERVO_DELAY_MS)
        {
            energy_fsm.state      = Thr_Wait;
            reload_fsm.state      = reload_non_task;
            reload_fsm.start_time = current_time;
        }
        break;

    case Thr_Up:
        LZMotor_set_pos_param(LEFT_MOTO, LR_HOME, 5);
        LZMotor_set_pos_param(RIGHT_MOTO, LR_HOME, 5);
        if (fabs(GET_LZ_MOTOR_ANGLE(LEFT_MOTO) - LR_HOME) < MOTOR_ANGLE_TOLERANCE &&
            fabs(GET_LZ_MOTOR_ANGLE(RIGHT_MOTO) - LR_HOME) < MOTOR_ANGLE_TOLERANCE)
        {
            energy_fsm.state      = Fou_Shoot_Ready;
            reload_fsm.state      = reload_non_task;
            reload_fsm.start_time = current_time;
        }
        break;

    default:
        break;
    }

    /* ================== 火控状态机 ================== */
    switch (fire_fsm.state)
    {
    case Stop_Fire:
        break;

    case Fir_Fire:
        fire_round_handler(Fir_Back, Fir_Half_Down);
        break;

    case Sec_Fire:
        fire_round_handler(Sec_Back, L_Go);
        break;

    case Thr_Fire:
        fire_round_handler(Thr_Back, R_Go);
        break;

    case Fou_Fire:
        fire_round_handler(Fou_Back, reload_non_task);
        break;

    default:
        fire_fsm.state = Stop_Fire;
        break;
    }
}

#endif /* test_p == 1 */

/* ==================== 夹爪调试模式 ==================== */

#if (test_p == 0)

void test(void) {}
void Auto_mode(void) {}

#endif

#if (test_p == 2)

void Auto_mode(void) {}

void test(void)
{
    set_servo_angle(PWM_PIN_1, 88);
    set_servo_angle(PWM_PIN_2, 70);
}

#endif
