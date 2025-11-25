/*
 * @Date: 2025-09-21 18:40:08
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-04 21:35:04
 * @FilePath: \Season-26-Code\User\Software\Gimbal\Gimbal_Direct.c
 */
#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)

#include "Gimbal_Direct.h"
#include "Gimbal.h"
#include "Global_status.h"

#include "User_math.h"
#include "pid.h"
#include "ramp_generator.h"

#include "IMU_updata.h"
#include "dm_imu.h"
/*pid*/
pid_t pitch_speed_pid, pitch_location_pid;
pid_t yaw_speed_pid, yaw_location_pid;
pid_t pitch_auto_speed_pid, pitch_auto_location_pid;
pid_t yaw_auto_speed_pid, yaw_auto_location_pid;
/*斜坡*/
RampGenerator pitch_ramp, yaw_ramp;

extern float W_now;
/**
 * @brief 云台初始化
 *
 */
void Gimbal_init()
{
    /*电机初始化*/
    GIMBALMotor_init(GIMBAL_YAW_MOTOR_TYPE, YAWMotor);
    GIMBALMotor_init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);

    /*PID速度环初始化*/
    pid_set(&pitch_speed_pid, 0.1, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&yaw_speed_pid, 0.48f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    // 自瞄
    pid_set(&pitch_auto_speed_pid, 100.0f, 0, 0, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&yaw_auto_speed_pid, 200.0f, 0.0f, 5.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    pid_set(&pitch_location_pid, 1.0f, 0.0f, 0.5, GIMBALMOTOR_MAX_CURRENT, 10);
    pid_set(&yaw_location_pid, 1.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);
    // 自瞄
    pid_set(&pitch_auto_location_pid, 30.0f, 0.05f, 0, GIMBALMOTOR_MAX_CURRENT, 100);
    pid_set(&yaw_auto_location_pid, 15.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 1000);

    // 达妙
    GIMBALMotor_setzero(YAW_ZERO, YAWMotor);
    GIMBALMotor_setzero(PITCH_ZERO, PITCHMotor);
}


#define K_YAW 1.16

void Gimbal_control()
{
    float pitch_speed, yaw_speed;
    // 非自瞄
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    { // 速度控制
        pitch_speed = pid_cal(&pitch_location_pid, imu.pitch, Global.Gimbal.input.pitch);
        if (Global.Chssis.mode != FLOW)
            yaw_speed = Global.Gimbal.input.yaw + W_now * K_YAW; // pid_cal(&yaw_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
        else
            yaw_speed = Global.Gimbal.input.yaw;
        if (Global.Chssis.mode == SPIN_P)
            yaw_speed += 0.4 * fabsf(Global.Gimbal.input.yaw);
        GIMBALMotor_set(PITCHMotor,0,0,(pid_cal(&pitch_speed_pid, -imu.gyro[1], -pitch_speed)),0,0);
        GIMBALMotor_set(YAWMotor,0,0,(pid_cal(&yaw_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed)), 0,0);
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
        Gimbal_set_yaw_angle(IMU_data.AHRS.yaw_rad_cnt * RAD_TO_DEG);
    }
    else if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == SHOOT)
    { // 射击模式位置控制
        pitch_speed = pid_cal(&pitch_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        yaw_speed = pid_cal(&yaw_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
        GIMBALMotor_set(PITCHMotor,0,0,-pid_cal(&pitch_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed),0,0);
        GIMBALMotor_set(YAWMotor,0,0,(pid_cal(&yaw_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed)), 0,0);
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    { // 自瞄
        pitch_speed = pid_cal(&pitch_auto_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        yaw_speed = pid_cal(&yaw_auto_location_pid, RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt, Global.Gimbal.input.yaw);
        GIMBALMotor_set(PITCHMotor,0,0,(-pid_cal(&pitch_auto_speed_pid, -RAD_TO_DEG * IMU_data.gyro[0], pitch_speed)),0,0 );
        GIMBALMotor_set(YAWMotor,0,0,(pid_cal(&yaw_auto_speed_pid, (cos(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * RAD_TO_DEG * IMU_data.gyro[1]), yaw_speed)),0,0 );
        Global.Auto.input.Auto_control_online--;
    }
}
#endif
