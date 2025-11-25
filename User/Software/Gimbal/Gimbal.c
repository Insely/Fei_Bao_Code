#include "Gimbal.h"
#include "Global_status.h"
#include "robot_param.h"

/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_set_pitch_angle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_set_yaw_angle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}