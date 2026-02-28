#include "Servos.h"
#include "tim.h"

void PWM_control_init(void)
{
    // 动态重配置定时器周期，适配舵机所需的 50Hz (20ms) 频率
    // 底层配置为 2ms 周期 (ARR=1999)，无法满足 2.5ms 脉宽需求
    // 这里将其修改为 20ms 周期 (ARR=19999)，假设 tick 为 1us
    __HAL_TIM_SET_AUTORELOAD(&htim1, 19999);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 19999);

	HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim2);	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

}
//设定舵机角度
void set_servo_angle(uint8_t channel, float angle) //统一给180舵机
{
	uint16_t CCR = (2000.0 / 180.0) * angle + 500;
	switch (channel)
	{
	case PWM_PIN_1:
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, CCR);//从下往上第一个
		break;
	case PWM_PIN_2:
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, CCR);//被串口占用了  从下往上第二个
		break;
	case PWM_PIN_3:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, CCR);//从下往上第三个
		break;
	case PWM_PIN_4:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, CCR);//从下往上第四个
		break;
	}
}

// 360度连续旋转舵机控制
// ctrl_val: -100 (反向最大) ~ 0 (停止) ~ 100 (正向最大)
// 对应PWM: 2.5ms ~ 1.5ms ~ 0.5ms
void set_360_servo_ctrl(uint8_t channel, int ctrl_val)
{
	// 基础中心值 1500us (1.5ms)
	// ctrl_val = 100 -> 目标 500us -> 1500 - 1000
	// ctrl_val = -100 -> 目标 2500us -> 1500 - (-1000) = 2500
	int32_t ccr_val = 1500 - (ctrl_val * 10);
	
	// 饱和限制，防止超出舵机物理范围 (500-2500)
	// 虽然用户说不要限位，但硬件PWM必须有上下限防止溢出或无效信号
	if (ccr_val < 500) ccr_val = 500;
	if (ccr_val > 2500) ccr_val = 2500;

	switch (channel)
	{
	case PWM_PIN_1:
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, (uint16_t)ccr_val);
		break;
	case PWM_PIN_2:
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, (uint16_t)ccr_val);
		break;
	case PWM_PIN_3:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)ccr_val);
		break;
	case PWM_PIN_4:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint16_t)ccr_val);
		break;
	}
}