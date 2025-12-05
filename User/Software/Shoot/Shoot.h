#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "pid.h"
#include "math.h"
#include "motor.h"
#include "remote_control.h"
#include "fei_bao_param.h"
#include "Stm32_time.h"
#include "stm32h7xx_hal.h"
/*
说明：
//发射架
//	PIN1：	                        PIN3：
//	初始值CCR:2417,对应172.5度      初始值CCR:500 ,对应0    度
//	目标值CCR:500 ,对应0    度      目标值CCR:2417,对应172.5度
//	PIN2：                          PIN4：
//	初始值CCR:2417,对应172.5度      初始值CCR:500 ,对应0    度
//	目标值CCR:500 ,对应0    度      目标值CCR:2417,对应172.5度

发射架
	PIN1：	                        PIN3：
	初始值CCR:2156,对应149度      初始值CCR:511 ,对应1  度
	目标值CCR:500 ,对应0  度      初始值CCR:2156,对应149度 
	PIN2：                          PIN4：
	初始值CCR:2156,对应149度      初始值CCR:500 ,对应0  度
	目标值CCR:678 ,对应16 度      初始值CCR:2167,对应150度 



//扳机  两个舵机，安装方式舵机输出轴朝后， 左舵机应顺时钟转动，右舵机应逆时针转动
//PIN5：左舵机，												PIN6：右舵机，
//初始值1750CCR,	 对应112.5度，				初始值 500CCR，  对应    0度，        
//目标值 500CCR，  对应    0度，				目标值1750CCR,	 对应112.5度，


PIN5：左舵机，												PIN6：右舵机，
初始值778CCR,	 对应25度，				    初始值 500CCR，  对应 0度，        
目标值 500CCR，  对应 0度，				    目标值778CCR,	 对应25度，


PIN5：左舵机，												PIN6：右舵机，
初始值833CCR,	 对应30度，				    初始值 500CCR，  对应 0度，        
目标值 500CCR，  对应 0度，				    目标值833CCR,	 对应30度，



PIN7：装填阻拦舵机，												
初始值1500CCR,	 对应90度，				          
目标值 500CCR，对应 0度，				    




*/


typedef enum 
{
   POSITION=1,
	 VELOCITY,
}	Controll_mode;
extern enum shoot_control shoot_control_mode;



typedef struct 
{
    float  Set_velocity, Now_velocity, Last_velocity, velocity;
    float  Set_position, Now_position, Last_position, Set_zero_piont, position;
    Controll_mode mode;
} Motor_Control_Data_t; // 定义通用的内部结构体

struct shoot_status
{
    Motor_Control_Data_t Sten_left;
    Motor_Control_Data_t Sten_right;
    Motor_Control_Data_t Trigger;
    Motor_Control_Data_t Push_dart;
    Motor_Control_Data_t Yaw_root;
    Motor_Control_Data_t Load_joint_root;
    Motor_Control_Data_t Load_joint_mid;
    Motor_Control_Data_t Load_joint_end;
};
extern struct shoot_status shoot;

enum shoot_control
{
	  MANUAL_MODE = 0,
	  AUTO_MODE,
	  PROTECT_MODE,
};

enum State
{
	NO=50,
	YES=80,
};	

//extern	int Count_dart;          //发射飞镖计数

//extern	float  Energy_storage_angle[2];//等效镖体发射行程
//extern  float  Energy_storage_mid_angle[2];//等效镖体发射行程
//extern  float  Push_angle;                        //等效推镖行程

//extern 	enum State  Stop_board_isready;      //阻挡板

//extern 	enum State  Sten_mid_isready;      //卡扣在中间位置是否就绪 

//extern 	enum State  Sten_isready;      //储能是否就绪 

//extern	enum State  Filing_isready;    //起落架装填是否就绪 

//extern	enum State  Push_isready;      //推镖是否就绪

//extern	enum State  Trigger_isready;      //扳机是否就绪

//extern	float Real_time[20];                    //储存现在的系统时间

//extern  int Read_only_one;
extern struct shoot_status shoot;

extern	float Count_dart;//发射飞镖计数
extern	float  Energy_storage_angle[2];    //等效镖体发射行程
extern	float  Energy_storage_mid_angle[2];//等效镖体发射行程
extern  float  Push_angle;                 //等效推镖行程
extern  float  Sten_velocity , Push_dart_velocity , Trigger_velocity,Yaw_velocity ;//手动模式下电机相关电流


extern  enum State  Save_time_allow;       //是否允许保存时间
extern  enum State  ReloadNewDart_allow;

extern 	enum State  Block_servos_isready;      //阻拦舵机是否就绪 
extern	enum State  Block_servos_isongoing;    //阻拦舵机是否进行中
extern	enum State  Block_servos_iscomplete;   //阻拦舵机是否完成
extern  enum State  Block_servos_isdown;
extern  enum State  Block_servos_isup;

extern 	enum State  Sten_mid_isready;      //储能中间是否就绪 
extern	enum State  Sten_mid_isongoing;    //储能中间是否进行中
extern	enum State  Sten_mid_iscomplete;   //储能中间是否完成

extern 	enum State  Sten_isready;          //储能是否就绪 
extern	enum State  Sten_isongoing;        //储能是否进行中
extern	enum State  Sten_iscomplete;       //储能是否完成

extern	enum State  Filing_isready;        //装填是否就绪 
extern	enum State  Filing_isongoing;      //装填是否进行中
extern	enum State  Filing_iscomplete;     //装填是否完成

extern	enum State  Push_isready;          //推镖是否就绪
extern  enum State  Push_isongoing;        //推镖是否进行中
extern  enum State  Push_iscomplete;       //推镖是否完成
extern  enum State  Push_backlittle;       //推镖回退是否完成


extern	enum State  Trigger_bullet_isready;          //是否就绪
extern  enum State  Trigger_bullet_isongoing;        //是否进行中
extern  enum State  Trigger_bullet_iscomplete;       //是否完成

extern  enum State  Launch_allow;              //是否允许发射
extern  enum State  Dart_plus_allow;      //裁判系统用


//手动模式防误触操作
extern enum State  Trigger_allow;//是否允许扳机扣下
extern enum State  Sten_allow;//是否允许卡口下拉

extern	float Real_time[20];               //储存现在的系统时间
extern  float  Real_time_reload[20];               //换弹流程所用的储存现在系统时间

extern  int Read_only_one;
extern  int Refree_open_time;


extern float left_sten;
extern float right_sten;
extern float Push_2006motor;
extern float trigger_2006motor;
extern float Yaw_root_3508motor;

void Shoot_init(void);

void Shoot_pid_cal(void);
void Shoot_updata(void);

void DART_double_sten(int altitude,int delay_time_ms,float change_num);
void DART_Push_2006motor(int altitude,int delay_time_ms,float change_num);
void DART_Trigger_2006motor(int altitude,int delay_time_ms,float change_num);
void DART_Yaw_root_3508motor(int altitude,int delay_time_ms,float change_num);
DJI_motor_data_s get_CAN1_DJImotor_data(DJIcan_id motorID);
void set_CAN1_DJImotor(int16_t val, DJIcan_id motorID);

void Shoot_set_sten_position(float Sten_left, float Sten_right);
void Shoot_set_sten_velocity(float Sten_left, float Sten_right);
void Shoot_set_sten_trigger_position(float Sten_trigger);
void Shoot_set_sten_trigger_velocity(float Sten_trigger);
void Shoot_set_push_dart_position(float Push_dart);
void Shoot_set_push_dart_velocity(float Push_dart);
void Shoot_set_yaw_root_position(float Yaw_root);
void Shoot_set_yaw_root_velocity(float Yaw_root);

float Get_Base_Current(float Target_Speed, float Current_Speed_M1, float Current_Speed_M2);
void Motor_Control_Loop(int16_t set);
float pid_sync_cal_abs(pid_t *SyncGoal, float NowSpeed1, float NowSpeed2);



#endif