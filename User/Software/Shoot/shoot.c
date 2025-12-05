#include "shoot.h"

#define LEFT_ERROR 0;
#define RIGHT_ERROR 0;

float Count_dart=0.0f;          //发射飞镖计数
float sten_left_val=0;
float sten_right_val=0;

pid_t SyncController;
pid_t SpeedBase_Controller;

int push_dart = 1;

//修改位置


float  Energy_storage_angle[2]={2150.0f,-2150.0f};//2250   最近一次2370  2320 2250 2370
float  Energy_storage_mid_angle[2]={1380.0f,-1380.0f};//
float  Push_angle=702.0f;                        //等效推镖行程
float  Sten_velocity = 0, Push_dart_velocity = 0, Trigger_velocity = 0,Yaw_velocity = 0;



enum State  Save_time_allow=YES;          //是否允许保存时间
enum State  ReloadNewDart_allow=YES;   //是否允许换弹

enum State  Block_servos_isready=NO;      //阻拦舵机是否就绪 
enum State  Block_servos_isongoing=NO;    //阻拦舵机是否进行中
enum State  Block_servos_iscomplete=NO;   //阻拦舵机是否完成
enum State  Block_servos_isdown=NO;       //阻拦舵机下降
enum State  Block_servos_isup=NO;         //阻拦舵机上升


enum State  Sten_mid_isready=NO;      //储能中间是否就绪 
enum State  Sten_mid_isongoing=NO;    //储能中间是否进行中
enum State  Sten_mid_iscomplete=NO;   //储能中间是否完成

enum State  Trigger_bullet_isready=NO;          //是否就绪
enum State  Trigger_bullet_isongoing=NO;        //是否进行中
enum State  Trigger_bullet_iscomplete=NO;       //是否完成

enum State  Launch_allow=YES;         //是否允许发射
enum State  Dart_plus_allow=YES;      //裁判系统用

enum State  Sten_isready=NO;      //储能是否就绪 
enum State  Sten_isongoing=NO;    //储能是否进行中
enum State  Sten_iscomplete=NO;   //储能是否完成
enum State  Filing_isready=NO;    //装填是否就绪 
enum State  Filing_isongoing=NO;  //装填是否进行中
enum State  Filing_iscomplete=NO; //装填是否完成
enum State  Push_isready=NO;      //推镖是否就绪
enum State  Push_isongoing=NO;    //推镖是否进行中
enum State  Push_iscomplete=NO;   //推镖是否完成
enum State  Push_backlittle=NO;   //推镖回退是否完成
//手动模式下防误触操作
enum State  Trigger_allow=YES;//是否允许扳机扣下
enum State  Sten_allow=YES;//是否允许卡口下拉

float  Real_time[20]={0};            //储存现在的系统时间
float  Real_time_reload[20]={0};     //换弹流程所用的储存现在系统时间
int Read_only_one=0;
int Refree_open_time=0;              //裁判系统舱门开启次数



float left_sten;
float right_sten;
float Push_2006motor;
float trigger_2006motor;
float Yaw_root_3508motor;

/*
舵机与C板接口说明：
  PIN_1  装填左一
	PIN_2  装填右二
	PIN_3  扳机左一
	PIN_4  扳机右一
	PIN_5  镖体姿态矫正
舵机旋转角度与CCR：
 逆时针 面向舵盘
 500  CCR 45度
 1000 CCR 90度
 1500 CCR 135度
 2000 CCR 180度
 2500 CCR 225度
*/
// 位置控制下的PID(自动 auto)
pid_t Sten_left_speed_pid_auto;
pid_t Sten_left_location_pid_auto;

pid_t Sten_right_speed_pid_auto;
pid_t Sten_right_location_pid_auto;

pid_t Push_dart_speed_pid_auto;
pid_t Push_dart_location_pid_auto;

pid_t Trigger_speed_pid;
pid_t Trigger_location_pid;

pid_t Yaw_root_speed_pid;
pid_t Yaw_root_location_pid;

struct shoot_status shoot;

// 初始化
void Shoot_init()
{
	// 飞镖
	// pid初始化
	// 位置控制下的PID
//	pid_set(&Sten_left_speed_pid_auto, 6.0f, 0.0f, 0.0f, 20000.0f, 0.0f);//单速度环下效果很好
//	pid_set(&Sten_left_location_pid_auto, 50.0f, 0.0f, 0.0f, 8000.0f, 0.0f);

//	pid_set(&Sten_right_speed_pid_auto, 6.0f, 0.0f, 0.0f, 20000.0f, 0.0f);//单速度环下效果很好
//	pid_set(&Sten_right_location_pid_auto, 50.0f, 0.0f, 0.0f, 8000.0f, 0.0f);

	// --- 设定 PID 参数 ---
    
    // 比例项 (Kp): 负责快速响应速度差。这是同步控制中最主要的项。
    // 初始值可以设得稍大，因为只需要处理相对速度差。
    SyncController.p = 20.0f; 
    
    // 积分项 (Ki): 负责消除两个电机长期存在的、稳定的速度偏差（如由于电机个体差异）。
    // 初始值应非常小，防止过积分导致震荡。
    SyncController.i = 0.05f;
    
    // 微分项 (Kd): 负责抑制速度突变带来的震荡。对于平稳运行的同步，初始可设为0。
    SyncController.d = 0.0f;
    
    
    // --- 设定限幅和初始状态 ---
    
    // 积分项限幅 (lim_i_out): 限制积分项的最大累积值。
    // 这个值限制了同步输出能提供的最大“补偿电流”，应小于电机最大电流的一半。
    // 假设你的 M3508 电机最大电流指令是 16384，这个值可以设得较小。
    SyncController.lim_i_out = 2000.0f; 
    
    // 初始化误差和累加值
    SyncController.err = 0.0f;
    SyncController.err_last = 0.0f;
    SyncController.i_out = 0.0f;
    
    // 通常将这个函数放在 System_Init() 或 Motor_Control_Init() 中调用

	// --- 设定 PID 参数 ---
    
    // 比例项 (Kp): 负责快速响应目标速度和平均实际速度之间的误差。
    // 比同步环的Kp要大，但需要谨慎防止超调。
    // 初始值可以设在一个中等偏大的范围，需要仔细整定。
    SpeedBase_Controller.p = 30.0f; 
    
    // 积分项 (Ki): 负责消除稳态误差，提供克服摩擦和恒定负载所需的电流。
    // 初始值应较小，但大于同步环，以确保能达到目标转速。
    SpeedBase_Controller.i = 0.2f;
    
    // 微分项 (Kd): 抑制速度的瞬时震荡。
    // 如果转速反馈有噪声，初始值设为 0.0f 更安全。如果速度反馈平滑，可以略微增大。
    SpeedBase_Controller.d = 0.0f;
    
    
    // --- 设定限幅和初始状态 ---
    
    // 积分项限幅 (lim_i_out): 防止积分项无限累积（积分饱和）。
    // 设为最大电流指令值的一个合理百分比（如 80%），避免 P 项和 D 项无法起作用。
    SpeedBase_Controller.lim_i_out = 10000.0f; 
    
    // 总输出限幅 (lim_total_out): 限制最终输出给电机的电流指令值。
    // 设为 M3508 电机的最大电流指令值，以保护电调和电机。
    SpeedBase_Controller.lim_out = 16384.0f; 
    
    // 初始化状态变量
    SpeedBase_Controller.set = 0.0f; // 初始目标转速为 0
    SpeedBase_Controller.err = 0.0f;
    SpeedBase_Controller.err_last = 0.0f;
    SpeedBase_Controller.i_out = 0.0f;
    
    // 提示：这个函数应该在你的 main() 函数开始运行后，所有硬件初始化完成后调用。
	
	pid_set(&Sten_left_speed_pid_auto, 30.0f, 0.0f, 0.0f, 15000.0f, 0.0f);//单速度环下效果很好 1.5 0.15 0.3
	pid_set(&Sten_left_location_pid_auto,50.0f, 80.0f, 0.21f, 5000.0f, 0.0f);//50   8  0

	pid_set(&Sten_right_speed_pid_auto, 30.0f, 0.0f, 0.0f, 15000.0f, 0.0f);//单速度环下效果很好
	pid_set(&Sten_right_location_pid_auto, 50.0f, 80.0f,0.21f, 5000.0f, 0.0f);	
	
	pid_set(&Trigger_speed_pid, 10.0f, 0.0f, 0.0f, 12000.0f, 0.0f);//单速度环下效果很好
	pid_set(&Trigger_location_pid, 100.0f, 0, 0.0f, 10000.0f, 0.0f);

	pid_set(&Push_dart_speed_pid_auto, 3.0f, 0.0f, 0.8f, 12000.0f, 0.0f);//单速度环下效果很好
	pid_set(&Push_dart_location_pid_auto, 35.0f, 0.4f,6.0f, 10000.0f, 0.0f);//100 0 4.0    10000

	pid_set(&Yaw_root_speed_pid, 3.0f, 0.1f, 0.3f, 20000.0f, 0.0f);//单速度环下效果很好
	pid_set(&Yaw_root_location_pid, 20.0f, 0.0f, 0.0f, 10000.0f, 0.0f);	////需测试
	
	
	// 左侧-储能3508
	shoot.Sten_left.Last_position = 0;
	shoot.Sten_left.Now_position = 0;
	shoot.Sten_left.Set_position = 0;
//	shoot.Sten_left.Set_zero_piont = get_CAN1_DJImotor_data(CAN_1_1).angle_cnt;
	shoot.Sten_left.Set_zero_piont =0;
	shoot.Sten_left.position = 0;

	shoot.Sten_left.Last_velocity = 0;
	shoot.Sten_left.Now_velocity = 0;
	shoot.Sten_left.Last_velocity = 0;
	shoot.Sten_left.Set_velocity = 0;
	shoot.Sten_left.velocity = 0;
	shoot.Sten_left.mode = VELOCITY;
	
	// 右侧-储能3508
	shoot.Sten_right.Last_position = 0;
	shoot.Sten_right.Now_position = 0;
	shoot.Sten_right.Set_position = 0;
//	shoot.Sten_right.Set_zero_piont = get_CAN1_DJImotor_data(CAN_1_2).angle_cnt;
	shoot.Sten_right.Set_zero_piont = 0;
	shoot.Sten_right.position = 0;

	shoot.Sten_right.Last_velocity = 0;
	shoot.Sten_right.Now_velocity = 0;
	shoot.Sten_right.Last_velocity = 0;
	shoot.Sten_right.Set_velocity = 0;
	shoot.Sten_right.velocity = 0;
	shoot.Sten_right.mode = VELOCITY;
	
	// 扳机2006
	shoot.Trigger.Last_position = 0;
	shoot.Trigger.Now_position = 0;
	shoot.Trigger.Set_position = 0;
//	shoot.Trigger.Set_zero_piont = get_CAN1_DJImotor_data(CAN_1_3).angle_cnt;
	shoot.Trigger.Set_zero_piont = 0;
	shoot.Trigger.position = 0;

	shoot.Trigger.Last_velocity = 0;
	shoot.Trigger.Now_velocity = 0;
	shoot.Trigger.Last_velocity = 0;
	shoot.Trigger.Set_velocity = 0;
	shoot.Trigger.velocity = 0;
	shoot.Trigger.mode = VELOCITY;
	
	// 推镖2006
	shoot.Push_dart.Last_position = 0;
	shoot.Push_dart.Now_position = 0;
	shoot.Push_dart.Set_position = 0;
//	shoot.Push_dart.Set_zero_piont = get_CAN1_DJImotor_data(CAN_1_4).angle_cnt;
	shoot.Push_dart.Set_zero_piont = 0;
	shoot.Push_dart.position = 0;

	shoot.Push_dart.Last_velocity = 0;
	shoot.Push_dart.Now_velocity = 0;
	shoot.Push_dart.Last_velocity = 0;
	shoot.Push_dart.Set_velocity = 0;
	shoot.Push_dart.velocity = 0;
	shoot.Push_dart.mode = VELOCITY;
	
	// yaw轴3508	
	shoot.Yaw_root.Last_position = 0;
	shoot.Yaw_root.Now_position = 0;
	shoot.Yaw_root.Set_position = 0;
//	shoot.Yaw_root.Set_zero_piont = get_CAN1_DJImotor_data(CAN_1_5).angle_cnt;
	shoot.Yaw_root.Set_zero_piont = 0;
	shoot.Yaw_root.position = 0;

	shoot.Yaw_root.Last_velocity = 0;
	shoot.Yaw_root.Now_velocity = 0;
	shoot.Yaw_root.Last_velocity = 0;
	shoot.Yaw_root.Set_velocity = 0;
	shoot.Yaw_root.velocity = 0;
	shoot.Yaw_root.mode = VELOCITY;	

}


// 数据更新
void Shoot_updata()
{
	// 从电机获取数据
	// 左侧-储能3508更新
	shoot.Sten_left.Now_position = get_CAN1_DJImotor_data(STEN_LEFT).angle_cnt - shoot.Sten_left.Set_zero_piont;
	shoot.Sten_left.Now_velocity = get_CAN1_DJImotor_data(STEN_LEFT).round_speed;
	// 右侧-储能3508更新
	shoot.Sten_right.Now_position = get_CAN1_DJImotor_data(STEN_RIGHT).angle_cnt - shoot.Sten_right.Set_zero_piont;
	shoot.Sten_right.Now_velocity = get_CAN1_DJImotor_data(STEN_RIGHT).round_speed;
	// 扳机2006更新
	shoot.Trigger.Now_position = get_CAN1_DJImotor_data(TRIGGER).angle_cnt - shoot.Trigger.Set_zero_piont;
	shoot.Trigger.Now_velocity = get_CAN1_DJImotor_data(TRIGGER).round_speed;
	// 推镖2006更新
	shoot.Push_dart.Now_position = get_CAN1_DJImotor_data(PUSH_DART).angle_cnt - shoot.Push_dart.Set_zero_piont;
//	shoot.Push_dart.Now_position = get_CAN1_DJImotor_data(PUSH_DART).ecd_cnt;
	shoot.Push_dart.Now_velocity = get_CAN1_DJImotor_data(PUSH_DART).round_speed;
	// Yaw轴3508更新
	shoot.Yaw_root.Now_position = get_CAN1_DJImotor_data(YAW_ROOT).angle_cnt - shoot.Yaw_root.Set_zero_piont;
	shoot.Yaw_root.Now_velocity = get_CAN1_DJImotor_data(YAW_ROOT).round_speed;

}
// 设定储能3508角度
void Shoot_set_sten_position(float Sten_left, float Sten_right)
{
	shoot.Sten_left.Set_position =  Sten_left;
	shoot.Sten_right.Set_position = Sten_right;
}
// 设置储能3508速度
void Shoot_set_sten_velocity(float Sten_left, float Sten_right)
{	if(RC_data.rc.ch[1]!=0&&(Sten_left>0||Sten_right>0))
	{
	shoot.Sten_left.Set_velocity =  Sten_left+ERROR;
	shoot.Sten_right.Set_velocity = -Sten_right-RIGHT_ERROR;
	}
	else if(RC_data.rc.ch[1]!=0&&(Sten_left<0||Sten_right<0)){
	shoot.Sten_left.Set_velocity =  Sten_left-ERROR;
	shoot.Sten_right.Set_velocity = -Sten_right+RIGHT_ERROR;

	}
}
// 设置扳机2006角度
void Shoot_set_sten_trigger_position(float Sten_trigger)
{
	shoot.Trigger.Set_position = Sten_trigger;
}
// 设置扳机2006速度
void Shoot_set_sten_trigger_velocity(float Sten_trigger)
{
	shoot.Trigger.Set_velocity = Sten_trigger;
}
// 设置推镖2006角度
void Shoot_set_push_dart_position(float Push_dart)
{
	shoot.Push_dart.Set_position = Push_dart;
}
// 设置推镖2006速度
void Shoot_set_push_dart_velocity(float Push_dart)
{
	shoot.Push_dart.Set_velocity = Push_dart;
}
// 设置Yaw轴角度
void Shoot_set_yaw_root_position(float Yaw_root)
{
	shoot.Yaw_root.Set_position = Yaw_root;
}
// 设置Yaw轴速度
void Shoot_set_yaw_root_velocity(float Yaw_root)
{
	shoot.Yaw_root.Set_velocity = Yaw_root;
}


// pid计算
void Shoot_pid_cal()
{
	// 位置环 （位置控制模式下）
	if (shoot.Sten_left.mode == POSITION)
		shoot.Sten_left.velocity = pid_cal(&Sten_left_location_pid_auto, shoot.Sten_left.Now_position, shoot.Sten_left.Set_position);
	else
		shoot.Sten_left.velocity = shoot.Sten_left.Set_velocity;

	if (shoot.Sten_right.mode == POSITION)
		shoot.Sten_right.velocity = pid_cal(&Sten_right_location_pid_auto, shoot.Sten_right.Now_position, shoot.Sten_right.Set_position);
	else
		shoot.Sten_right.velocity = shoot.Sten_right.Set_velocity;

	if (shoot.Push_dart.mode == POSITION)
		//shoot.Push_dart.velocity = 500.0f;
	  shoot.Push_dart.velocity = -pid_cal(&Push_dart_location_pid_auto, shoot.Push_dart.Now_position, shoot.Push_dart.Set_position);
	else
		shoot.Push_dart.velocity = shoot.Push_dart.Set_velocity;
	
	if (shoot.Trigger.mode == POSITION)
		shoot.Trigger.velocity = -pid_cal(&Push_dart_location_pid_auto, shoot.Trigger.Now_position, shoot.Trigger.Set_position);
	else
		shoot.Trigger.velocity = shoot.Trigger.Set_velocity;

	if (shoot.Yaw_root.mode == POSITION)
		shoot.Yaw_root.velocity = pid_cal(&Yaw_root_location_pid, shoot.Yaw_root.Now_position, shoot.Yaw_root.Set_position);
	else
		shoot.Yaw_root.velocity = shoot.Yaw_root.Set_velocity;	
	
	
//		shoot.Sten_left.velocity = pid_cal(&Sten_left_location_pid_auto, shoot.Sten_left.Now_position, shoot.Sten_left.Set_position);
//		shoot.Sten_right.velocity = pid_cal(&Sten_right_location_pid_auto, shoot.Sten_right.Now_position, shoot.Sten_right.Set_position);
//		shoot.Push_dart.velocity = -pid_cal(&Push_dart_location_pid_auto, shoot.Push_dart.Now_position, shoot.Push_dart.Set_position);
//		shoot.Trigger.velocity = pid_cal(&Push_dart_location_pid_auto, shoot.Trigger.Now_position, shoot.Trigger.Set_position);
//		shoot.Yaw_root.velocity = pid_cal(&Yaw_root_location_pid, shoot.Yaw_root.Now_position, shoot.Yaw_root.Set_position);


	// 速度环
	// 左侧储能3508
		// sten_left_val=pid_cal(&Sten_left_speed_pid_auto, get_CAN1_DJImotor_data(STEN_LEFT).speed_rpm, shoot.Sten_left.velocity);
		// set_CAN1_DJImotor(sten_left_val, STEN_LEFT);

	// 右侧储能3508
		// sten_right_val=pid_cal(&Sten_right_speed_pid_auto, get_CAN1_DJImotor_data(STEN_RIGHT).speed_rpm, shoot.Sten_right.velocity);
		// set_CAN1_DJImotor(sten_right_val, STEN_RIGHT);
	// 推镖2006
	set_CAN1_DJImotor(pid_cal(&Push_dart_speed_pid_auto, -get_CAN1_DJImotor_data(PUSH_DART).speed_rpm, shoot.Push_dart.velocity), PUSH_DART);

	//扳机2006
	set_CAN1_DJImotor(pid_cal(&Trigger_speed_pid, -get_CAN1_DJImotor_data(TRIGGER).speed_rpm, shoot.Trigger.velocity), TRIGGER);

	//Yaw轴3508
	set_CAN1_DJImotor(pid_cal(&Yaw_root_speed_pid, get_CAN1_DJImotor_data(YAW_ROOT).speed_rpm, shoot.Yaw_root.velocity), YAW_ROOT);



}




void DART_double_sten(int altitude,int delay_time_ms,float change_num)   
{                                                                                                                                      
//174，                                                                                                                                                 
	static uint32_t last_random_time =0;  
  const int max = 5000;
  const int min = -5000; 	
  float *left_sten = &shoot.Sten_left.Set_position ;		                	  //从属电机
	float *right_sten = &shoot.Sten_right.Set_position; 	                	  //从属电机	                                       
  if (Get_sys_time_ms() - last_random_time > delay_time_ms)                 	  //飞镖如果 遥控器推杆方向                        
  {                                                                         	  //和 双电机推镖上升方向一致的话
		last_random_time = Get_sys_time_ms();                               	  //左电机顺时针转动，右电机逆时针转动，
		if(*left_sten>=-max&&*left_sten<=-min&&*right_sten<=max&&*right_sten>=min)//即 遥控器值 和 电机需求值 相反 表现为 左负右正
		{
			if(altitude > 0&&*left_sten!=-max&&*left_sten!=max)                        
			{
				*left_sten -=change_num;
				*right_sten +=change_num;
			}
			if(altitude < 0&&*left_sten!=-min&&*right_sten!=min)
			{
				*left_sten +=change_num;
				*right_sten -=change_num;
			}
		}
		else if(*left_sten<-max&&*right_sten>max)
		{
			*left_sten=-max;
			*right_sten=max;
		}
		else if(*left_sten>-min&&*right_sten<min)
		{
			*left_sten=-min;
			*right_sten=min;	
		}


	}

}




void DART_Push_2006motor(int altitude,int delay_time_ms,float change_num)
{
//174，
	static uint32_t last_time =0;
//	static float yaw_set=0;
  const int max = 1000;
  const int min = -1000;
	float *Push_2006motor = &shoot.Push_dart.Set_position;
	
  if (Get_sys_time_ms() - last_time > delay_time_ms)
  {
		last_time = Get_sys_time_ms();
		if(*Push_2006motor<=max&&*Push_2006motor>=min)
		{
			if(altitude > 0&&*Push_2006motor!=max)
			{
				*Push_2006motor +=change_num;
			}
			if(altitude < 0&&*Push_2006motor!=min)
			{
				*Push_2006motor -=change_num;
			}
		}
		else if(*Push_2006motor>max)
		{
			*Push_2006motor=max;
		}
		else if(*Push_2006motor<min)
		{
			*Push_2006motor=min;	
		}
	}
}



void DART_Trigger_2006motor(int altitude,int delay_time_ms,float change_num)
{
//174，
	static uint32_t last_time =0;
//	static float yaw_set=0;
	const int max = 5000;
	const int min = -5000;
	float *trigger_2006motor = &shoot.Trigger.Set_position;
	
  if (Get_sys_time_ms() - last_time > delay_time_ms)
  {
		last_time = Get_sys_time_ms();
		if(*trigger_2006motor<=max&&*trigger_2006motor>=min)
		{
			if(altitude > 0&&*trigger_2006motor!=max)
			{
				*trigger_2006motor +=change_num;
			}
			if(altitude < 0&&*trigger_2006motor!=min)
			{
				*trigger_2006motor -=change_num;
			}
		}
		else if(*trigger_2006motor>max)
		{
			*trigger_2006motor=max;
		}
		else if(*trigger_2006motor<min)
		{
			*trigger_2006motor=min;	
		}
	}
}

void DART_Yaw_root_3508motor(int altitude,int delay_time_ms,float change_num)
{
//174，
	static uint32_t last_time =0;
//	static float yaw_set=0;
	const int max = 1000;
	const int min = -1000;
	float *Yaw_root_3508motor =&shoot.Yaw_root.Set_position;
	
  if (Get_sys_time_ms() - last_time > delay_time_ms)
  {
		last_time = Get_sys_time_ms();
		if(*Yaw_root_3508motor<=max&&*Yaw_root_3508motor>=min)
		{
			if(altitude > 0&&*Yaw_root_3508motor!=max)
			{
				*Yaw_root_3508motor +=change_num;
			}
			if(altitude < 0&&*Yaw_root_3508motor!=min)
			{
				*Yaw_root_3508motor -=change_num;
			}
		}
		else if(*Yaw_root_3508motor>max)
		{
			*Yaw_root_3508motor=max;
		}
		else if(*Yaw_root_3508motor<min)
		{
			*Yaw_root_3508motor=min;	
		}
	}
}


DJI_motor_data_s get_CAN1_DJImotor_data(DJIcan_id motorID){

  return DJIMotor_data[1][motorID];

}


void set_CAN1_DJImotor(int16_t val, DJIcan_id motorID) // 设定马达电流
{
  DJIMotor_data[0][motorID].set = val; // val
}






/**
 * @brief 计算两个电机速度同步误差的PID输出
 * * @param SyncGoal 同步控制器的PID结构体指针
 * @param NowSpeed1 电机1的当前转速（有符号）
 * @param NowSpeed2 电机2的当前转速（有符号）
 * @return float 计算出的电流补偿值
 */
float pid_sync_cal_abs(pid_t *SyncGoal, float NowSpeed1, float NowSpeed2)
{
    // --- 1. 取速度绝对值（关键修正点） ---
    // 假设我们希望 M1 的速度大小与 M2 的速度大小相等
    float abs_speed1 = fabs(NowSpeed1);
    float abs_speed2 = fabs(NowSpeed2);

    // --- 2. 设置目标值 (始终为 0) ---
    SyncGoal->set = 0.0; 
    
    // --- 3. 计算误差 ---
    // 误差 e = M1绝对速度 - M2绝对速度。
    // 如果 e > 0，说明 M1 比 M2 快
    SyncGoal->err_last = SyncGoal->err;
    SyncGoal->err = abs_speed1 - abs_speed2; 

    // --- 4. 计算误差微分 ---
    SyncGoal->diff = SyncGoal->err - SyncGoal->err_last;
    
    // --- 5. 计算PID三项输出 ---
    
    SyncGoal->p_out = SyncGoal->p * SyncGoal->err;
    
    if(SyncGoal->i != 0.0) {
        SyncGoal->i_out += SyncGoal->i * SyncGoal->err;
    }
    
    SyncGoal->d_out = SyncGoal->d * SyncGoal->diff;

    // --- 6. 积分限幅 (防饱和) ---
    if(fabs(SyncGoal->i_out) > SyncGoal->lim_i_out)
    {
        if(SyncGoal->i_out < 0.0)
            SyncGoal->i_out = -SyncGoal->lim_i_out;
        else
            SyncGoal->i_out = SyncGoal->lim_i_out;
    }
    
    // --- 7. 计算总输出 ---
    SyncGoal->total_out = SyncGoal->p_out + SyncGoal->i_out + SyncGoal->d_out;

    // --- 8. 总输出限幅 ---
    if(fabs(SyncGoal->total_out) > SyncGoal->lim_out)
    {
        if(SyncGoal->total_out < 0.0)
            SyncGoal->total_out = -SyncGoal->lim_out;
        else
            SyncGoal->total_out = SyncGoal->lim_out;
    }
    
    return SyncGoal->total_out;
}




void Motor_Control_Loop(int16_t set) {
    // 假设 M1 正在正转 (100.0 RPM), M2 正在反转 (-95.0 RPM)
    float M1_Speed_Actual = get_CAN1_DJImotor_data(STEN_LEFT).speed_rpm;
    float M2_Speed_Actual = get_CAN1_DJImotor_data(STEN_LEFT).speed_rpm; 
    
    float TargetCurrent_Base = Get_Base_Current(set,M1_Speed_Actual,M2_Speed_Actual); 
    
    // --- 运行同步控制器 (使用修正后的函数) ---
    // 此时 SyncOutput 根据 (fabs(100.0) - fabs(-95.0)) = 5.0 计算，得到一个正值
    float SyncOutput = pid_sync_cal_abs(&SyncController, M1_Speed_Actual, M2_Speed_Actual); 
    
    // --- 计算最终目标电流 ---
    
    // M1 绝对速度较快 (SyncOutput > 0)，需要减小电流
    float M1_Target_Current = TargetCurrent_Base - SyncOutput; 
    
    // M2 绝对速度较慢 (SyncOutput > 0)，需要增大电流
    float M2_Target_Current = -(TargetCurrent_Base + SyncOutput); 
    
    // --- 发送给电机驱动 ---
	set_CAN1_DJImotor(M1_Target_Current, STEN_LEFT);
    set_CAN1_DJImotor(M2_Target_Current, STEN_RIGHT);
}







/**
 * @brief Get_Base_Current 函数
 * * 使用位置式PID计算维持系统整体目标速度所需的电流基准线。
 * * @param Target_Speed: 期望系统达到的目标转速（例如：100 rpm）
 * @param Current_Speed_M1: 电机1的实际转速
 * @param Current_Speed_M2: 电机2的实际转速
 * @return float: 返回计算出的基础目标电流（TargetCurrent_Base）
 */
float Get_Base_Current(float Target_Speed, float Current_Speed_M1, float Current_Speed_M2)
{
    // 获取 PID 控制器结构体的指针
    pid_t *PidGoal = &SpeedBase_Controller;
    
    // --- 1. 计算平均实际转速 ---
    // 两个电机的平均转速作为主速度环的反馈
    float Average_Speed = (Current_Speed_M1 + Current_Speed_M2) / 2.0f;

    // --- 2. PID 计算核心 ---
    
    PidGoal->set = Target_Speed;
    PidGoal->err_last = PidGoal->err;
    PidGoal->err = PidGoal->set - Average_Speed; // 计算误差 (目标 - 平均实际)
    
    // P项
    float p_out = PidGoal->p * PidGoal->err;
    
    // I项 (累积)
    PidGoal->i_out += PidGoal->i * PidGoal->err;
    
    // D项 (位置式微分，使用误差变化率)
    float diff = PidGoal->err - PidGoal->err_last;
    float d_out = PidGoal->d * diff;
    
    // --- 3. 积分项限幅（抗饱和） ---
    if(fabs(PidGoal->i_out) > PidGoal->lim_i_out)
    {
        if(PidGoal->i_out < 0)
            PidGoal->i_out = -PidGoal->lim_i_out;
        else
            PidGoal->i_out = PidGoal->lim_i_out;
    }
    
    // --- 4. 计算总输出（即电流基准线） ---
    float total_out = p_out + PidGoal->i_out + d_out;
    
    // --- 5. 总输出限幅（电流限幅） ---
    // 这个限幅是防止 PID 计算出超出电机最大承受能力的电流值
    if(fabs(total_out) > PidGoal->lim_out)
    {
        if(total_out < 0)
            total_out = -PidGoal->lim_out;
        else
            total_out = PidGoal->lim_out;
    }
    
    return total_out; // 返回计算出的基础目标电流
}

// end of file