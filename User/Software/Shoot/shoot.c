#include "shoot.h"

#define LEFT_ERROR 0;
#define RIGHT_ERROR 0;

float Count_dart=0.0f;          //发射飞镖计数


int push_dart = 1;

//修改位置


float  Energy_storage_angle[2]={2150.0f,-2150.0f};//2250   最近一次2370  2320 2250 2370
float  Energy_storage_mid_angle[2]={1380.0f,-1380.0f};//
float  Push_angle=702.0f;                        //等效推镖行程



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


struct shoot_status shoot;

// 初始化
void Shoot_init()
{
}


// 数据更新
void Shoot_updata()
{

}


// pid计算
void Shoot_pid_cal()
{
}








