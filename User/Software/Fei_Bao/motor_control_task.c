#include "motor_control_task.h"

void motor_control_task(void){

    Shoot_pid_cal();
    Shoot_updata();
    DJIMotor_Send_Task();
}

void DJIMotor_Send_Task(void){

       DJIMotor_SendCurrent (CAN_20063508_1_4_ID , DJI_CAN_1);
 
}

