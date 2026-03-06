#include "motor_control_task.h"

void motor_control_task(void){
    
    Shoot_updata();
    Shoot_pid_cal();
    DJIMotor_Send_Task();
    LZMotor_Send_Task();
}

void DJIMotor_Send_Task(void){

    DJIMotor_SendCurrent (CAN_20063508_1_4_ID , DJI_CAN_2);
 
}

void LZMotor_Send_Task(void){

    LZMotor_send_command(MID_MOTO);
    LZMotor_send_command(LEFT_MOTO);
    osDelay(2);
    LZMotor_send_command(RIGHT_MOTO);
}
