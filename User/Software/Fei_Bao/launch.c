#include "launch.h"

void Fei_Bao_motor_init(void){

      DJIMotor_init(DJI_M3508,STEN_LEFT);
      DJIMotor_init(DJI_M3508,STEN_RIGHT);
}

void Manual_mode(void){

// 设置储能3508速度
 Shoot_set_sten_velocity(RC_data.rc.ch[1]/6, RC_data.rc.ch[1]/6);

}
