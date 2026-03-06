
#ifndef FEI_BAO_TYPES_H
#define FEI_BAO_TYPES_H

#include "cmsis_os2.h"

/* 飞豹状态机枚举类型 */
typedef enum
{
    FLING_IDLE,
    Reset_Motor, // 归零电机归零工况
    Fir_Ready,   // 第一次待发
    Fir_Fire,    // 第一次开炮

    Fir_Back,  // 第一次回勾
    Sec_Ready, // 第二次待发

    Fir_Down, // 第一次装填下降
    Fir_Push, // 第一次放镖
    Fir_up,   // 第一次装填上升
    Sec_Fire, // 第二次开炮

    Sec_Back,  // 第二次回勾
    Thr_Ready, // 第三次储能

    L_Go,         // X轴坐移
    L_Down,       // 左侧下降
    L_Catch,      // 左侧夹镖
    L_Down_Again, // 第二次下降
    L_Up,         // 左侧上升
    L_Back,       // 左侧归中
    Sec_Down,     // 第二次下降
    Sec_Push,     // 第二次放镖
    Sec_Up,       // 第二次上升
    Thr_Fire,     // 第三次开炮

    Thr_Back,  // 第三次回勾
    Fou_Ready, // 第四次储能

    R_Go,
    R_Down,       // 右侧下降
    R_Catch,      // 右侧夹镖
    R_Down_Again, // 第二次下降
    R_Up,         // 右侧上升
    R_Back,       // 右侧归中
    Thr_Down,     // 第三次下降
    Thr_Push,     // 第三次放镖
    Thr_Up,       // 第三次上升
    Fou_Fire,     // 第四次开炮
    Fou_Back,     // 第四次回勾

    Non_Task // 没有任何任务，调试使用
} FlingState_t;

#endif /* FEI_BAO_TYPES_H */
