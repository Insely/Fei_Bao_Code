#ifndef __AUTO_CONTROL_H__
#define __AUTO_CONTROL_H__

#include "stdint.h"

#pragma pack(1)

typedef struct
{
    struct
    {
        uint8_t sof;
        uint8_t crc8;
    } FrameHeader; // 2
    struct
    {
        float curr_yaw;
        float curr_pitch;
        float curr_omega;
        uint8_t state;// state 0是打车 1是打前哨站 2是打小符 3是打符
        uint8_t autoaim; // autoaim那个0是不用自瞄 1是开自瞄
        uint8_t enemy_color;// 0为蓝色，1为红色
    } To_minipc_data; // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;//2
    uint8_t enter;//1
} STM32_data_t;

typedef struct
{
    struct
    {
        uint8_t sof;
        uint8_t crc8;
    } FrameHeader; // 2
    struct
    {
        float shoot_yaw;
        float shoot_pitch;
        uint8_t fire;      // 发弹信号
        uint8_t target_id; // 目标ID,UI显示用
    } from_minipc_data;    // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;
} MINIPC_data_t;

// 上位机发送的数据结构 (VisionData)
// 必须确保与视觉端的结构体定义完全一致
typedef struct __attribute__((packed)) {
    uint8_t  header;        // 帧头 0xA5
    float    yaw_error;     // 水平偏差
    uint8_t  at_center;     // 是否到达中心
    uint8_t  allow_fire;    // 是否允许发射
} VisionData; 

// 雷达发送的数据结构 (RadarData)
typedef struct __attribute__((packed)) {
    uint8_t  header;        // 帧头 0xBB
    float    dist_value;    // 距离值，单位米
    uint8_t  at_center;     // 是否在中心区域（1是，0否）
    uint8_t  allow_fire;    // 是否允许触发（1允许，0不允许）
} RadarData;

// 奥丁发送的数据结构 (OdinData)
typedef struct __attribute__((packed)) {
    uint8_t  header;        // 帧头 0xCC
    float    distance;      // 距离值
} OdinData;

#pragma pack(4)

void STM32_to_MINIPC(float yaw,float pitch,float omega);
void decodeMINIPCdata(VisionData *target, uint8_t *buff, uint16_t len);
void decodeRADARdata(RadarData *target, uint8_t *buff, uint16_t len);
void decodeODINdata(OdinData *target, uint8_t *buff, uint16_t len);
void Auto_control();
void MINIPC_to_STM32();

extern MINIPC_data_t fromMINIPC;
extern STM32_data_t toMINIPC;

#endif