
/*
 * Copyright (c) 2006-2024, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "hipnuc_dec.h"

/* HiPNUC 协议解码驱动文件，请勿修改 */

/* HiPNUC 协议常量 */
#define CHSYNC1                 (0x5A)              /* 帧同步字节1 */
#define CHSYNC2                 (0xA5)              /* 帧同步字节2 */
#define CH_HDR_SIZE             (0x06)              /* 协议帧头长度 */

/* 兼容旧版 HI226/HI229 的数据包ID */
#define HIPNUC_ID_USRID         (0x90)              /* 用户ID */
#define HIPNUC_ID_ACC_RAW       (0xA0)              /* 原始加速度 */
#define HIPNUC_ID_ACC_CAL       (0xA1)              /* 校准加速度 */
#define HIPNUC_ID_GYR_RAW       (0xB0)              /* 原始陀螺仪 */
#define HIPNUC_ID_GYR_CAL       (0xB1)              /* 校准陀螺仪 */
#define HIPNUC_ID_MAG_RAW       (0xC0)              /* 原始磁力计 */
#define HIPNUC_ID_EUL           (0xD0)              /* 欧拉角 */
#define HIPNUC_ID_QUAT          (0xD1)              /* 四元数 */
#define HIPNUC_ID_PRS           (0xF0)              /* 气压 */

/* 新版 HiPNUC 标准数据包ID */
#define HIPNUC_ID_HI91        (0x91)                /* 浮点型IMU数据包 */
#define HIPNUC_ID_HI92        (0x92)                /* 整型IMU数据包 */
#define HIPNUC_ID_HI81        (0x81)                /* INS组合导航数据包 */

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif


static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len);

/* 通用类型转换宏和函数 */
#define I2(p) (*((int16_t *)(p)))       /* 从缓冲区读取有符号16位整数 */

/* 从缓冲区读取无符号16位整数 */
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

/* 从缓冲区读取32位浮点数 */
static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

/* 解析一帧数据的有效载荷，将解析结果填入对应的数据结构 */
static int parse_data(hipnuc_raw_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];  /* 指向帧头之后的有效载荷起始位置 */
    
    /* 清除上一帧的数据标记 */
    raw->hi91.tag = 0;
    raw->hi81.tag = 0;
    raw->hi92.tag = 0;

    /* 逐个解析有效载荷中的子数据包 */
    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_ID_USRID:       /* 用户ID，跳过 */
            ofs += 2;
            break;
        case HIPNUC_ID_ACC_RAW:     /* 旧版加速度数据（原始/校准），单位转换为g */
        case HIPNUC_ID_ACC_CAL:
             raw->hi91.tag = HIPNUC_ID_HI91;
             raw->hi91.acc[0] = (float)I2(p + ofs + 1) / 1000;
             raw->hi91.acc[1] = (float)I2(p + ofs + 3) / 1000;
             raw->hi91.acc[2] = (float)I2(p + ofs + 5) / 1000;
            ofs += 7;
            break;
        case HIPNUC_ID_GYR_RAW:     /* 旧版陀螺仪数据（原始/校准），单位转换为°/s */
        case HIPNUC_ID_GYR_CAL:
            raw->hi91.tag = HIPNUC_ID_HI91;
            raw->hi91.gyr[0] = (float)I2(p + ofs + 1) / 10;
            raw->hi91.gyr[1] = (float)I2(p + ofs + 3) / 10;
            raw->hi91.gyr[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_MAG_RAW:     /* 旧版磁力计数据 */
            raw->hi91.tag = HIPNUC_ID_HI91;
            raw->hi91.mag[0] = (float)I2(p + ofs + 1) / 10;
            raw->hi91.mag[1] = (float)I2(p + ofs + 3) / 10;
            raw->hi91.mag[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_EUL:         /* 旧版欧拉角数据：俯仰/横滚/航向 */
            raw->hi91.tag = HIPNUC_ID_HI91;
            raw->hi91.pitch = (float)I2(p + ofs + 1) / 100;
            raw->hi91.roll = (float)I2(p + ofs + 3) / 100;
            raw->hi91.yaw = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_QUAT:        /* 旧版四元数数据 (w,x,y,z) */
            raw->hi91.tag = HIPNUC_ID_HI91;
            raw->hi91.quat[0] = R4(p + ofs + 1);
            raw->hi91.quat[1] = R4(p + ofs + 5);
            raw->hi91.quat[2] = R4(p + ofs + 9);
            raw->hi91.quat[3] = R4(p + ofs + 13);
            ofs += 17;
            break;
        case HIPNUC_ID_PRS:         /* 旧版气压数据 */
            raw->hi91.tag = HIPNUC_ID_HI91;
            raw->hi91.air_pressure = R4(p + ofs + 1);
            ofs += 5;
            break;
        case HIPNUC_ID_HI91:        /* 新版0x91浮点型IMU数据包，整体拷贝 */
            memcpy(&raw->hi91, p + ofs, sizeof(hi91_t));
            ofs += sizeof(hi91_t);
            break;
        case HIPNUC_ID_HI81:        /* 新版0x81 INS组合导航数据包，整体拷贝 */
            memcpy(&raw->hi81, p + ofs, sizeof(hi81_t));
            ofs += sizeof(hi81_t);
            break;
        case HIPNUC_ID_HI92:        /* 新版0x92整型IMU数据包，整体拷贝 */
            memcpy(&raw->hi92, p + ofs, sizeof(hi92_t));
            ofs += sizeof(hi92_t);
            break;
        default:                    /* 未知数据包ID，跳过1字节 */
            ofs++;
            break;
        }
    }
    return 1;
}

/* 解码一帧完整的 HiPNUC 数据：先校验CRC，再解析有效载荷 */
static int decode_hipnuc(hipnuc_raw_t *raw)
{
    uint16_t crc = 0;

    /* CRC16 校验：对帧头(不含CRC字段)和有效载荷分别计算 */
    hipnuc_crc16(&crc, raw->buf, (CH_HDR_SIZE-2));
    hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
    if (crc != U2(raw->buf + (CH_HDR_SIZE-2)))
    {
        /* CRC校验失败，丢弃该帧 */
        return -1;
    }

    return parse_data(raw);
}

/* 帧同步：检测连续的两个同步字节 0x5A 0xA5 */
static int sync_hipnuc(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

/**
 * @brief  HiPNUC 解码器输入函数，每次输入一个字节
 *
 * @param  raw  解码器结构体指针
 * @param  data 从串口读取的一个字节
 * @return >0: 成功接收并解码一帧数据; 0: 需要更多数据; -1: 错误
 */
int hipnuc_input(hipnuc_raw_t *raw, uint8_t data)
{
    /* 第一步：寻找帧同步字节 0x5A 0xA5 */
    if (raw->nbyte == 0)
    {
        if (!sync_hipnuc(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    /* 第二步：读取帧头中的有效载荷长度字段 */
    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE))
        {
            /* 长度超限，重置状态 */
            raw->nbyte = 0;
            return -1;
        }
    }

    /* 第三步：等待接收完整帧数据 */
    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        return 0;
    }

    /* 第四步：接收完毕，进行CRC校验和解码 */
    raw->nbyte = 0;

    return decode_hipnuc(raw);
}


/**
 * @brief  将解码后的数据包格式化为JSON字符串，用于调试输出
 *
 * @param  raw       解码器结构体指针，包含已解码的数据
 * @param  buf       输出字符串缓冲区，确保大小不小于256字节
 * @param  buf_size  输出缓冲区大小
 * @return 写入缓冲区的字符数
 */
int hipnuc_dump_packet(hipnuc_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret;

    /* 输出 0x91 浮点型IMU数据包 */
    if(raw->hi91.tag == HIPNUC_ID_HI91)
    {
        /* 数据格式说明:
         * system_time: 系统时间戳 (ms)
         * acc: 加速度 (m/s?)
         * gyr: 角速度 (°/s)
         * mag: 磁场强度 (uT)
         * pitch/roll/yaw: 俯仰角/横滚角/航向角 (°)
         * quat: 四元数 (w,x,y,z)
         * air_pressure: 气压 (Pa)
         */
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI91\",\n"
            "  \"system_time\": %d,\n"
            "  \"acc\": [%.3f, %.3f, %.3f],\n"
            "  \"gyr\": [%.3f, %.3f, %.3f],\n"
            "  \"mag\": [%.3f, %.3f, %.3f],\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"yaw\": %.2f,\n"
            "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
            "  \"air_pressure\": %.1f\n"
            "}\n",
            raw->hi91.system_time,
            raw->hi91.acc[0]*GRAVITY, raw->hi91.acc[1]*GRAVITY, raw->hi91.acc[2]*GRAVITY,
            raw->hi91.gyr[0], raw->hi91.gyr[1], raw->hi91.gyr[2],
            raw->hi91.mag[0], raw->hi91.mag[1], raw->hi91.mag[2],
            raw->hi91.pitch, raw->hi91.roll, raw->hi91.yaw,
            raw->hi91.quat[0], raw->hi91.quat[1], raw->hi91.quat[2], raw->hi91.quat[3],
            raw->hi91.air_pressure);
    }
    
    /* 输出 0x92 整型IMU数据包 */
    else if(raw->hi92.tag == HIPNUC_ID_HI92)
    {
        /* 数据格式说明:
         * temperature: 温度 (°C)
         * acc: 加速度 (m/s?)
         * gyr: 角速度 (°/s)
         * mag: 磁场强度 (uT)
         * pitch/roll/yaw: 俯仰角/横滚角/航向角 (°)
         */
        ret = snprintf(buf + written, buf_size - written,
            "{\n"
            "  \"type\": \"HI92\",\n"
            "  \"status\": %d,\n"
            "  \"temperature\": %d,\n"
            "  \"acc\": [%.3f, %.3f, %.3f],\n"
            "  \"gyr\": [%.3f, %.3f, %.3f],\n"
            "  \"mag\": [%.3f, %.3f, %.3f],\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"yaw\": %.2f\n"
            "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
            "}\n",
            raw->hi92.status,
            raw->hi92.temperature,
            raw->hi92.acc_b[0]*0.0048828, raw->hi92.acc_b[1]*0.0048828, raw->hi92.acc_b[2]*0.0048828,
            raw->hi92.gyr_b[0]*(0.001*R2D), raw->hi92.gyr_b[1]*(0.001*R2D), raw->hi92.gyr_b[2]*(0.001*R2D),
            raw->hi92.mag_b[0]*0.030517, raw->hi92.mag_b[1]*0.030517, raw->hi92.mag_b[2]*0.030517,
            raw->hi92.pitch*0.001, raw->hi92.roll*0.001, raw->hi92.yaw*0.001,
            raw->hi91.quat[0], raw->hi91.quat[1], raw->hi91.quat[2], raw->hi91.quat[3]);
    }

    /* 输出 0x81 INS组合导航数据包 */
else if(raw->hi81.tag == HIPNUC_ID_HI81)
{
    /* 数据格式说明:
     * status: 设备状态
     * ins_status: INS算法状态
     * gpst_wn/tow: GPS周数/周内秒
     * gyr: 角速度 (°/s)
     * acc: 加速度 (m/s?)
     * mag: 磁场强度 (uT)
     * air_pressure: 气压 (Pa)
     * temperature: 温度 (°C)
     * utc: UTC时间 YYYY-MM-DD HH:mm:ss.SSS
     * pitch/roll/yaw: 俯仰角/横滚角/航向角 (°)
     * quat: 四元数 (w,x,y,z)
     * ins_lat/lon: INS纬度/经度 (°)
     * ins_msl: INS海拔高度 (m)
     * pdop/hdop: 位置/水平精度因子
     * solq_pos: 定位质量 0:无效 1:单点 2:差分 4:RTK浮点 5:RTK固定
     * nv_pos: 定位使用的卫星数
     * solq_heading: 航向质量 0:无效 4:有效
     * nv_heading: 航向使用的卫星数
     * diff_age: 差分龄期 (s)
     * undulation: 大地水准面差距 (m)
     * vel_enu: 东/北/天速度 (m/s)
     * acc_enu: 东/北/天加速度 (m/s?)
     */
    ret = snprintf(buf + written, buf_size - written,
        "{\n"
        "  \"type\": \"HI81\",\n"
        "  \"status\": %d,\n"
        "  \"ins_status\": %d,\n"
        "  \"gpst_wn\": %d,\n"
        "  \"gpst_tow\": %d,\n"
        "  \"gyr\": [%.3f, %.3f, %.3f],\n"
        "  \"acc\": [%.3f, %.3f, %.3f],\n"
        "  \"mag\": [%.3f, %.3f, %.3f],\n"
        "  \"air_pressure\": %.1f,\n"
        "  \"temperature\": %d,\n"
        "  \"utc\": \"20%02d-%02d-%02d %02d:%02d:%02d.%03d\",\n"
        "  \"pitch\": %.2f,\n"
        "  \"roll\": %.2f,\n"
        "  \"yaw\": %.2f,\n"
        "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
        "  \"ins_lat\": %.7f,\n"
        "  \"ins_lon\": %.7f,\n"
        "  \"ins_msl\": %.2f,\n"
        "  \"pdop\": %.1f,\n"
        "  \"hdop\": %.1f,\n"
        "  \"solq_pos\": %d,\n"
        "  \"nv_pos\": %d,\n"
        "  \"solq_heading\": %d,\n"
        "  \"nv_heading\": %d,\n"
        "  \"diff_age\": %d,\n"
        "  \"undulation\": %.2f,\n"
        "  \"vel_enu\": [%.2f, %.2f, %.2f],\n"
        "  \"acc_enu\": [%.2f, %.2f, %.2f],\n"
        "}\n",
        raw->hi81.status,
        raw->hi81.ins_status,
        raw->hi81.gpst_wn,
        raw->hi81.gpst_tow,
        raw->hi81.gyr_b[0]*(0.001*R2D), raw->hi81.gyr_b[1]*(0.001*R2D), raw->hi81.gyr_b[2]*(0.001*R2D),
        raw->hi81.acc_b[0]*0.0048828, raw->hi81.acc_b[1]*0.0048828, raw->hi81.acc_b[2]*0.0048828,
        raw->hi81.mag_b[0]*0.030517, raw->hi81.mag_b[1]*0.030517, raw->hi81.mag_b[2]*0.030517,
        (float)raw->hi81.air_pressure,
        raw->hi81.temperature,
        raw->hi81.utc_year,
        raw->hi81.utc_month,
        raw->hi81.utc_day,
        raw->hi81.utc_hour,
        raw->hi81.utc_min,
        raw->hi81.utc_msec/1000,
        raw->hi81.utc_msec%1000,
        raw->hi81.pitch*0.01,
        raw->hi81.roll*0.01,
        raw->hi81.yaw*0.01,
        raw->hi81.quat[0]*0.0001, raw->hi81.quat[1]*0.0001, raw->hi81.quat[2]*0.0001, raw->hi81.quat[3]*0.0001,
        raw->hi81.ins_lat*1e-7,
        raw->hi81.ins_lon*1e-7,
        raw->hi81.ins_msl*1e-3,
        raw->hi81.pdop*0.1,
        raw->hi81.hdop*0.1,
        raw->hi81.solq_pos,
        raw->hi81.nv_pos,
        raw->hi81.solq_heading,
        raw->hi81.nv_heading,
        raw->hi81.diff_age,
        raw->hi81.undulation*0.01,
        raw->hi81.vel_enu[0]*0.01, raw->hi81.vel_enu[1]*0.01, raw->hi81.vel_enu[2]*0.01,
        raw->hi81.acc_enu[0]*0.0048828, raw->hi81.acc_enu[1]*0.0048828, raw->hi81.acc_enu[2]*0.0048828);
    }

    if (ret > 0) written += ret;
    return written;
}

/**
 * @brief  计算 HiPNUC CRC16 校验值
 *
 * @param  inital CRC初始值指针（累加计算时传入上次结果）
 * @param  buf    输入数据缓冲区指针
 * @param  len    缓冲区数据长度
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *inital = crc;
}
