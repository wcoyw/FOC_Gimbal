#include "as5600.h"
#include "soft_i2c.h"  /* i2cRead */
#include "foc.h"

float as5600_mechanical_angle = 0.0f;
float as5600_electrical_angle = 0.0f;
float as5600_velocity         = 0.0f;
float as5600_accumulated_angle = 0.0f;
float zero_electric_angle     = 0.0f;  /* 对齐后记录的电角度零偏 */

static float    last_angle = 0.0f;

/* 读取12bit原始角度，返回0~4095 */
static uint8_t AS5600_ReadRaw(uint16_t *raw)
{
    uint8_t buf[2];
    if (i2cRead(AS5600_ADDR, AS5600_REG_ANGLE_H, 2, buf) != 0)
        return 1;
    *raw = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return 0;
}

uint8_t AS5600_Init(void)
{
    uint16_t raw;
    if (AS5600_ReadRaw(&raw) != 0)
        return 1;
    last_angle = (float)raw / 4096.0f * 2.0f * PI;
    as5600_accumulated_angle = 0.0f;
    as5600_mechanical_angle = last_angle;
    return 0;
}

/*
 * 获取不带圈数跟踪的机械角度 0~2PI
 * 与官方 GetAngle_NoTrack() 一致
 */
static float AS5600_GetAngleNoTrack(void)
{
    uint16_t raw;
    if (AS5600_ReadRaw(&raw) != 0)
        return as5600_mechanical_angle;  /* 读取失败返回上次值 */
    return (float)raw / 4096.0f * 2.0f * PI;
}

/*
 * 电角度计算 — 与官方 electricAngle() 完全一致:
 *   normalizeAngle(GetAngle_NoTrack() * pp * Dir - zero_electric_Angle)
 */
float AS5600_GetElecAngle(void)
{
    float angle = AS5600_GetAngleNoTrack();
    float elec = angle * (float)MOTOR_POLE_PAIRS * (float)MOTOR_DIR - zero_electric_angle;
    /* normalizeAngle: 归一化到 0~2PI */
    elec = fmodf(elec, 2.0f * PI);
    if (elec < 0.0f) elec += 2.0f * PI;
    return elec;
}

/* 每25次15kHz中断调用一次，实际600Hz，dt=0.001667s */
void AS5600_Update(void)
{
    const float d_time = 0.001667f;
    const float Tf = 0.01f;  /* 低通滤波时间常数，与官方一致 */

    uint16_t raw;
    if (AS5600_ReadRaw(&raw) != 0)
        return;

    /* 计算机械角度 0~2PI */
    float angle = (float)raw / 4096.0f * 2.0f * PI;

    /* 处理过零点跳变（0~2PI边界） */
    float d_angle = angle - last_angle;
    if (d_angle > PI)       d_angle -= 2.0f * PI;
    else if (d_angle < -PI) d_angle += 2.0f * PI;

    /* 低通滤波速度 */
    float raw_velocity = d_angle / d_time;
    as5600_velocity += (raw_velocity - as5600_velocity) * (d_time / (Tf + d_time));

    as5600_mechanical_angle = angle;
    as5600_accumulated_angle -= d_angle;

    /* 电角度 = 与官方electricAngle()一致 */
    as5600_electrical_angle = AS5600_GetElecAngle();

    last_angle = angle;
}
