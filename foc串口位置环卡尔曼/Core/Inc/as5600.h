#ifndef __AS5600_H__
#define __AS5600_H__

#include "main.h"

/* AS5600 I2C地址和寄存器 */
#define AS5600_ADDR       0x36
#define AS5600_REG_ANGLE_H 0x0C  /* 原始角度高8位 */
#define AS5600_REG_ANGLE_L 0x0D  /* 原始角度低4位 */

/* 电机极对数：2804电机通常为7对极，根据实际修改 */
#define MOTOR_POLE_PAIRS  7

/* 电机旋转方向 (与官方Dir变量一致) */
#define MOTOR_DIR         1

extern float as5600_mechanical_angle;  /* 机械角度 0~2PI */
extern float as5600_electrical_angle;  /* 电角度   0~2PI */
extern float as5600_velocity;          /* 角速度   rad/s */
extern float as5600_accumulated_angle; /* 累积角度，连续不归一化，用于位置环 */
extern float zero_electric_angle;      /* 电角度零点偏移(对齐后记录) */

uint8_t AS5600_Init(void);
void    AS5600_Update(void);  /* 在motor_run中调用，更新角度和速度 */
float   AS5600_GetElecAngle(void);  /* 获取当前电角度(含零点校正) */

#endif /* __AS5600_H__ */
