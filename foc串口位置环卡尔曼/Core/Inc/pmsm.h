#ifndef __PMSM_H
#define __PMSM_H
#include "main.h"
#include <stdint.h>

extern float i_abc[3];
extern float power;
extern uint16_t adc1_value[1];
extern float current_offset[2];

/* 对齐计数器，主循环用来判断对齐阶段是否结束 */
extern volatile uint32_t g_align_count;
extern volatile uint8_t  g_report_ready;
#define ALIGN_TICKS   45000U
#define RELEASE_TICKS 52500U

void motor_Init(void);
void motor_run(void);
void Motor_Enable(void);
void Motor_Disable(void);
#endif /* __PMSM_H */
