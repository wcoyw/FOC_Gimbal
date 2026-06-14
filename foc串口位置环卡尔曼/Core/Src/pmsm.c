#include "pmsm.h"
#include "foc.h"
#include "main.h"
#include "adc.h"
#include "soft_i2c.h"
#include "as5600.h"
#include <stdio.h>

float i_abc[3];
float power;

/*
 * V3P硬件参数（来自手册和官方例程）:
 *   采样电阻: 0.01 ohm
 *   INA240A2运放增益: 50x
 *   Vref = 3.3V, 12bit ADC
 *   电流 = (ADC - offset) * 3.3 / 4096 / (0.01 * 50)
 *   极性: 官方gain_a = gain_b = -1 (取反)
 */
#define SHUNT_R       0.01f
#define AMP_GAIN      50.0f
#define ADC_CONV      (3.3f / 4096.0f / (SHUNT_R * AMP_GAIN))

/*
 * PWM参数: TIM1向上计数, ARR=11200, 168MHz/11200 = 15kHz
 * 与官方TIM2(ARR=4800, 72MHz/4800=15kHz)频率一致
 */
#define PWM_ARR       11200.0f

/* 供电电压，与官方一致 */
#define VOLTAGE_SUPPLY  12.6f
#define VOLTAGE_LIMIT   12.6f

float current_offset[2] = {2048.0f, 2048.0f};

/* 对齐阶段用：主循环判断是否还在对齐中 */
volatile uint32_t g_align_count = 0;
volatile uint8_t  g_report_ready = 0;
#define ALIGN_TICKS   45000U   /* 3s  @15kHz */
#define RELEASE_TICKS 52500U   /* 3.5s @15kHz */

/* 使能/禁用电机 (PB12 -> V3P驱动板EN引脚) */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

/*
 * SetPwm: 与官方DFOC.c中的SetPwm完全一致
 *   输入: 三相电压 Ua/Ub/Uc (0 ~ voltage_supply)
 *   输出: 归一化占空比 × ARR -> 写入TIM1 CCR
 */
static void SetPwm(float Ua, float Ub, float Uc)
{
    /* 限幅 0 ~ voltage_limit */
    if (Ua < 0.0f) Ua = 0.0f; else if (Ua > VOLTAGE_LIMIT) Ua = VOLTAGE_LIMIT;
    if (Ub < 0.0f) Ub = 0.0f; else if (Ub > VOLTAGE_LIMIT) Ub = VOLTAGE_LIMIT;
    if (Uc < 0.0f) Uc = 0.0f; else if (Uc > VOLTAGE_LIMIT) Uc = VOLTAGE_LIMIT;

    /* 归一化到0~1，再乘ARR写入比较寄存器 */
    float dc_a = Ua / VOLTAGE_SUPPLY;
    float dc_b = Ub / VOLTAGE_SUPPLY;
    float dc_c = Uc / VOLTAGE_SUPPLY;

    TIM1->CCR1 = (uint32_t)(dc_a * PWM_ARR);
    TIM1->CCR2 = (uint32_t)(dc_b * PWM_ARR);
    TIM1->CCR3 = (uint32_t)(dc_c * PWM_ARR);
}

/*
 * SetPhaseVoltage: 与官方DFOC.c中的SetPhaseVoltage一致
 *   反Park变换 + 中点注入SPWM
 *   输入: Uq/Ud (电压V), angle_el (电角度rad)
 */
static void SetPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    float ua = Ualpha + VOLTAGE_SUPPLY / 2.0f;
    float ub = (1.7320508f * Ubeta - Ualpha) / 2.0f + VOLTAGE_SUPPLY / 2.0f;
    float uc = -(Ualpha + 1.7320508f * Ubeta) / 2.0f + VOLTAGE_SUPPLY / 2.0f;

    SetPwm(ua, ub, uc);
}

void motor_Init(void)
{
    /* ---- 步骤1：使能驱动板，启动PWM ---- */
    Motor_Enable();

    /* 只启动3路普通PWM + CH4(ADC触发)，不启动互补通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* TIM1高级定时器需要使能MOE (主输出使能) */
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 设置双重同步注入模式 */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_MULTI) | ADC_DUALMODE_INJECSIMULT;

    /* 确保ADC1/ADC2上电 */
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC2->CR2 |= ADC_CR2_ADON;

    /* CCR1/2/3 = 0：初始占空比为0 */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    HAL_Delay(10);  /* 等待PWM稳定、运放建立 */

    /* ---- 步骤2：校准电流零偏（加超时防止ADC未触发时死等）---- */
    /*
     * 关键修正：注入模式的数据在 JDR1 寄存器，不在 CDR！
     * CDR 仅用于 regular（规则）双重模式。
     * ADC1->JDR1 = Ia通道(CH0/PA0), ADC2->JDR1 = Ib通道(CH8/PB0)
     */
    float off0 = 0.0f, off1 = 0.0f;
    for (int i = 0; i < 128; i++)
    {
        uint32_t timeout = 200000;
        while (!(ADC1->SR & ADC_SR_JEOC))
        {
            if (--timeout == 0) goto calibration_done;
        }
        off0 += (float)(ADC1->JDR1 & 0xFFFU);
        off1 += (float)(ADC2->JDR1 & 0xFFFU);
        ADC1->SR &= ~ADC_SR_JEOC;
        ADC2->SR &= ~ADC_SR_JEOC;
    }
    current_offset[0] = off0 / 128.0f;
    current_offset[1] = off1 / 128.0f;
calibration_done:

    /* ---- 步骤3：初始化外设 ---- */
    ADC1->SR &= ~ADC_SR_JEOC;
    ADC2->SR &= ~ADC_SR_JEOC;

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    IIC_Init();
    AS5600_Init();
    FOC_Init();

    /* 最后使能ADC1注入中断，开始正常控制 */
    ADC1->SR  &= ~ADC_SR_JEOC;
    ADC2->SR  &= ~ADC_SR_JEOC;
    ADC1->CR1 |= ADC_CR1_JEOCIE;
}

void motor_run(void)
{
    static uint32_t count = 0;

    /* 对齐阶段：锁定转子，与参考项目一致用局部count */
    if (count < 10000)
    {
        count++;
        SetPhaseVoltage(3.0f, 0.0f, 4.71238898f);
        if (count == 10000)
        {
            AS5600_Init();
            AS5600_Update();
            /* 对齐完成后清零累计角度，作为位置环零点 */
            as5600_accumulated_angle = 0.0f;
        }
        return;
    }

    /* 编码器200Hz更新 + 串口上报标志 */
    static uint8_t enc_div = 0;
    if (++enc_div >= 25)
    {
        enc_div = 0;
        AS5600_Update();
        g_report_ready = 1;
        /* 位置环：目标0弧度，反馈as5600_accumulated_angle，dt=1/200Hz */
        Position_PID_Calc(0.0f, as5600_accumulated_angle, 0.005f);
    }

    /* 读取电流 (注入结果在 JDR1) */
    float raw_ia = (float)(ADC1->JDR1 & 0xFFFU);
    float raw_ib = (float)(ADC2->JDR1 & 0xFFFU);
    float ia_raw = -(raw_ia - current_offset[0]) * ADC_CONV;
    float ib_raw = -(raw_ib - current_offset[1]) * ADC_CONV;
    /* 卡尔曼滤波抑制ADC+运放噪声 */
    current_abc.Ia = Kalman_Update(&kalman_ia, ia_raw);
    current_abc.Ib = Kalman_Update(&kalman_ib, ib_raw);
    current_abc.Ic = -current_abc.Ia - current_abc.Ib;

    /* Clarke + Park 变换（直接用AS5600读到的电角度，不做外推） */
    Angle_To_Cos_Sin(as5600_electrical_angle, &Transf_Cos_Sin);
    Clarke_Transform(current_abc, &current_alpha_beta);
    Park_Transform(current_alpha_beta, Transf_Cos_Sin, &current_dq);

    /* IIR低通滤波：抑制Park变换后Id/Iq残余噪声，再送入电流环PID */
    current_dq.Id = IIR_Update(&iir_id, current_dq.Id);
    current_dq.Iq = IIR_Update(&iir_iq, current_dq.Iq);

    /* 电流环PID */
    Current_PID_Calc(0.0f, iq_ref, current_dq, 0.0000667f);

    /* 输出PWM */
    SetPhaseVoltage(Voltage_DQ.Vq, Voltage_DQ.Vd, as5600_electrical_angle);
}
