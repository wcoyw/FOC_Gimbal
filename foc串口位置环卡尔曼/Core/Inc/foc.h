#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <math.h>

#define PI 3.14159265358979f

typedef struct
{
    float theta;        /* 转子角度 */
    float Udc;          /* 电机供电电压 */
    float Tpwm;         /* 电机PWM周期 */
} ExtX;

typedef struct
{
    float Tcmp1;        /* SVPWM输出U项电流 */
    float Tcmp2;        /* SVPWM输出V项电流 */
    float Tcmp3;        /* SVPWM输出W项电流 */
} ExtY;

typedef struct
{
    float Vd;           /* PI控制器输出d轴电压 */
    float Vq;           /* PI控制器输出q轴电压 */
} Voltage_DQ_DEF;

typedef struct
{
    float Cos;          /* 转子角度计算Cos值 */
    float Sin;          /* 转子角度计算Sin值 */
} Transf_Cos_Sin_DEF;

typedef struct
{
    float Valpha;       /* RevPark变换输出Alpha轴电压 */
    float Vbeta;        /* RevPark变换输出Beta轴电压 */
} Voltage_Alpha_Beta_DEF;

typedef struct
{
    float Ia;           /* 电机U项电流 */
    float Ib;           /* 电机V项电流 */
    float Ic;           /* 电机W项电流 */
} Current_ABC_DEF;

typedef struct
{
    float Ialpha;
    float Ibeta;
} Current_Alpha_Beta_DEF;

typedef struct
{
    float Id;
    float Iq;
} Current_DQ_DEF;

typedef struct
{
    float kp;
    float ki;
    float integral;
    float out_max;
    float out_min;
} PID_DEF;

/*
 * 一维标量卡尔曼滤波器
 *   Q: 过程噪声协方差（越大，越信任测量值，响应越快）
 *   R: 测量噪声协方差（越大，滤波越强，响应越慢）
 * 典型电流噪声场景：Q=0.01, R=1.0
 */
typedef struct
{
    float Q;    /* 过程噪声协方差 */
    float R;    /* 测量噪声协方差 */
    float P;    /* 误差协方差估计 */
    float x;    /* 状态估计值 */
} Kalman_DEF;

/*
 * 一阶IIR低通滤波器
 *   alpha: 平滑系数 (0,1)，越小滤波越强、响应越慢
 *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *   15kHz采样下截止频率 fc ≈ alpha * fs / (2*PI) ≈ alpha * 2387 Hz
 *   默认 alpha=0.15 → fc ≈ 358 Hz，对禁止不动噪声抑制明显
 */
typedef struct
{
    float alpha;  /* 平滑系数 */
    float y;      /* 上一拍输出 */
} IIR_DEF;

/* 外部调用变量 — volatile防止ARMCC高优化级别下被优化掉 */
extern volatile ExtX rtX;
extern volatile ExtY rtY;
extern Transf_Cos_Sin_DEF Transf_Cos_Sin;
extern Voltage_DQ_DEF Voltage_DQ;
extern Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta;
extern PID_DEF speed_pid;
extern PID_DEF id_pid;
extern PID_DEF iq_pid;
extern PID_DEF pos_pid;
extern float target_velocity;   /* rad/s，目标速度 */
extern float target_position;   /* rad，目标累积角度 */
extern float iq_ref;            /* 位置/速度环输出的Iq目标 A */
extern Current_ABC_DEF current_abc;
extern Current_Alpha_Beta_DEF current_alpha_beta;
extern Current_DQ_DEF current_dq;

/* 外部调用函数 */
extern void FOC_Init(void);
extern void FOC_Run(void);
extern void Position_PID_Calc(float target, float measured, float dt);
extern void Speed_PID_Calc(float target, float measured, float dt);
extern void Clarke_Transform(Current_ABC_DEF i_abc, Current_Alpha_Beta_DEF* i_ab);
extern void Park_Transform(Current_Alpha_Beta_DEF i_ab, Transf_Cos_Sin_DEF cos_sin, Current_DQ_DEF* i_dq);
extern void Current_PID_Calc(float id_target, float iq_target, Current_DQ_DEF i_dq, float dt);
extern void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin);
extern void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta);
extern void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm);

/* 卡尔曼滤波器（用于Ia/Ib） */
extern void  Kalman_Init(Kalman_DEF* kf, float Q, float R, float init_val);
extern float Kalman_Update(Kalman_DEF* kf, float measurement);
extern Kalman_DEF kalman_ia;
extern Kalman_DEF kalman_ib;

/* 一阶IIR低通滤波（用于Id/Iq） */
extern void  IIR_Init(IIR_DEF* f, float alpha, float init_val);
extern float IIR_Update(IIR_DEF* f, float x);
extern IIR_DEF iir_id;
extern IIR_DEF iir_iq;

#endif /* __FOC_H__ */
