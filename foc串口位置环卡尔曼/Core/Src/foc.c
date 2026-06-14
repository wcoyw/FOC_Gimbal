#include "foc.h"
#include "as5600.h"

/*
 *****************************************************************************
 *                              定义变量
 *****************************************************************************
 */
volatile ExtX rtX;               /* 输入结构体变量 */
volatile ExtY rtY;               /* 输出结构体变量 */
Transf_Cos_Sin_DEF Transf_Cos_Sin;         /* 当前角度计算三角函数 */
Voltage_DQ_DEF Voltage_DQ;                 /* DQ电压输出 */
Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta; /* RevPark变换输出 */
Current_ABC_DEF current_abc;
Current_Alpha_Beta_DEF current_alpha_beta;
Current_DQ_DEF current_dq;

/* 电流卡尔曼滤波器实例（Ia/Ib） */
Kalman_DEF kalman_ia;
Kalman_DEF kalman_ib;

/* Id/Iq 一阶IIR低通滤波器实例 */
IIR_DEF iir_id;
IIR_DEF iir_iq;


/*
 *****************************************************************************
 *                              函数声明
 *****************************************************************************
 */
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin);
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta);
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm);


/*
 *****************************************************************************
 * 功  能：FOC算法初始化
 * 形  参：无
 * 说  明：初始化PI控制器参数
 *****************************************************************************
 */
PID_DEF speed_pid;
PID_DEF id_pid;
PID_DEF iq_pid;
PID_DEF pos_pid;
float target_velocity = 0.0f;
float target_position = 0.0f;
float iq_ref          = 0.0f;

/* 位置PID内部状态 */
static float pos_last_error  = 0.0f;
static float pos_integral     = 0.0f;

void FOC_Init(void)
{
    Voltage_DQ.Vd = 0.0f;
    Voltage_DQ.Vq = 0.0f;

    /*
     * 位置环参数（参考例程 Set_Angle: Kp=0.133, Ki=0.01, Kd=0，误差单位度）
     * 本工程误差单位rad，等效: Kp_rad = Kp_deg * (180/PI) ≈ 0.133*57.3 ≈ 7.6
     * 从保守值开始调：Kp=3, Ki=0.1, Kd=0.05，输出限幅±0.8A
     */
    pos_pid.kp       = 3.0f;
    pos_pid.ki       = 0.10f;
    pos_pid.integral = 0.0f;
    pos_pid.out_max  = 0.8f;
    pos_pid.out_min  = -0.8f;
    pos_last_error   = 0.0f;
    pos_integral     = 0.0f;

    /* 速度环：输出Iq目标（A） */
    speed_pid.kp       = 2.0f;
    speed_pid.ki       = 0.5f;
    speed_pid.integral = 0.0f;
    speed_pid.out_max  = 0.5f;
    speed_pid.out_min  = -0.5f;

    /* 电流环：输入误差A，输出电压V，限幅=Vsupply/2=6.3V（与官方一致） */
    id_pid.kp       = 1.0f;
    id_pid.ki       = 0.5f;
    id_pid.integral = 0.0f;
    id_pid.out_max  = 6.3f;
    id_pid.out_min  = -6.3f;

    iq_pid.kp       = 2.0f;
    iq_pid.ki       = 0.5f;
    iq_pid.integral = 0.0f;
    iq_pid.out_max  = 6.3f;
    iq_pid.out_min  = -6.3f;

    /*
     * 电流卡尔曼滤波器初始化（Ia/Ib）
     *   Q=0.01: 过程噪声小（电流变化平滑）
     *   R=1.0 : 测量噪声大（ADC+运放噪声强）
     */
    Kalman_Init(&kalman_ia, 0.01f, 1.0f, 0.0f);
    Kalman_Init(&kalman_ib, 0.01f, 1.0f, 0.0f);

    /*
     * Id/Iq IIR低通滤波器初始化
     *   alpha=0.15: 15kHz下截止频率约358Hz
     *   噪声强时减小alpha（如0.05~0.10），动态要求高时增大（如0.3）
     */
    IIR_Init(&iir_id, 0.15f, 0.0f);
    IIR_Init(&iir_iq, 0.15f, 0.0f);
}

/*
 * 位置PID — 完整PID含D项，与参考例程 PID_Controller() 结构一致
 * dt: 调用周期，单位s（pmsm.c中以200Hz调用，dt=0.005f）
 */
void Position_PID_Calc(float target, float measured, float dt)
{
    float error = target - measured;

    /* I项：梯形积分（与参考例程一致：Ki*0.5*Ts*Error的累加） */
    pos_integral += pos_pid.ki * 0.5f * dt * error;
    /* 积分限幅（直接限 ±out_max） */
    if (pos_integral >  pos_pid.out_max) pos_integral =  pos_pid.out_max;
    if (pos_integral < -pos_pid.out_max) pos_integral = -pos_pid.out_max;

    /* D项：基于相邻误差差分，比速度反馈噪声更低 */
    float kd   = 0.05f;
    float deriv = kd * (error - pos_last_error) / dt;

    float out = pos_pid.kp * error + pos_integral + deriv;

    if (out >  pos_pid.out_max) out =  pos_pid.out_max;
    if (out < -pos_pid.out_max) out = -pos_pid.out_max;

    pos_last_error = error;
    iq_ref = out;  /* 直接输出iq_ref，跳过速度环 */
}

void Speed_PID_Calc(float target, float measured, float dt)
{
    float error = target - measured;
    speed_pid.integral += error * dt;
    if (speed_pid.integral > speed_pid.out_max / (speed_pid.ki + 0.001f))
        speed_pid.integral = speed_pid.out_max / (speed_pid.ki + 0.001f);
    if (speed_pid.integral < speed_pid.out_min / (speed_pid.ki + 0.001f))
        speed_pid.integral = speed_pid.out_min / (speed_pid.ki + 0.001f);
    float out = speed_pid.kp * error + speed_pid.ki * speed_pid.integral;
    if (out > speed_pid.out_max) out = speed_pid.out_max;
    if (out < speed_pid.out_min) out = speed_pid.out_min;
    iq_ref = out;  /* 速度环输出Iq目标（A） */
}

void Clarke_Transform(Current_ABC_DEF i_abc, Current_Alpha_Beta_DEF* i_ab)
{
    i_ab->Ialpha = i_abc.Ia;
    i_ab->Ibeta  = (i_abc.Ia + 2.0f * i_abc.Ib) / 1.7320508f;
}

void Park_Transform(Current_Alpha_Beta_DEF i_ab, Transf_Cos_Sin_DEF cos_sin, Current_DQ_DEF* i_dq)
{
    i_dq->Id =  i_ab.Ialpha * cos_sin.Cos + i_ab.Ibeta * cos_sin.Sin;
    i_dq->Iq = -i_ab.Ialpha * cos_sin.Sin + i_ab.Ibeta * cos_sin.Cos;
}

void Current_PID_Calc(float id_target, float iq_target, Current_DQ_DEF i_dq, float dt)
{
    float error_d = id_target - i_dq.Id;
    id_pid.integral += error_d * dt;
    if (id_pid.integral > id_pid.out_max / (id_pid.ki + 0.001f)) id_pid.integral = id_pid.out_max / (id_pid.ki + 0.001f);
    if (id_pid.integral < id_pid.out_min / (id_pid.ki + 0.001f)) id_pid.integral = id_pid.out_min / (id_pid.ki + 0.001f);
    Voltage_DQ.Vd = id_pid.kp * error_d + id_pid.ki * id_pid.integral;
    if (Voltage_DQ.Vd > id_pid.out_max) Voltage_DQ.Vd = id_pid.out_max;
    if (Voltage_DQ.Vd < id_pid.out_min) Voltage_DQ.Vd = id_pid.out_min;

    float error_q = iq_target - i_dq.Iq;
    iq_pid.integral += error_q * dt;
    if (iq_pid.integral > iq_pid.out_max / (iq_pid.ki + 0.001f)) iq_pid.integral = iq_pid.out_max / (iq_pid.ki + 0.001f);
    if (iq_pid.integral < iq_pid.out_min / (iq_pid.ki + 0.001f)) iq_pid.integral = iq_pid.out_min / (iq_pid.ki + 0.001f);
    Voltage_DQ.Vq = iq_pid.kp * error_q + iq_pid.ki * iq_pid.integral;
    if (Voltage_DQ.Vq > iq_pid.out_max) Voltage_DQ.Vq = iq_pid.out_max;
    if (Voltage_DQ.Vq < iq_pid.out_min) Voltage_DQ.Vq = iq_pid.out_min;
}


/*
 *****************************************************************************
 * 功  能：FOC算法
 * 形  参：无
 * 说  明：执行FOC控制流程，用PI控制器控制电流环
 *****************************************************************************
 */
void FOC_Run(void)
{
    /* 从volatile结构体拷贝到局部变量，避免ARMCC高优化下的volatile传参警告 */
    float theta = rtX.theta;
    float udc   = rtX.Udc;
    float tpwm  = rtX.Tpwm;

    Angle_To_Cos_Sin(theta, &Transf_Cos_Sin);
    Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta);
    SVPWM_Calc(Voltage_Alpha_Beta, udc, tpwm);
}


/*
 *****************************************************************************
 * 功  能：COS_SIN值计算
 * 形  参：角度以及COS_SIN结构体
 * 说  明：COS_SIN值计算
 *****************************************************************************
 */
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin)
{
    cos_sin->Cos = cosf(angle);
    cos_sin->Sin = sinf(angle);
}


/*
 *****************************************************************************
 * 功  能：反PARK变换
 * 形  参：DQ轴电压、COS_SIN值、alpha_beta电压
 *          Valpha = Vd * cos(theta) - Vq * sin(theta);
 *          Vbeta  = Vd * sin(theta) + Vq * cos(theta);
 * 说  明：直流变交流
 *****************************************************************************
 */
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta)
{
    v_alpha_beta->Valpha = cos_sin.Cos * v_dq.Vd - cos_sin.Sin * v_dq.Vq;
    v_alpha_beta->Vbeta  = cos_sin.Sin * v_dq.Vd + cos_sin.Cos * v_dq.Vq;
}


/*
 *****************************************************************************
 * 功  能：SVPWM计算
 * 形  参：alpha_beta电压以及母线电压、定时器周期
 * 说  明：根据alpha_beta电压计算三相占空比
 *****************************************************************************
 */
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm)
{
    int sector;
    float Tx, Ty, T, Ta, Tb, Tc;

    // 矢量扇区选择    (N=4C+2B+A)
    sector = 0;
    if (v_alpha_beta.Vbeta > 0.0f)
        sector = 1;
    if ((1.7320508f * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0f > 0.0f)
        sector += 2;
    if ((-1.7320508f * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0f > 0.0f)
        sector += 4;

    // 计算矢量作用时间
    switch (sector)
    {
        case 1: // 扇区1
            Tx =  1.7320508f * v_alpha_beta.Valpha * Tpwm / Udc;
            Ty = (-1.7320508f * v_alpha_beta.Valpha / 2.0f + 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            break;
        case 2: // 扇区2
            Tx = ( 1.7320508f * v_alpha_beta.Valpha / 2.0f + 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            Ty = (-1.7320508f * v_alpha_beta.Valpha / 2.0f + 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            break;
        case 3: // 扇区3
            Tx = -1.7320508f * v_alpha_beta.Valpha * Tpwm / Udc;
            Ty = ( 1.7320508f * v_alpha_beta.Valpha / 2.0f + 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            break;
        case 4: // 扇区4
            Tx = -1.7320508f * v_alpha_beta.Valpha * Tpwm / Udc;
            Ty = (-1.7320508f * v_alpha_beta.Valpha / 2.0f - 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            break;
        case 5: // 扇区5
            Tx = (-1.7320508f * v_alpha_beta.Valpha / 2.0f - 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            Ty = ( 1.7320508f * v_alpha_beta.Valpha / 2.0f - 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            break;
        case 6: // 扇区6
            Tx = ( 1.7320508f * v_alpha_beta.Valpha / 2.0f - 1.5f * v_alpha_beta.Vbeta) * Tpwm / Udc * 2.0f / 1.7320508f;
            Ty = -1.7320508f * v_alpha_beta.Valpha * Tpwm / Udc;
            break;
        default:
            Tx = 0.0f;
            Ty = 0.0f;
            break;
    }

    /* 过调制处理：Tx+Ty超出Tpwm时等比例缩放，不能截断负值（会破坏矢量方向） */
    T = Tx + Ty;
    if (T > Tpwm)
    {
        Tx = Tx * Tpwm / T;
        Ty = Ty * Tpwm / T;
    }
    /* 过调制后Tx/Ty仍可能因浮点误差略负，在此做安全下限 */
    if (Tx < 0.0f) Tx = 0.0f;
    if (Ty < 0.0f) Ty = 0.0f;

    // 计算三相比较值
    Ta = (Tpwm - Tx - Ty) / 4.0f;
    Tb = Ta + Tx / 2.0f;
    Tc = Tb + Ty / 2.0f;

    // 根据扇区赋值比较寄存器
    switch (sector)
    {
        case 1:
            rtY.Tcmp1 = Tb;
            rtY.Tcmp2 = Ta;
            rtY.Tcmp3 = Tc;
            break;
        case 2:
            rtY.Tcmp1 = Ta;
            rtY.Tcmp2 = Tc;
            rtY.Tcmp3 = Tb;
            break;
        case 3:
            rtY.Tcmp1 = Ta;
            rtY.Tcmp2 = Tb;
            rtY.Tcmp3 = Tc;
            break;
        case 4:
            rtY.Tcmp1 = Tc;
            rtY.Tcmp2 = Tb;
            rtY.Tcmp3 = Ta;
            break;
        case 5:
            rtY.Tcmp1 = Tc;
            rtY.Tcmp2 = Ta;
            rtY.Tcmp3 = Tb;
            break;
        case 6:
            rtY.Tcmp1 = Tb;
            rtY.Tcmp2 = Tc;
            rtY.Tcmp3 = Ta;
            break;
        default:
            rtY.Tcmp1 = 0.0f;
            rtY.Tcmp2 = 0.0f;
            rtY.Tcmp3 = 0.0f;
            break;
    }
}


/*
 *****************************************************************************
 * 功  能：一维标量卡尔曼滤波器（用于Ia/Ib）
 * 说  明：预测 -> 更新两步递推，抑制ADC采样噪声
 *****************************************************************************
 */
void Kalman_Init(Kalman_DEF* kf, float Q, float R, float init_val)
{
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0f;
    kf->x = init_val;
}

float Kalman_Update(Kalman_DEF* kf, float measurement)
{
    /* 预测步 */
    float P_prior = kf->P + kf->Q;

    /* 更新步 */
    float K = P_prior / (P_prior + kf->R);
    kf->x = kf->x + K * (measurement - kf->x);
    kf->P = (1.0f - K) * P_prior;

    return kf->x;
}


/*
 *****************************************************************************
 * 功  能：一阶IIR低通滤波器（用于Id/Iq）
 * 说  明：y[n] = alpha*x[n] + (1-alpha)*y[n-1]
 *         alpha越小滤波越强，计算量仅2次乘加，适合15kHz高频中断
 *****************************************************************************
 */
void IIR_Init(IIR_DEF* f, float alpha, float init_val)
{
    f->alpha = alpha;
    f->y     = init_val;
}

float IIR_Update(IIR_DEF* f, float x)
{
    f->y = f->alpha * x + (1.0f - f->alpha) * f->y;
    return f->y;
}
