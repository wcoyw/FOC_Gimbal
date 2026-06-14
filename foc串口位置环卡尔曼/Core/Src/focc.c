#include "focc.h"
#include "as5600.h"
#include "foc.h"

volatile ExtX rtX;               /* 输入结构体变量 */
volatile ExtX rtY;
Transf_Cos_Sin_DEF Transf_Cos_Sin;         /* 当前角度计算三角函数 */
Voltage_DQ_DEF Voltage_DQ;                 /* DQ电压输出 */
Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta;
Current_ABC_DEF current_abc;
Current_Alpha_Beta_DEF current_alpha_beta;
Current_DQ_DEF current_dq;




PID_DEF speed_pid;
PID_DEF id_pid;
PID_DEF iq_pid;
PID_DEF pos_pid;
float target_velocity = 0.0f;   /* rad/s，目标速度 */
float target_position = 0.0f;   /* rad，目标累积角度 */
float iq_ref          = 0.0f;   /* 位置/速度环输出*/
static float pos_last_error  = 0.0f;
static float pos_integral     = 0.0f;
void FOC_Init(void)
{
   Voltage_DQ.Vd = 0.0f;
   Voltage_DQ.Vq = 0.0f;
    
   pos_pid.kp =5.0f;
   pos_pid.ki=0.0f;
   pos_pid.integral=0.0f;
   pos_pid.out_max=10.0f;
   pos_pid.out_min=-10.0f;
   pos_last_error=0.0f;
   pos_integral=0.0f;

   speed_pid.kp = 0.1f;
   speed_pid.ki = 0.5f;
   speed_pid.integral = 0.0f;
   speed_pid.out_max = 10.0f;
   speed_pid.out_min = -10.0f;

   id_pid.kp = 2.0f;
   id_pid.ki = 100.0f;
   id_pid.integral = 0.0f;
   id_pid.out_max = 12.0f; /* 最大d轴电压 */
   id_pid.out_min = -12.0f;

   iq_pid.kp = 2.0f;
   iq_pid.ki = 100.0f;
   iq_pid.integral = 0.0f;
   iq_pid.out_max = 12.0f; /* 最大q轴电压 */
   iq_pid.out_min = -12.0f;

}
void Position_PID_Calc(float target, float measured, float dt)
{
   float error =target -measured;
    pos_interal +=pos_pid.ki * error * dt*0.5;
    if(pos_interal >pos_pid.out_max) pos_interal =pos_pid.out_max;
    if(pos_interal <pos_pid.out_min) pos_interal =pos_pid.out_min;
    float  kd=0.05f;
    float deriv =kd*(error -pos_pid.last_error)/dt;
    float out =pos_pid.kp * error +pos_interal +deriv;
    if(out >pos_pid.out_max) out =pos_pid.out_max;
    if(out <pos_pid.out_min) out =pos_pid.out_min;
    pos_last_error =error;
    iq_ref =out;
}
 void Speed_PID_Calc(float target, float measured, float dt)
 {
    float error =target -measured;
    speed_pid.interal +=error *dt;
    if(speed_pid.interal >speed_pid.out_max/speed_pid.ki) speed_pid.interal =speed_pid.out_max/speed_pid.ki;
    if(speed_pid.interal <speed_pid.out_min/speed_pid.ki) speed_pid.interal =speed_pid.out_min/speed_pid.ki;
    float out =speed_pid.kp *error +speed_pid.ki*speed_pid.interal;
    if(out >speed_pid.out_max) out =speed_pid.out_max;
    if(out <speed_pid.out_min) out =speed_pid.out_min;
    iq_ref =out;

 }
void Clarke_Transform(Current_ABC_DEF i_abc, Current_Alpha_Beta_DEF *i_ab)  
{
     i_ab->Ialpha = i_abc.Ia;
     i_ab->Ibeta  = (i_abc.Ia + 2.0f * i_abc.Ib) / 1.7320508f;
}















