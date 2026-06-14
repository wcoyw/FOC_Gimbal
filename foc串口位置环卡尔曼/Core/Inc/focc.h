#ifndef __FOCC_H__
#define __FOCC_H__
#include "main.h"
#include "foc.h"
#include <math.h>
#define PI 3.14159265358979f
typedef struct
{
   float theta;
   float udc;
   float tpwm;

} ExtX;
typedef struct
{
     float Tcmp1;        /* SVPWM输出U项电流 */
     float Tcmp2;        /* SVPWM输出V项电流 */
     float Tcmp3;        /* SVPWM输出W项电流 */
} ExtY;
typedef struct
{
   float Vd;
   float Vq;

} Voltage_DQ_DEF;
typedef struct
{
    float Cos;
    float Sin;

} Transf_Cos_Sin_DEF;
typedef struct
{
    float Valpha;
    float Vbeta;

} Voltage_Alpha_Beta_DEF;
typedef struct
{
    float Ia;
    float Ib;
    float Ic;

} Current_ABC_DEF;
typedef struct
{
    float Ialpha;
    float Ibeta;
}  Current_Alpha_Beta_DEF;
typedef struct
{
    float Id;
    float Iq;

} Current_DQ_DEF;
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float intergral;
    float last_error;
    float out_max;

} PID_DEF;

typedef struct
{
    float Q;
    float R;
    float p;
    float X;

}Kalman_DEF;
typedef struct
{
    float alpha;  /* 平滑系数 */
    float y;      /* 上一拍输出 */
} IIR_DEF;

extern volatile ExtX rtX;               /* 输入结构体变量 */
extern volatile ExtY rtY;






#endif /* __FOCC_H__ */