#ifndef ___MOTOR_CURVE_CONTROL_H___
#define ___MOTOR_CURVE_CONTROL_H___

#include "canopen/techMotorCtl.h"
#include "canopen/type.h"

struct S_CURVE_PARAM
{
        float Tj1;
        float Ta;
        float Tv;
        float Tj2;
        float Td;
        float T;
        float q0;
        float q1;
        float v0;
        float v1;
        float vlim;
        float alima;
        float alimd;
        float jmax;
        float jmin;
        float sigma;
};

float velocity(float t);
int s_curve_calc(float q0,float q1,float v0,float v1,float v_max,float a_max,float j_max);
int steering_motor_s_curve(float q0,float q1,float v0,float v1,float v_max,float a_max,float j_max);



#endif