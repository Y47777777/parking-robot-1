#ifndef __CAR_DRIVER_H__
#define __CAR_DRIVER_H__

#include "canopen/type.h"

/*小车轮子速度*/
typedef struct{
	float LF_speed;
	float RF_speed;
	float LR_speed;
	float RR_speed;
}STR_wheel_speed;

extern int  buff_shift_left(float *buff,int size,float data);

extern void FrontalWheel_GoAhead(u8 mode,int arg);

extern void RearWheel_GoAhead(u8 mode,int arg);

extern void FrontalWheel_GoBack(u8 mode,int arg);

extern void RearWheel_GoBack(u8 mode,int arg);

extern void car_spin(u8 LR_flag, int angle);

extern void car_GoAhead(u8 mode,int arg);

extern void car_GoBack(u8 mode,int arg);

extern void car_stop(void);

extern u8 wheel_turn(u8 direction, u16 digree);

extern u8 wheel_lift(u8 select, int degree);
extern float get_FrontWheel_speed(void);
extern float get_ReartWheel_speed(void);
extern float get_car_speed(void);
extern void set_FrontWheel_speed(float speed);
extern void set_RearWheel_speed(float speed);
extern void set_car_speed(float speed);
extern void set_acc(float acc);

#endif
