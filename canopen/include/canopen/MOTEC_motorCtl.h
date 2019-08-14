#ifndef ____MOTEC_MOTORCtl_H___
#define ____MOTEC_MOTORCtl_H___

//#include "CANopen.h"
//#include "techMotorCtl.h"
#include "canopen/type.h"
#include <canopen/PCANBasic.h>
#define TIME_OUT	15

extern TPCANMsg Message;
extern TPCANStatus Status;
extern u8 Enquire_ID;

//slaveID:需要配置的从站ID
u8 set_mode_control(void); 
u8 enable_model_set(u8 slaveID);
u8 disable_model_set(u8 slaveID);
u8 set_motor_speed(u8 slaveID, float speed);
u8 motor_start(u8 slaveID);
u8 motor_stop(u8 slaveID);
u8 set_motor_to_move(u8 slaveID);
u8 set_motor_position(u8 slaveID, int pos);
u8 motec_mode(u8 slaveID,u8 mode_x)	;
u8 set_motor_posspeed(u8 slaveID, float Maxspeed);
void Motec_Disable(void);
void Motec_Enable(void);
void Stop_AGV(void);
void Speed_AGV(float speed);
void Shift_AGV(float speed);
void FrontalWheel_GoAhead(u8 mode,int arg);
void RearWheel_GoAhead(u8 mode,int arg);
void FrontalWheel_GoBack(u8 mode,int arg);
void RearWheel_GoBack(u8 mode,int arg);
extern void car_spin(u8 LR_flag, int angle);
void car_GoAhead(u8 mode,int arg);
void car_GoBack(u8 mode,int arg);
void car_stop(void);
u8 wheel_lift(u8 select, int degree);
u8 AGV_position_Enquire(u8 slaveID) ;
u8 AGV_SPEED_Enquire(u8 slaveID) ;
u8 Set_motor_acc(u8 slaveID, float acc);
u8 Set_motor_decc(u8 slaveID, float decc);
u8 Set_TKmotor_acc(u8 slaveID, unsigned int acc);
u8 Set_TKmotor_decc(u8 slaveID, unsigned int decc);
u8 Set_motor_decc_stop(u8 slaveID, float decc);//急停减速度
u8 Set_motor_rateCurrent(u8 slaveID, float current);
u8 Set_motor_maxCurrent(u8 slaveID, float current);
u8 Set_motor_I2Ttime(u8 slaveID, float time);
u8 error_code(u8 slaveID);
u8 current_read(u8 slaveID);
u8 max_current_read(u8 slaveID);
u8 rate_current_read(u8 slaveID);
u8 I2T_time_read(u8 slaveID);
extern u8 current_read_TK(u8 slaveID);


//所有电机初始化
void motorCtlInit(void);

void test_motor();


#endif

