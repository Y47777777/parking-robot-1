#ifndef __SPORT_CONTROL__
#define __SPORT_CONTROL__

#define SendV_Step 			0.015   //发送速度的步长
/*
#define CarMode_GoAhead	1
#define CarMode_GoBack		2
#define CarMode_RearWheel_GoBack	3
#define CarMode_FrontWheel_GoBack	4
#define CarMode_RaarWheel_GoAhead	5
#define CarMode_FrontWheel_GoAhead	6 
*/

#define CarMode_GoAhead	2
#define CarMode_GoBack		1
#define CarMode_RearWheel_GoBack	6
#define CarMode_FrontWheel_GoBack	5
#define CarMode_RaarWheel_GoAhead	4
#define CarMode_FrontWheel_GoAhead	3

#define CmdLaser_CheckWheel_1 0x01 //第一次检测到轮胎边沿
#define CmdLaser_CheckWheel_2 0x02 //第二次检测到轮胎边沿
#define CmdLaser_CheckWheel_3 0x03 //第三次检测到轮胎边沿

#include "canopen/type.h"
#include "ros/ros.h"

typedef struct{
	float s_a0;
	float s_a1;
	float s_a2;
	float s_a3;
	float s_a4; //四次项系数
	float s_a5; //五次项系数
}STR_five_Sa; //位置轨迹的五次项系数

/*机器人尺寸参数*/
typedef struct{
float ROB_laser_height; //激光传单器离地面的高度 90 mm
float	laser_RobWheel_distance; //激光传感器离机器人前轮中心位置的距离
float	Rob_WheelDistance_max; //机器人最大轴距
float	Rob_WheelDistance_min; //机器人最小轴距
float	Rob_JiaGan_Banjing; //机器人夹杆的半径
float Rob_JiaGan_GaoDu; //机器人夹杆中心位置离地面的高度
float Rob_len; //机器人长度
float Rob_width; //机器人宽度
}ST_Rob_DimensionPara_float;

/*车轮数据,添加PID和曲线算法之后*/
typedef struct{
	float last_p; //上一次小车行走的位置
	float wheel_len; //前轮弦长
	float wheel_radius; //前轮半径
	float rob_car_len; //机器人和汽车轮子之间的距离
	float car_len;  //汽车轴距 
}ST_wheel_data_pid; 


extern volatile char control_cycle;
extern ST_Rob_DimensionPara_float Rob_DimensionPara_float; //?ú?÷è?3?′?2?êy 
extern volatile u32 tickclock;
//extern void get_Gui_Ji(float vmax, float amax, float s, char dir, float V0, float Vt, float step);
extern float car_go_vxs(u8 sport_mode, float user_s, float v_now, float amax);
extern u8 send_flag;  // 数据发送标志
extern volatile u8 Cmd_Laser ; //激光传感器检测到轮胎发出的命令
extern u8 wheel_aim(void);
extern void tset_auto_lift_PID(void);
extern void car_Sheng_Suo(u16  distance,  float Vmax, u8 mode);







#endif
