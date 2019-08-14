/*******************************************************************************
file naime	:auto_lift.h	
function	:为auto_lift.c文件做接口声明
author		:ZWL
date		:20180809
*******************************************************************************/
#ifndef __AAUTO_LIFT_H__
#define __AAUTO_LIFT_H__

#include "canopen/type.h"

#define CAR_LEN_MAX 3000 // 最大车长 3000MM
#define CAR_LEN_MIN 2400 //最短车长 2400MM

/**********错误码定义*********/
#define CAR_LEN_ERR 0X01


/**********电机运动模式*********/
#define PO_MODE 0x01 //位置模式
#define SP_MODE 0x00 //速度模式

/**********定时器6工作模式定义*********/
#define tim6_all_work 0x02
#define tim6_no_work 0x03
#define tim6_can_work 0x00
#define tim6_laser_work 0x01

/**********机器人参数定义*********/

/*机器人尺寸参数*/
typedef struct{
u16 ROB_laser_height; //激光传单器离地面的高度 90 mm
u16	laser_RobWheel_distance; //激光传感器离机器人前轮中心位置的距离
u16	Rob_WheelDistance_max; //机器人最大轴距
u16	Rob_WheelDistance_min; //机器人最小轴距
u16	Rob_JiaGan_Banjing; //机器人夹杆的半径
u16 Rob_JiaGan_GaoDu; //机器人夹杆中心位置离地面的高度
u16 Rob_len; //机器人长度
u16 Rob_width; //机器人宽度
}ST_Rob_DimensionPara;

/*车轮数据*/
typedef struct{
	u16 front_wheel_len; //前轮弦长
	u16 rear_wheel_len; //后轮弦长
	u16 front_wheel_radius; //前轮半径
	u16 rear_wheel_radius; //后轮半径
	u16 rob_car_len;		//机器人和汽车轮子之间的距离
	u32 car_len;  //汽车轴距
}ST_wheel_data; 

/*车轮边沿触发单个传感器的时间*/
typedef struct{
	u32 t1;
	u32 t2;
	u32 t3;
	u32 t4;
	u16 laser_CarWheel_distance_F; //激光传感器到前轮的距离
	u16 laser_CarWheel_distance_R;//激光传感器到后轮的距离
	u8 t1_count; //从无到有计数
	u8 t0_count; // 从有道无计数
}ST_OneTrigerTime;

typedef struct{
	u32 L_time; //左传感器时间
	u16 L_distance; //左边距离
	u32 R_time; // 右传感器时间
	u16 R_distance; //右边距离
}ST_Set_right_time;

/*车轮触发四个传感器时间*/
typedef struct{
	ST_OneTrigerTime FL; //左前方传感器
	ST_OneTrigerTime FR; //右前方传感器
	ST_OneTrigerTime RL; //左后方传感器
	ST_OneTrigerTime RR; //右后方传感器
	u8 inclined_mode; //小车倾斜情况: 0x00向左前倾， 0x01 向右前倾 
}ST_triger_time;

/*
                   n
OUT=(Kp*Ek) + (Ki∑Ek) + (KD(EK-Ek-1)) +OUT0           
                   k=0
*/
typedef struct{

	float set_v; //用户设定值
	float FB_v; //反馈值 feedback
	float Kp;
	float Ti;
	float Td;
	float Dt; //微分时间
	float ctrl_T; //控制周期
	float Out_0; //0状态输出

	float Ek;  //当前偏差
	float Ek_1; //上一次偏差
	float SEk; //累计偏差

	float PID_out; //PID计算后的输出值
	float P_out;
	float I_out;
	float D_out;
	
}ST_PID_arg;


extern ST_Rob_DimensionPara Rob_DimensionPara; //机器人尺寸参数 
extern ST_OneTrigerTime wheel_time_buff[4]; //LF RF LR RR;
extern volatile u32 timer_count;
extern u32 controlMode;
extern u32 ps2KeyBitmap;
extern u8 timer_mode; //定时器工作模式: 0x00--> 车底测距模式  0x01--> 轮前矫正模式
extern ST_Set_right_time Set_right_time; //姿态矫正计时器
extern u16 car_len; //车身长度 
extern u16 car_len_buff[3];
extern u8 car_len_count;
extern u8 tim6_mode[2];
extern u8 FB_speed_flag; //得到速度反馈为1，否则为0

extern u16 set_WheelSpeedBuff_count;
extern float set_WheelSpeedBuff[];
extern u32 send_speed_count ;
extern ST_OneTrigerTime Laser_signal; //激光传感器信号，用于非匀速曲线模式
 

extern void sys_source_init(void);
extern void test_timer_6(void);
extern void auto_lift_test(void);
extern void test_car_lift(void); 
extern void SongKai_ChiLun(u8  select);
extern void SuoJIng_ChiLun(u8  select);
extern void test_Sheng_Suo(void);
extern void test_check_car_len(void);
extern void car_status_init(void);
extern void speed_pid(void) ;
extern void PID_arg_init(void);
extern void car_lift(void);
extern void car_lay_down(void);  
extern void laser_open_close(u8 cmd);

#endif


