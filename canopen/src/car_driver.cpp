
#include "canopen/car_driver.h"
#include "canopen/MOTEC_motorCtl.h"
#include <string.h>


STR_wheel_speed FB_WheelSpeed; //小车的反馈速度


/*******************************************************************
name	:read_wheel_speed
funtion	:读取轮子速度
input	:cmd_mode: 1 读取单个轮子速度，
		:digree 旋转的度数
output	:成功返回0 失败返回1
*******************************************************************/
void read_wheel_speed(u8 cmd_mode, u8 cmd_select, STR_wheel_speed* wheel_speed)
{
	;
}



/*维护一个数组，进来一个值放到数组末尾，同时将数组里的其他值整体前移,即数组左移*/
int  buff_shift_left(float *buff,int size,float data)
{
	float temp_buff[2048] = {0};
	if(size >= 2048)
		return 1;
	memcpy(temp_buff,buff+1,(sizeof(buff[0])*(size-1))); 	
	temp_buff[size-1] = data;	
	memcpy(buff,temp_buff,sizeof(buff[0])*size);
	return 0;		
}


/*******************************************************************
name	:get_FrontWheel_speed
funtion	:获取小车前轮速度
input	:none
output	:speed
*******************************************************************/
float get_FrontWheel_speed(void)
{
	/*
	1、开启中断，允许发送速度查询指令
	2、到can中断里读取速度，且写入标志位
	3、判断两个轮子的速度，如果差值不大，则视为同步取平均
	*/
	AGV_SPEED_Enquire(1);
	AGV_SPEED_Enquire(2);
}

/*******************************************************************
name	:get_ReartWheel_speed
funtion	:获取小车后轮速度
input	:none
output	:speed
*******************************************************************/
float get_ReartWheel_speed(void)
{
	AGV_SPEED_Enquire(3);
	AGV_SPEED_Enquire(4);
}


/*******************************************************************
name	:get_car_speed
funtion	:获取小车速度
input	:none
output	:speed
*******************************************************************/
float get_car_speed(void)
{
	AGV_SPEED_Enquire(1);
	AGV_SPEED_Enquire(2);
	AGV_SPEED_Enquire(3); 
	AGV_SPEED_Enquire(4);
}


/*******************************************************************
name	:set_FrontWheel_speed
funtion	:设置小车前轮速度
input	:speed: 小车前轮速度
output	:成功返回0 失败返回1
*******************************************************************/
void set_FrontWheel_speed(float speed)
{
	set_motor_speed(1,speed);
	set_motor_speed(2,speed);
}


/*******************************************************************
name	:set_RearWheel_speed
funtion	:设置小车后轮速度
input	:speed: 小车后轮速度
output	:成功返回0 失败返回1
*******************************************************************/
void set_RearWheel_speed(float speed)
{ 
	set_motor_speed(3,speed);
	set_motor_speed(4,speed);
}

/*******************************************************************
name	:set_car_speed
funtion	:设置小车速度
input	:speed: 小车速度
output	:成功返回0 失败返回1
*******************************************************************/
void set_car_speed(float speed)
{
	//printf("%8f  \r\n",speed);
	set_motor_speed(4,-speed); 
	set_motor_speed(3,speed);
	set_motor_speed(2,-speed);
	set_motor_speed(1,speed);  
} 

/*******************************************************************
name	:wheel_turn
funtion	:驱动轮偏转XX度
input	:direction 要转向的方向，0 顺时针旋转， 1 逆时针方向
		:digree 旋转的度数
output	:成功返回0 失败返回1
*******************************************************************/
u8 wheel_turn(u8 direction, u16 digree)
{    
	return 0;
}

/*******************************************************************
name	:set_acc
funtion	:设置小车加速度
input	:加速度
output	:
*******************************************************************/
void set_acc(float acc)
{
	Set_motor_acc(1, acc);
	Set_motor_decc(1, acc);
	Set_motor_acc(2, acc);
	Set_motor_decc(2, acc);
	Set_motor_acc(3, acc);
	Set_motor_decc(3, acc);   
	Set_motor_acc(4, acc);
	Set_motor_decc(4, acc);
}



#if 0
/*******************************************************************
name	:FrontalWheel_GoAhead
funtion	:实现小车前轮匀速向前运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void FrontalWheel_GoAhead(u8 mode,u32 arg)
{
;
}

/*******************************************************************
name	:RearWheel_GoAhead
funtion	:实现小车后轮匀速向前运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void RearWheel_GoAhead(u8 mode,u32 arg)
{
;
}

/*******************************************************************
name	:FrontalWheel_GoAhead
funtion	:实现小车前轮匀速向后运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void FrontalWheel_GoBack(u8 mode,u32 arg)
{
;
}

/*******************************************************************
name	:RearWheel_GoAhead
funtion	:实现小车后轮匀速向后运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void RearWheel_GoBack(u8 mode,u32 arg)
{
;
}

/*******************************************************************
name	:car_spin
funtion	:小车自当前位置旋转xx度
input	:LR_flag,旋转方向选取:0 顺时针旋转，1 逆时针旋转
		:angle 旋转的度数
output	:none
*******************************************************************/
void car_spin(u8 LR_flag, u16 angle)
{
	;
}

/*******************************************************************
name	:car_GoAhead
funtion	:小车匀速前进
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void car_GoAhead(u8 mode,u32 arg)
{
	;
} 

/*******************************************************************
name	:car_GoAhead
funtion	:小车匀速后退
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void car_GoBack(u8 mode,u32 arg)
{
	;
}

/*******************************************************************
name	:car_stop
funtion	:小车停止
input	:none
output	:none
*******************************************************************/
void car_stop(void)
{
	;
}




/*******************************************************************
name	:wheel_lift
funtion	:车轮举升
input	:select 选中要举升的轮子，1LF  2RF 3 LR 4RR
		:degree 夹杆收缩袋的角度
output	:命令执行成功返回0，失败返回1
*******************************************************************/
u8 wheel_lift(u8 select, u8 degree)
{
	;
}
#endif


