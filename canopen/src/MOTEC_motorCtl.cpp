//#include "CANopen.h"
#include "canopen/MOTEC_motorCtl.h"
#include "canopen/techMotorCtl.h"
#include "canopen/type.h"
#include <stdio.h>    //usb_can函数
#include <unistd.h>
#include <asm/types.h>
#include "ros/ros.h"

u8 Enquire_ID=0;//查询id
u32 motorEnableBitmap = 0; //标记驱动电机是否锁死

TPCANMsg Message;
TPCANStatus Status;

/**void THETA_run(void)**/
float ss[4];
float distance1=0.0;
float distance2=0.0;
u8 num[4];
u16 k2=1;
float speed[4];

float repairvalue=0;//转向函数修正值
float  repairvalue2=0;
float  repairvalue3=0;	 

//slaveID:需要配置的从站ID
u8 set_mode_control(void)
{
    Message.ID = 0;
	Message.LEN = 2;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0]=0x10;
    Message.DATA[1]=0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    printf("set mode control CAN: %d\n",(int)Status);
	return 0;
}

// 驱动轮锁死   
u8 enable_model_set(u8 slaveID)
{
    int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x00;
	Message.DATA[2] = 0x20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x01;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    printf("enable_model_set: %x---%d\n",(int)Status,slaveID);
	usleep(1000);
	return 0;
}

// 解除驱动轮锁死
u8 disable_model_set(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x00;
	Message.DATA[2] = 0x20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x00;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;
}


//MOTEC速度设置函数
u8 set_motor_speed(u8 slaveID, float data_speed)
{
    int nodeID = 0x0C;
	short speed;
	speed =data_speed*2013;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0xff;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = speed;
	Message.DATA[5] = (speed >> 8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
	motor_start(slaveID);
	// ROS_INFO("car speed :%8f",data_speed);
	return 0;
}


u8 motor_start(u8 slaveID)
{
    int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x01;
	Message.DATA[2] = 0x20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x01;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

u8 motor_stop(u8 slaveID)
{
    int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x02;
	Message.DATA[2] = 0x20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x01;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;
}

//
u8 set_motor_to_move(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x06;
	Message.DATA[2] = 0x20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x00;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;
}

// 位置模式
u8 set_motor_position(u8 slaveID, int pos)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x23;
	Message.DATA[1] = 0x7a;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = (u8)pos;
	Message.DATA[5] = (u8)(pos>>8);
	Message.DATA[6] = (u8)(pos>>16);
	Message.DATA[7] = (u8)(pos>>24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
	set_motor_to_move(slaveID);
	motor_start(slaveID);
    return 0;
}

//motec电机工作模式命令函数
u8 motec_mode(u8 slaveID,u8 mode_x)	
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x60;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = mode_x;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;
}


u8 set_motor_posspeed(u8 slaveID, float Maxspeed)//Maxspeed 单位为m/s
{
	short maxspeed;
	maxspeed=Maxspeed*2013;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x7F;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = (u8)maxspeed;
	Message.DATA[5] = (u8)(maxspeed>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;
}


//程序名称：位置查询函数
u8 AGV_position_Enquire(u8 slaveID)  
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x63;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x00;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

//程序名称：速度查询函数  slaveID: 1-LF; 2 RF; 3-LR; 4-RR 
u8 AGV_SPEED_Enquire(u8 slaveID) 
{
	int nodeID = 0x0C;
	Enquire_ID=slaveID;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	if((slaveID >= 0x585)&&(slaveID <= 0x588))
		{
			Message.DATA[1] = 0x69;
		}
	else
		{
			Message.DATA[1] = 0x6c;
		}
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x00;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

//加速度设置函数 :slaveID: 1-LF; 2 RF; 3-LR; 4-RR 
u8 Set_motor_acc(u8 slaveID, float acc)
{
	short Motec_acc;
	Motec_acc=acc;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x82;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_acc;
	Message.DATA[5] = (Motec_acc>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}
 
//加速度设置函数 :slaveID: 1-LF; 2 RF; 3-LR; 4-RR 
u8 Set_TKmotor_acc(u8 slaveID, unsigned int acc)
{
	unsigned int Motec_acc;
	Motec_acc=acc;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x83;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_acc;
	Message.DATA[5] = (Motec_acc>>8);
	Message.DATA[6] = (Motec_acc>>16);
	Message.DATA[7] = (Motec_acc>>24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}
 //减速度设置函数 :slaveID: 1-LF; 2 RF; 3-LR; 4-RR T曲线减速度
u8 Set_TKmotor_decc(u8 slaveID, unsigned int decc)
{

	unsigned Motec_decc;
	Motec_decc=decc;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x84;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_decc;
	Message.DATA[5] = (Motec_decc>>8);
	Message.DATA[6] = (Motec_decc>>16);
	Message.DATA[7] = (Motec_decc>>24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}
 //减速度设置函数 :slaveID: 1-LF; 2 RF; 3-LR; 4-RR T曲线减速度
u8 Set_motor_decc(u8 slaveID, float decc)
{

	short Motec_decc;
	Motec_decc=decc;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x83;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_decc;
	Message.DATA[5] = (Motec_decc>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

 //减速度设置函数 :slaveID: 1-LF; 2 RF; 3-LR; 4-RR 停止减速度
u8 Set_motor_decc_stop(u8 slaveID, float decc)
{

	short Motec_decc;
	Motec_decc=decc;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x69;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_decc;
	Message.DATA[5] = (Motec_decc>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}


u8 Set_motor_rateCurrent(u8 slaveID, float current)
{

	short Motec_CUR;
	Motec_CUR=current;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x75;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_CUR;
	Message.DATA[5] = (Motec_CUR>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

u8 Set_motor_maxCurrent(u8 slaveID, float current)
{

	short Motec_CUR;
	Motec_CUR=current;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x73;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_CUR;
	Message.DATA[5] = (Motec_CUR>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

u8 Set_motor_I2Ttime(u8 slaveID, float time)
{

	short Motec_CUR;
	Motec_CUR=time;
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x70;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = Motec_CUR;
	Message.DATA[5] = (Motec_CUR>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

u8 error_clear(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x05;
	Message.DATA[2] = 0x20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x01;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

 //错误代码查询
u8 error_code(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x0B;
	Message.DATA[2] = 0X20;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0X00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

 //错误代码查询
u8 current_read(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
	//ROS_INFO("motoID = %x",nodeID);
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x78;
	Message.DATA[2] = 0X60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0X00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

  //错误代码查询
u8 current_read_TK(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
	//ROS_INFO("motoID = %x",nodeID);
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x1c;
	Message.DATA[2] = 0X22;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0X00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}


u8 max_current_read(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x73;
	Message.DATA[2] = 0X60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0X00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}


u8 rate_current_read(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x75;
	Message.DATA[2] = 0X60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0X00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}

u8 I2T_time_read(u8 slaveID)
{
	int nodeID = 0x0C;
    nodeID <<= 7;
    nodeID |= slaveID;
    Message.ID = nodeID;
    Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x70;
	Message.DATA[2] = 0X60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0X00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
    return 0;	
}


//驱动电机使能失效
void Motec_Disable(void)
{
	disable_model_set(1);		//所有驱动轮使能失效
	disable_model_set(2);
	disable_model_set(3);
	disable_model_set(4);
}

//程序功能：驱动电机使能
void Motec_Enable(void)
{
	enable_model_set(1);		//所有驱动轮使能失效
	enable_model_set(2);
	enable_model_set(3);
	enable_model_set(4);
}


//驱动电机停止
void Stop_AGV(void)
{
	motor_stop(1);//整车驱动器停止
	motor_stop(2);
	motor_stop(3);
	motor_stop(4);
}

//整车前进、后退指令
void Speed_AGV(float speed)
{
	// ROS_INFO("speed:-----%f-------",speed);
	set_motor_speed(2, -speed);							
	set_motor_speed(1, speed);
	set_motor_speed(4, -speed);
	set_motor_speed(3, speed); // 匀速前进
}


void Shift_AGV(float speed)
{
	// ROS_INFO("speed:-----%f-------",speed);
	set_motor_speed(1, -speed);
    set_motor_speed(2, -speed);
    set_motor_speed(3, speed);
    set_motor_speed(4, speed);
}



/*******************************************************************
name	:FrontalWheel_GoAhead
funtion	:实现小车前轮匀速向前运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void FrontalWheel_GoAhead(u8 mode,int arg)
{
	static u8 pos_mode=1;
	static u8 speed_mode=1;
	if(mode==0)//速度模式
		{
			if(speed_mode==1)
			{
				set_mode_control();			//进入PO模式
				disable_model_set(1);	//motec电机ID，使能失效
				disable_model_set(2);	//motec电机ID，使能失效

				motec_mode(1,0x03);		//模式选择
				motec_mode(2,0x03);		//模式选择

				enable_model_set(1);	//motec电机ID，使能
				enable_model_set(2);	//motec电机ID，使能
				enable_model_set(3);	//motec电机ID，使能
				enable_model_set(4);	//motec电机ID，使能

				speed_mode=0;
				pos_mode=1;
			}

			set_motor_speed(1,((float)arg)/100 );			//后退
			set_motor_speed(2,((float)-arg)/100 );
		}

		else if(mode==1)
		{

			if(pos_mode==1)
			{
				set_mode_control();			//进入PO模式
				disable_model_set(1);	//motec电机ID，使能失效
				disable_model_set(2);	//motec电机ID，使能失效
					
				motec_mode(1,0x01);	//模式选择
				motec_mode(2,0x01);	//模式选择

				enable_model_set(1);	//motec电机ID，使能
				enable_model_set(2);	//motec电机ID，使能
				enable_model_set(3);	//motec电机ID，使能
				enable_model_set(4);	//motec电机ID，使能

				pos_mode=0;
				speed_mode=1;
			}
		set_motor_posspeed(1,0.1);
		set_motor_posspeed(2,0.1);//设置位置模式最大速度
		//读取驱动器位置信息

		set_motor_position(1, arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
		set_motor_position(2, -arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	}
}

/*******************************************************************
name	:RearWheel_GoAhead
funtion	:实现小车后轮匀速向前运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void RearWheel_GoAhead(u8 mode,int arg)
{
	static u8 pos_mode=1;
	static u8 speed_mode=1;
	if(mode==0)//速度模式
	{
	if(speed_mode==1)
	{
	set_mode_control();			//进入PO模式

	disable_model_set(3);	//motec电机ID，使能失效	
	disable_model_set(4);	//motec电机ID，使能失效
		
	motec_mode(3,0x03);	//模式选择
	motec_mode(4,0x03);	//模式选择
	
	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能		
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

	speed_mode=0;
	pos_mode=1;
	}

	set_motor_speed(3,((float)arg)/100 );//前进
	set_motor_speed(4,((float)-arg)/100 );
}

else if(mode==1)
{

	if(pos_mode==1)
	{
	set_mode_control();			//进入PO模式

	disable_model_set(3);	//motec电机ID，使能失效
	disable_model_set(4);	//motec电机ID，使能失效
		
	motec_mode(3,0x01);	//模式选择
	motec_mode(4,0x01);	//模式选择

	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

		pos_mode=0;
		speed_mode=1;
	}
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能
	//读取驱动器位置信息
	set_motor_posspeed(3,0.1);
	set_motor_posspeed(4,0.1);//设置位置模式最大速度
	
	set_motor_position(3, arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(4, -arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)	
	}

}

/*******************************************************************
name	:FrontalWheel_GoBack
funtion	:实现小车前轮匀速向后运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void FrontalWheel_GoBack(u8 mode,int arg)
{
	static u8 pos_mode=1;
	static u8 speed_mode=1;
	if(mode==0)//速度模式
{
	if(speed_mode==1)
	{
	set_mode_control();			//进入PO模式
	disable_model_set(1);	//motec电机ID，使能失效
	disable_model_set(2);	//motec电机ID，使能失效
		
	motec_mode(1,0x03);		//模式选择
	motec_mode(2,0x03);		//模式选择
		
	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

	speed_mode=0;
	pos_mode=1;
	}

	set_motor_speed(1,((float)-arg)/100 );			//后退
	set_motor_speed(2,((float)arg)/100 );
}

else if(mode==1)
{

	if(pos_mode==1)
	{
	set_mode_control();			//进入PO模式
	disable_model_set(1);	//motec电机ID，使能失效
	disable_model_set(2);	//motec电机ID，使能失效
		
	motec_mode(1,0x01);	//模式选择
	motec_mode(2,0x01);	//模式选择

	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

		pos_mode=0;
		speed_mode=1;
	}
	
	//读取驱动器位置信息
	set_motor_posspeed(1,0.1);
	set_motor_posspeed(2,0.1);//设置位置模式最大速度
	set_motor_position(1, -arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(2, arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
}

}

/*******************************************************************
name	:RearWheel_GoAhead
funtion	:实现小车后轮匀速向后运动
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void RearWheel_GoBack(u8 mode,int arg)
{
	static u8 pos_mode=1;
	static u8 speed_mode=1;
	if(mode==0)//速度模式
	{
		if(speed_mode==1)
		{
			set_mode_control();			//进入PO模式

			disable_model_set(3);	//motec电机ID，使能失效	
			disable_model_set(4);	//motec电机ID，使能失效
				
			motec_mode(3,0x03);	//模式选择
			motec_mode(4,0x03);	//模式选择
			
			enable_model_set(1);	//motec电机ID，使能
			enable_model_set(2);	//motec电机ID，使能		
			enable_model_set(3);	//motec电机ID，使能
			enable_model_set(4);	//motec电机ID，使能

			speed_mode=0;
			pos_mode=1;
		}

		set_motor_speed(3,((float)-arg)/100 );//后退
		set_motor_speed(4,((float)arg)/100 );
	}

	else if(mode==1)
	{

		if(pos_mode==1)
		{
		set_mode_control();			//进入PO模式

		disable_model_set(3);	//motec电机ID，使能失效
		disable_model_set(4);	//motec电机ID，使能失效
			
		motec_mode(3,0x01);	//模式选择
		motec_mode(4,0x01);	//模式选择

		enable_model_set(1);	//motec电机ID，使能
		enable_model_set(2);	//motec电机ID，使能	
		enable_model_set(3);	//motec电机ID，使能
		enable_model_set(4);	//motec电机ID，使能

		pos_mode=0;
		speed_mode=1;
		}
		//读取驱动器位置信息
		set_motor_posspeed(3,0.1);
		set_motor_posspeed(4,0.1);//设置位置模式最大速度
		
		set_motor_position(3, -arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
		set_motor_position(4, arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)	
	}
}

/*******************************************************************
name	:car_spin
funtion	:小车自当前位置旋转xx度
input	:LR_flag,旋转方向选取:0 顺时针旋转，1 逆时针旋转
		:angle 旋转的度数
output	:none
*******************************************************************/

void car_spin(u8 LR_flag, int angle)
{
	static u8 pos_mode;
			Motec_Disable();			//motec电机使能失效
			tech_turn_right_ex(5, 0);	//78°，实际值需计算
			tech_turn_left_ex(6, 0);
			tech_turn_left_ex(7, 0);
			tech_turn_right_ex(8, 0);
			usleep(1000);
			usleep(1000);
			usleep(1000);
			usleep(1000);
			//位置检测函数，到位开始运动
		if(pos_mode==0)
	{
			set_mode_control();			//进入PO模式

			motec_mode(1,0x01);			//位置模式选择
			motec_mode(2,0x01);	
			motec_mode(3,0x01);	
			motec_mode(4,0x01);	

			enable_model_set(1);	//motec电机ID，使能
			enable_model_set(2);	//motec电机ID，使能
			enable_model_set(3);	//motec电机ID，使能
			enable_model_set(4);	//motec电机ID，使能
			pos_mode=1;
		}
			Motec_Enable();
			if(LR_flag==0)
			{	
				set_motor_posspeed(1,0.1);
				set_motor_posspeed(2,0.1);
				set_motor_posspeed(3,0.1);
				set_motor_posspeed(4,0.1);//设置位置模式最大速度
				//读取驱动器位置信息				
				set_motor_position(1,angle*580.8);	//(angle*3.14/180)*1986.66*1000/596.6 );		//顺时针
				set_motor_position(2,angle*580.8);	//(angle*3.14/180)*1986.66*1000/596.6 );
				set_motor_position(3,angle*580.8);	//(angle*3.14/180)*1986.66*1000/596.6 );
				set_motor_position(4,angle*580.8);	// (angle*3.14/180)*1986.66*1000/596.6);
			}
			else if(LR_flag==1)
			{
				set_motor_posspeed(1,0.1);
				set_motor_posspeed(2,0.1);
				set_motor_posspeed(3,0.1);
				set_motor_posspeed(4,0.1);//设置位置模式最大速度
				//读取驱动器位置信息
				set_motor_position(1,-angle*580.8);//1000/596.6*(angle*3.14/180)*1986.66 );		//逆时针
				set_motor_position(2, -angle*580.8);//1000/596.6*(angle*3.14/180)*1986.66 );
				set_motor_position(3, -angle*580.8);//1000/596.6*(angle*3.14/180)*1986.66 );
				set_motor_position(4, -angle*580.8);//1000/596.6*(angle*3.14/180)*1986.66 );
			}

}

/*******************************************************************
name	:car_GoAhead
funtion	:小车匀速前进
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void car_GoAhead(u8 mode,int arg)
{
	static u8 pos_mode=1;
	static u8 speed_mode=1;
	if(mode==0)//速度模式
{
	if(speed_mode==1)
	{
	set_mode_control();			//进入PO模式
	disable_model_set(1);	//motec电机ID，使能失效
	disable_model_set(2);	//motec电机ID，使能失效
	disable_model_set(3);	//motec电机ID，使能失效		
	disable_model_set(4);	//motec电机ID，使能失效
		
	motec_mode(1,0x03);	//模式选择
	motec_mode(2,0x03);	//模式选择
	motec_mode(3,0x03);	//模式选择
	motec_mode(4,0x03);	//模式选择
		
	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

	speed_mode=0;
	pos_mode=1;
	}

	set_motor_speed(1,((float)arg)/100 );			//前进
	set_motor_speed(2,((float)-arg)/100 );
	set_motor_speed(3,((float)arg)/100 );
	set_motor_speed(4,((float)-arg)/100 );
}

else if(mode==1)
{

	if(pos_mode==1)
	{
	set_mode_control();			//进入PO模式
	disable_model_set(1);	//motec电机ID，使能失效
	disable_model_set(2);	//motec电机ID，使能失效
	disable_model_set(3);	//motec电机ID，使能失效
	disable_model_set(4);	//motec电机ID，使能失效

	motec_mode(1,0x01);	//模式选择
	motec_mode(2,0x01);	//模式选择
	motec_mode(3,0x01);	//模式选择
	motec_mode(4,0x01);	//模式选择

	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

		pos_mode=0;
		speed_mode=1;
	}
	set_motor_posspeed(1,0.1);
	set_motor_posspeed(2,0.1);
	set_motor_posspeed(3,0.1);
	set_motor_posspeed(4,0.1);
	//读取驱动器位置信息
	
	set_motor_position(1, arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)，距离/轮子周长*10000*10
	set_motor_position(2, -arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(3, arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(4, -arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)	

}

}


/*******************************************************************
name	:car_GoAhead
funtion	:小车匀速后退
input	:mode 运动模式 0 速度模式; 1 位置模式
		:arg 速度模式下,指定小车速度，单位:cm/s
		:arg 位置模式下，指定小车移动的距离，单位:MM
output	:none
*******************************************************************/
void car_GoBack(u8 mode,int arg)
{
	static u8 pos_mode=1;
	static u8 speed_mode=1;
	if(mode==0)//速度模式
{
	if(speed_mode==1)
	{
	set_mode_control();			//进入PO模式
	disable_model_set(1);	//motec电机ID，使能失效
	disable_model_set(2);	//motec电机ID，使能失效
	disable_model_set(3);	//motec电机ID，使能失效	
	disable_model_set(4);	//motec电机ID，使能失效
		
	motec_mode(1,0x03);	//模式选择
	motec_mode(2,0x03);	//模式选择
	motec_mode(3,0x03);	//模式选择
	motec_mode(4,0x03);	//模式选择
		
	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

		speed_mode=0;
		pos_mode=1;
	}

	set_motor_speed(1,((float)-arg)/100 );			//后退
	set_motor_speed(2,((float)arg)/100 );
	set_motor_speed(3,((float)-arg)/100 );
	set_motor_speed(4,((float)arg)/100 );
}

else if(mode==1)
{

	if(pos_mode==1)
	{
	set_mode_control();			//进入PO模式
	disable_model_set(1);	//motec电机ID，使能失效
	disable_model_set(2);	//motec电机ID，使能失效
	disable_model_set(3);	//motec电机ID，使能失效
	disable_model_set(4);	//motec电机ID，使能失效
	motec_mode(1,0x01);	//模式选择
	motec_mode(2,0x01);	//模式选择
	motec_mode(3,0x01);	//模式选择
	motec_mode(4,0x01);	//模式选择

	enable_model_set(1);	//motec电机ID，使能
	enable_model_set(2);	//motec电机ID，使能
	enable_model_set(3);	//motec电机ID，使能
	enable_model_set(4);	//motec电机ID，使能

	pos_mode=0;
	speed_mode=1;
	}
	
	//读取驱动器位置信息
	set_motor_posspeed(1,0.1);
	set_motor_posspeed(2,0.1);
	set_motor_posspeed(3,0.1);
	set_motor_posspeed(4,0.1);
	
	set_motor_position(1, -arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(2, arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(3, -arg*10000/(298.45)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)
	set_motor_position(4, arg*10000/(298.45)*10);//-arg*1000000/(596)*10);//计算行走的距离出所需脉冲数 arg*10000/(596.6)	
}

}


/*******************************************************************
name :wheel_lift
funtion :车轮举升
input :select 选中要举升的轮子，			//1LF 2RF 3LR 4RR
	  :degree 夹杆收缩袋的角度.			//正角度张开	//负角度缩回
output :命令执行成功返回0，失败返回1.
*******************************************************************/
u8 wheel_lift(u8 select, int degree)
{
	set_motor_position((select+8), -1*21111 * degree);//升降涡轮蜗杆传动比38 减速机减速比20 夹杆转1°，电机转760°

}


//所有电机初始化
void motorCtlInit(void)
{
    ROS_INFO("motorCtlInit!!!");
	set_mode_control();
    usleep(10000);
	enable_model_set(1);
	enable_model_set(2);
	enable_model_set(3);
	enable_model_set(4); // 1~4MOTECÇý¶¯µç»ú  
	enable_model_set(9);
	enable_model_set(10);
	enable_model_set(11);
	enable_model_set(12); // 9~12 ŸÙÉýµç»ú
	tect_motor_init();
}

void test_motor()
{
	static int flag=0;
	if(flag==0)
	car_GoAhead(0,-20);
	for(int i=1;i<5;i++)
	{
		AGV_SPEED_Enquire(i);
		usleep(1000);
	}
	flag++;
	if(flag==300)
	Stop_AGV();
	if(flag>=500)
	flag=0;
}




