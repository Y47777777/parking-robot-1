
//#include <canopen/motorCanl.h>
#include <unistd.h>
#include <asm/types.h>
#include <canopen/PCANBasic.h>
#include <canopen/MOTEC_motorCtl.h>
#include <canopen/type.h>
#include "canopen/techMotorCtl.h"
#include "ros/ros.h"



int curPosition[4] = {0, 0 ,0 ,0}; //记录四个转向电机方向
int teckPosition[4] = {0, 0 ,0 ,0}; //实时记录四个转向电机角度
int teckgetPosition[4] = {0, 0 ,0 ,0}; //初始化时获取位置

//启动远程节点
int tech_start_node(int slaveID)
{	
    TPCANMsg Message;
    TPCANStatus Status;
    Message.ID = 0;
	Message.LEN = 2;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0]=0x01;
    Message.DATA[1]=slaveID;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

//改变为ready to switch on 状态
int tech_ready_switch_on(int slaveID)
{
	TPCANMsg Message;
    TPCANStatus Status;
	int nodeId = 0x04; //TPDO1 function code 0x04
	nodeId <<= 7;
	nodeId |= slaveID;
    Message.ID = nodeId;
	Message.LEN = 2;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0]=0x06;
    Message.DATA[1]=0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

//改变为 Switch on状态
int tech_switch_on(int slaveID)
{
    TPCANMsg Message;
    TPCANStatus Status;
	int nodeId = 0x04; //TPDO1 function code 0x04
	nodeId <<= 7;
	nodeId |= slaveID;
    Message.ID = nodeId;
	Message.LEN = 2;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0]=0x07;
    Message.DATA[1]=0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

//改变为Enable operation 状态
int tech_enable_operation(int slaveID)
{
	TPCANMsg Message;
	TPCANStatus Status;
	int nodeId = 0x04; //TPDO1 function code 0x04
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 2;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0]=0x0f;
    Message.DATA[1]=0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

//选择控制模式
int tech_set_control_mode(int slaveID, u32  mode)
{	
	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+2F 60 60 00 01 00 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0]=0x2f;
    Message.DATA[1]=0x60;
    Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = (int) mode;
	Message.DATA[5] = (int) (mode >> 8);
	Message.DATA[6] = (int) (mode >> 16);
	Message.DATA[7] = (int) (mode >> 24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

//发送目标位置
int tech_set_position(int slaveID, int plusNum)
{
	TPCANMsg Message;
	TPCANStatus Status;

	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x23;
	Message.DATA[1] = 0x7A;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = (int) plusNum;
	Message.DATA[5] = (int) (plusNum >> 8);
	Message.DATA[6] = (int) (plusNum >> 16);
	Message.DATA[7] = (int) (plusNum >> 24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
	return 0;
}

//发送目标速度
int tech_set_speed(int slaveID, int speed)
{
	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x23;
	Message.DATA[1] = 0x81;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = (int) speed;
	Message.DATA[5] = (int) (speed >> 8);
	Message.DATA[6] = (int) (speed >> 16);
	Message.DATA[7] = (int) (speed >> 24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

int tech_set_speed_mode(int slaveID, int speed)
{
	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x23;
	Message.DATA[1] = 0xFF;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = (int) speed;
	Message.DATA[5] = (int) (speed >> 8);
	Message.DATA[6] = (int) (speed >> 16);
	Message.DATA[7] = (int) (speed >> 24);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

int tech_set_acc(int slaveID,int acceleration)
{
	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x80;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = short(acceleration);
	Message.DATA[5] = short(acceleration>>8);
    Status=CAN_Write(PCAN_USBBUS1,&Message);
	ROS_INFO("tech_set_acc: id=%d acc=%d data[4]=%x data[5]=%x",slaveID,acceleration,Message.DATA[4],Message.DATA[5]);
    usleep(1000);
	return 0;
}

int tech_set_arr(int slaveID)
{
	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x80;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	// Message.DATA[4] = 0x20;
	// Message.DATA[5] = 0x03;
	Message.DATA[4] = 0x40;
	Message.DATA[5] = 0x06;

    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

int tech_set_arr_fast(int slaveID)
{
	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x80;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	// Message.DATA[4] = 0xBC;
	// Message.DATA[5] = 0x02;
	Message.DATA[4] = 0xBC;
	Message.DATA[5] = 0x02;

    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

//启动指令
int tech_start(int slaveID)
{

	TPCANMsg Message;
	TPCANStatus Status;
	//COB-ID:602h+2B 40 60 00 3F 00
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
    Message.ID = nodeId;
	Message.LEN = 6;
	Message.MSGTYPE = MSGTYPE_STANDARD; 
	Message.DATA[0] = 0x2B;
	Message.DATA[1] = 0x40;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x3F;
	Message.DATA[5] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
	return 0;
}

void tech_check_turn(int slaveID) 
{
    int nodeId = 0x0C; //RSDO function Code Îª 0x0C
    char hex[4] ;
    nodeId <<= 7;
    nodeId |= slaveID;
    Message.ID = nodeId;
    Message.LEN = 8;
    Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
    Message.DATA[0] = 0x40;
    Message.DATA[1] = 0x63;
    Message.DATA[2] = 0x60;
    Message.DATA[3] = 0x00;
    Message.DATA[4] = 0x00;
    Message.DATA[5] = 0x00;
    Message.DATA[6] = 0x00;
    Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
}

void tect_motor_init(void)
{
	tech_start_node(0x06);	
	tech_ready_switch_on(0x06);	
	tech_switch_on(0x06);	
	tech_enable_operation(0x06);
	tech_set_arr(0x06);
	tech_set_control_mode(0x06, 1);

	tech_start_node(0x05);
	tech_ready_switch_on(0x05);
	tech_switch_on(0x05);
	tech_enable_operation(0x05);
	tech_set_arr(0x05);
	tech_set_control_mode(5, 1);

	tech_start_node(0x07);
	tech_ready_switch_on(0x07);
	tech_switch_on(0x07);
	tech_enable_operation(0x07);
	tech_set_arr(0x07);
	tech_set_control_mode(0x07, 1);

	tech_start_node(0x08);
	tech_ready_switch_on(0x08);
	tech_switch_on(0x08);
	tech_enable_operation(0x08);
	tech_set_arr(0x08);
	tech_set_control_mode(0x08, 1);
}

void tect_motor_reinit(int slaveID)
{
	//tech_start_node();
	//tech_ready_switch_on(0x02);
	//tech_ready_switch_on(0x06);
	//tech_switch_on(0x02);
	//tech_switch_on(0x06);
	//tech_enable_operation(0x02);
	tech_enable_operation(slaveID);
	//tech_set_control_mode(2, 1);
	//tech_set_control_mode(6, 1);
}



void tech_turn(int slaveID, float angle)   //每次叠加跑一个角度
{	
	int plusNum = 7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] += plusNum;
	tech_set_position(slaveID, curPosition[index]-teckgetPosition[index]);
	// teckPosition[index] = curPosition[index]-teckgetPosition[index];
	tech_set_speed(slaveID, 0x200100);
	tech_start(slaveID);
}

void tech_turn_ex(int slaveID, float angle)   //只接跑到指定点:右转为正
{	
	int plusNum = 7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] += plusNum;
	tech_set_position(slaveID, plusNum-teckgetPosition[index]);
	tech_set_speed(slaveID, 0x200100);
	tech_start(slaveID);
}

void tech_turn_ex_slow(int slaveID, float angle)   //只接跑到指定点:右转为正
{	
	int plusNum = 7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] += plusNum;
	tech_set_position(slaveID, plusNum-teckgetPosition[index]);
	tech_set_speed(slaveID, 0x100100);
	tech_start(slaveID);
}

void tech_turn_right(int slaveID, float angle)   //每次叠加跑一个角度
{	
	int plusNum = 7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] += plusNum;
	tech_set_position(slaveID, curPosition[index]-teckgetPosition[index]);
	tech_set_speed(slaveID, 0x100100);
	tech_start(slaveID);
}

void tech_turn_left(int slaveID, float angle)
{ 
	int plusNum = -7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] -= plusNum;
	tech_set_position(slaveID, curPosition[index]-teckgetPosition[index]);
	tech_set_speed(slaveID, 0x100100);
	tech_start(slaveID);
}

void tech_turn_right_ex(int slaveID, float angle)   //只接跑到指定点
{	
	int plusNum = 7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] += plusNum;
	tech_set_position(slaveID, plusNum);
	tech_set_speed(slaveID, 0x100100);
	tech_start(slaveID);
}

void tech_turn_left_ex(int slaveID,  float angle)
{ 
	int plusNum = -7083.3*angle;
	int index = slaveID - 5;
	tect_motor_reinit(slaveID);
	curPosition[index] -= plusNum;
	//tech_set_position(slaveID, curPosition[index]);
	tech_set_position(slaveID, 0 - plusNum);
	tech_set_speed(slaveID, 0x100100);
	tech_start(slaveID);
}


/****************************************************
读取泰科电机的位置
****************************************************/
int AGV_position_Enquire(int slaveID)  
{
	//COB-ID:602h+23 7A 60 00 C3 50 00 00.
	int nodeId = 0x0C; //RSDO function Code Îª 0x0C
	nodeId <<= 7;
	nodeId |= slaveID;
	Message.ID = nodeId;
	Message.LEN = 8;
	Message.MSGTYPE = MSGTYPE_STANDARD;  //MSGTYPE_STANDARD   PCAN_MESSAGE_EXTENDED
	Message.DATA[0] = 0x40;
	Message.DATA[1] = 0x63;
	Message.DATA[2] = 0x60;
	Message.DATA[3] = 0x00;
	Message.DATA[4] = 0x00;
	Message.DATA[5] = 0x00;
	Message.DATA[6] = 0x00;
	Message.DATA[7] = 0x00;
    Status=CAN_Write(PCAN_USBBUS1,&Message);
    usleep(1000);
 	return 0;
}


void TK_position_Enquire(void)
{
	AGV_position_Enquire(5);
	AGV_position_Enquire(6);
	AGV_position_Enquire(7);
	AGV_position_Enquire(8);
}

  

