#include "canopen/type.h"
#include "string.h"
#include "canopen/sensor_cmc.h"
#include "canopen/gyroscope.h"
#include "canopen/auto_lift.h"
#include "canopen/sport_control.h"
#include "canopen/motorCAN.h"
#include "canopen/MOTEC_motorCtl.h"
#include "canopen/PCANBasic.h"
#include "ros/ros.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <canopen/joy_callback.h>
#include <stdio.h>


#define CAN_NODE 0x58f

#define CmdLaser_CheckWheel_1 0x01 //第一次检测到轮胎边沿
#define CmdLaser_CheckWheel_2 0x02 //第二次检测到轮胎边沿
#define CmdLaser_CheckWheel_3 0x03 //第三次检测到轮胎边沿

#define Laser_CarLen_0  225  //车身传感器初值



TPCANMsg Message_cmc;
TPCANStatus Status_cmc;


//数据类型转字节数数组
const u8 CMC_DATAT_BYTECOUNT[] = {1,1,2,2,4,4,4};

//寄存器数据对象
short current_1;//电流
u16 vop_1;//电压
u16 soc_1;//电量
float hdistance1_1;//1号超声波距离
float hdistance2_1;//2号超声波距离
float gdistance1_1;//1号激光测距距离
float gdistance2_1;//2号激光测距距离

short current_2;//电流
u16 vop_2;//电压
u16 soc_2;//电量
float hdistance1_2;//1号超声波距离
float hdistance2_2;//2号超声波距离
float gdistance1_2;//1号激光测距距离
float gdistance2_2;//2号激光测距距离

u32 Get_data;
float Get_Speed1;			//1号电机运行速度
float Get_Speed2;			//2号电机运行速度
float Get_Speed3;			//3号电机运行速度
float Get_Speed4;			//4号电机运行速度

u32 Get_Angle9;		//9号电机举升的角度
u32 Get_Angle10;		//10号电机举升的角度
u32 Get_Angle11;		//11号电机举升的角度
u32 Get_Angle12;		//12号电机举升的角度
u32 TxMailbox;

u32 canRxFlag = 0;
int canSlaveBoardRxFlag = 1;//接收到从机消息的标志

float WheelSpeed_RecBuff1[1024] = {0.0}; //左前方驱动电机回传的速度 
u16  WheelSpeed_RecCount = 0;// 左前方驱动器电机回传速度计数值 
s16 laser_car_len = 0; //传感器测量到的车身长度
int log_buff[8]={0}; //电机电流数据
float speed_buff[8] = {0.0}; //电机速度

//0x0e节点寄存器列表对象
CMComReg cmcreg_arr1[]=
{
	{CMC_DATAT_S16,&current_1},
	{CMC_DATAT_U16,&vop_1},
	{CMC_DATAT_U16,&soc_1},//电量
	{CMC_DATAT_FLOAT_IEE32,&hdistance1_1},
	{CMC_DATAT_FLOAT_IEE32,&hdistance2_1},
	{CMC_DATAT_FLOAT_IEE32,&gdistance1_1},
	{CMC_DATAT_FLOAT_IEE32,&gdistance2_1}	
};

//0x0f节点板寄存器列表对象
CMComReg cmcreg_arr2[]=
{
	{CMC_DATAT_S16,&current_2},
	{CMC_DATAT_U16,&vop_2},
	{CMC_DATAT_U16,&soc_2},//电量
	{CMC_DATAT_FLOAT_IEE32,&hdistance1_2},
	{CMC_DATAT_FLOAT_IEE32,&hdistance2_2},
	{CMC_DATAT_FLOAT_IEE32,&gdistance1_2},
	{CMC_DATAT_FLOAT_IEE32,&gdistance2_2}	
};


//从板对象数组
SlaveBoardComObj slaveboard[]=
{
	{0x0f,cmcreg_arr1}, //左前方小板
	{0x10,cmcreg_arr2}, // 右前方小板
	{0x11,cmcreg_arr2}, // 左后方小板
	{0x12,cmcreg_arr2} // 右后方小板
};


/*功能数组*/
CMCom_FunS cmc_fun_array[]=
{
	{0x04,CMC_MaterFun04H},
	{0x05,CMC_MaterFun05H}
};


//04功能码回应解析
static int CMC_MaterFun04H(CMComReg *regArray)
{
	int i;
	u8 regAddr;
	u8 dataCount;
	u8 response[8];
	u8 offset;
	
  //缓冲接收
	for(i=0;i<8;i++)
	{
		response[i] = Message_cmc.DATA[i];
	}
	
	//从接受中获取
	regAddr = response[1];
	
  //计算单个寄存器对象的size	
	dataCount = CMC_DATAT_BYTECOUNT[regArray[regAddr].dataType]; //通过 cmcreg_arr1 ---> CMC_DATAT
	offset = response[2];
	
	//小端序
	memcpy((u8*)((u8*)(regArray[regAddr].p)+offset*dataCount),&response[3],dataCount);
	return 0;
}

//05功能码回应解析
//static u8 wheel_count = 0;
#if 1  // 曲线行走模式
static int CMC_MaterFun05H(CMComReg *regArray)
{
	int i;
//	u8 regAddr;
//	u8 dataCount;
	u8 response[8];
//	u8 offset;
	u16 distence = 0;
	
	memset(response,0xff,sizeof(response));
  //缓冲接收
	for(i=0;i<8;i++)
	{
		response[i] = Message_cmc.DATA[i];
	} 
	switch(response[2])
		{
			case 0x55: //从轮胎出来
				if(!timer_mode) //车底测距
					{
						wheel_time_buff[response[1]-0x0f].t0_count++;
						ROS_INFO("wheel out");
						switch(wheel_time_buff[response[1]-0x0f].t0_count)
							{
								case 0x01:
									//wheel_time_buff[response[1]-0x0f].t2 = timer_count;   
									//ROS_INFO("back wheel out");
									Cmd_Laser = CmdLaser_CheckWheel_2;
									distence = response[3];
									distence = distence<<8;
									wheel_time_buff[response[1]-0x0f].laser_CarWheel_distance_F = distence + response[4];
									break;
								case 0x02:
									//wheel_time_buff[response[1]-0x0f].t4 = timer_count; 
									//ROS_INFO("front wheel out");
									distence = response[3];
									distence = distence<<8;
									wheel_time_buff[response[1]-0x0f].laser_CarWheel_distance_R= distence + response[4];
									break;
								default:
									break;
							}
					}
				else //轮前姿态矫正
					{
						;
					}   
				break;
			case 0xaa: //检测到轮胎
				if(!timer_mode) //车底测距
					{   
						wheel_time_buff[response[1]-0x0f].t1_count++;
						ROS_INFO("wheel in");
						switch(wheel_time_buff[response[1]-0x0f].t1_count)
							{
								case 0x01:
									//wheel_time_buff[response[1]-0x0f].t1 = timer_count;
									//ROS_INFO("back wheel in");
									Cmd_Laser = CmdLaser_CheckWheel_1;
									break; 
								case 0x02:
									//wheel_time_buff[response[1]-0x0f].t3 = timer_count;
									//ROS_INFO("front wheel in");
									Cmd_Laser = CmdLaser_CheckWheel_3;
									break;
								default:
									break;
							}
					}
				else //轮前姿态矫正
					{
						if(response[1]-0x0f == 0x00)
							{
								Set_right_time.L_time = timer_count;//Lf
							}
						if(response[1]-0x0f == 0x01)
							{
								Set_right_time.R_time = timer_count;//Rf
							}
					}
				break;
			case 0x33: //检测到轮胎
				laser_car_len =  response[3];
				laser_car_len = (laser_car_len << 8) + response[4];
				laser_car_len -= Laser_CarLen_0;
				if(laser_car_len <= 0)
					laser_car_len = 0;
				laser_car_len *= 2;	
				//laser_car_len += 25;
				//ROS_INFO("Interupt >>>>>>>>>>>> laser_car_len = %d",laser_car_len); 
				break;
			default:
				break;
		}
	//ROS_INFO("Cmd_Laser = %d \r\n",Cmd_Laser);
	return 0;
}

#else  //匀速直线运动模式
static int CMC_MaterFun05H(CMComReg *regArray)
{
	int i;
//	u8 regAddr;
//	u8 dataCount;
	u8 response[8];
//	u8 offset;
	u16 distence = 0;
	
  //缓冲接收
	for(i=0;i<8;i++)
	{
		response[i] = Message_cmc.DATA[i];
	} 
	switch(response[2])
		{
			case 0x55: //从轮胎出来
				if(!timer_mode) //车底测距
					{
						wheel_time_buff[response[1]-0x0f].t0_count++;
						ROS_INFO("%d 轮胎出来\r\n",response[1]);
						switch(wheel_time_buff[response[1]-0x0f].t0_count)
							{
								case 0x01:
									wheel_time_buff[response[1]-0x0f].t2 = timer_count;
									distence = response[3];
									distence = distence<<8;
									wheel_time_buff[response[1]-0x0f].laser_CarWheel_distance_F = distence + response[4];
									break;
								case 0x02:
									wheel_time_buff[response[1]-0x0f].t4 = timer_count;
									distence = response[3];
									distence = distence<<8;
									wheel_time_buff[response[1]-0x0f].laser_CarWheel_distance_R= distence + response[4];
									break;
								default:
									break;
							}
					}
				else //轮前姿态矫正
					{
						;
					}
				break;
			case 0xaa: //检测到轮胎
				if(!timer_mode) //车底测距
					{
						wheel_time_buff[response[1]-0x0f].t1_count++;
						ROS_INFO("%d 轮胎进入\r\n",response[1]);
						switch(wheel_time_buff[response[1]-0x0f].t1_count)
							{
								case 0x01:
									wheel_time_buff[response[1]-0x0f].t1 = timer_count;
									break;
								case 0x02:
									wheel_time_buff[response[1]-0x0f].t3 = timer_count;
									break;
								default:
									break;
							}
					}
				else //轮前姿态矫正
					{
						if(response[1]-0x0f == 0x00)
							{
								Set_right_time.L_time = timer_count;//Lf
							}
						if(response[1]-0x0f == 0x01)
							{
								Set_right_time.R_time = timer_count;//Rf
							}
					}
				break;
			default:
				break;
		}
	return 0;
}
#endif


int CMC_slaveBoardMsgPaser(void)
{
	int slaveId,i;
	CMComReg *regArray;
	u8 fnctionCode = Message_cmc.DATA[0];
	
	//查找对应的板
	slaveId = Message_cmc.ID;
	for(i=0;i<sizeof(slaveboard)/sizeof(slaveboard[0]);i++)
	{
		if(slaveboard[i].boardId == slaveId)
		{
			regArray = slaveboard[i].regArray;//获取寄存器列表
			break;
		}
	}
	if(i == sizeof(slaveboard)/sizeof(slaveboard[0]))return 1;//查找失败
	
	//目前只开发04功能,所以可以直接调用
	//CMC_MaterFun04H(regArray);
	for(i=0;i<sizeof(cmc_fun_array)/sizeof(cmc_fun_array[0]);i++) 
	{
		if(cmc_fun_array[i].functionCode == fnctionCode)
		{
			cmc_fun_array[i].function(regArray);
			/*激光测距程序在此处理，其他放入循环助理*/
			break;
		}
	}
	
	if(i == sizeof(cmc_fun_array)/sizeof(cmc_fun_array[0]))return 1;//查找功能函数失败
	return 0;
}


/*
* 组装04命令帧
*/
void CMC_requestSlaveData_04(u8 id,u8 regaddr,u8 regType)
{
	Message.ID = 0x600+id;//为了与canopen兼容,从机id要加上0x600
	Message.MSGTYPE = MSGTYPE_STANDARD;
	//0	       1	       2	        3	      4
  //命令字	逻辑地址	读取数据类型	偏移地址	寄存器数量
	Message.LEN = 3;	
	Message.DATA[0] = 0x04;//命令字
	Message.DATA[1] = regaddr;//逻辑地址
	Message.DATA[2] = 0x00;//偏移地址
	//Message.DATA[3] = 0x01;//寄存器数量
	
}




//接收到从板消息标志
extern int canSlaveBoardRxFlag;


/*
* 组装05命令帧
*/
void CMC_SendData_05(u8 id,u8 regaddr,u8 regType)
{
	Message.ID = 0x600+id;//为了与canopen兼容,从机id要加上0x600
	Message.MSGTYPE = MSGTYPE_STANDARD;
	//0	       1	       2	        3	      4
  //命令字	逻辑地址	读取数据类型	偏移地址	寄存器数量
	Message.LEN = 2;
	Message.DATA[0] = regaddr;//命令字
	Message.DATA[1] = regType;
	Status=CAN_Write(PCAN_USBBUS1,&Message);
	usleep(1000);
	//send_to_NodeBoard(id,Message.DATA,Message.LEN);
}

/*
* 向节点板发请求
* 返回值同CMC_sendRequest函数
name	:
*/
/*******************************************************************
name	:CMC_grabSlaveBoardData
funtion :向节点板发请求,返回值同CMC_sendRequest函数
input	:id 节点板，不加SDO命令字的ID
		:regaddr 对应从机的寄存器在寄存器列表的的逻辑地址
		:regType 对应寄存器的数据类型,详见主机cmc.h中的定义
		:wait 拓展时用,一直为零
NOTICE	:因为从机的缓冲区长度为1,所以不宜频繁的调用该函数
output	:成功返回0，失败返回1
*******************************************************************/
int CMC_grabSlaveBoardData(u8 id,u8 regaddr,u8 regType,int wait)
{
	CMC_requestSlaveData_04(id,regaddr,regType);
	return 0;
}

//判断是否是节点板发来的消息
int isSlaveBoardMsg(TPCANMsg *header)
{
	u32 id=0;
	id = header->ID ;
	id -= 0x580;

	if(id >= 0x0d && id <=0xf)
	{
		return 1;
	}
	return 0;
}

/****************************************************
* 程序名称：电机位置处理函数
* 程序功能：读取得到电机位置数据
* 作者/修改者：许定哲
****************************************************/
u32 AGV_position_Get(void)
{
	u8 position_buff[8];
	u32 position_temp;
	
	position_buff[4]=Message_cmc.DATA[4];
	position_buff[5]=Message_cmc.DATA[5];
	position_buff[6]=Message_cmc.DATA[6];
	position_buff[7]=Message_cmc.DATA[7];
	
	position_temp=position_buff[4];
	position_temp+=position_buff[5]<<8;
	position_temp+=position_buff[6]<<16;
	position_temp+=position_buff[7]<<24;
	
	return	position_temp;
}

/****************************************************
* 程序名称：电机速度处理函数
* 程序功能：读取得到电机速度数据
* 作者/修改者：许定哲
****************************************************/
short AGV_Speed_Get(void)
{
	u8 Speed_buff[8]={0};
	short Speed_temp;
	
	Speed_buff[4]=Message_cmc.DATA[4];
	Speed_buff[5]=Message_cmc.DATA[5];
	Speed_buff[6]=Message_cmc.DATA[6];
	Speed_buff[7]=Message_cmc.DATA[7];

	Speed_temp=Speed_buff[4];
	Speed_temp+=Speed_buff[5]<<8;
	Speed_temp+=Speed_buff[6]<<16;
	Speed_temp+=Speed_buff[7]<<24;
	
	return	Speed_temp;
}


int AGV_Speed_TK(void)
{
	u8 Speed_buff[8]={0};
	u32 Speed_temp = 0;
	int res_speed = 0;
	
	Speed_buff[4]=Message_cmc.DATA[4];
	Speed_buff[5]=Message_cmc.DATA[5];
	Speed_buff[6]=Message_cmc.DATA[6];
	Speed_buff[7]=Message_cmc.DATA[7];

	Speed_temp=Speed_buff[6];
	Speed_temp=(Speed_temp << 8) + Speed_buff[5];
	Speed_temp=(Speed_temp << 8) + Speed_buff[4];
	//Speed_temp+=Speed_buff[7]<<24;
	
	if(Speed_buff[7] == 0xff)
		{
			res_speed = (int)((~(Message_cmc.DATA[4]+(Message_cmc.DATA[5]<<8)+
				(Message_cmc.DATA[6]<<16)+(Message_cmc.DATA[7]<<24)))+1)*(-1);
		}
	else
		{
			res_speed = Speed_temp;
		}
	return	res_speed;
}


void AGV_log_speed(void)
{
	char tmp[]="motorCurrent[0]\tmotorCurrent[1]\tmotorCurrent[2]\tmotorCurrent[3]\tmotorCurrent[4]\tmotorCurrent[5]\tmotorCurrent[6]\tmotorCurrent[7]\n";
	
	int log_fp;
	int temp_speed = 0;
	char i = 0;
	
	
	if((int)Message_cmc.ID >= 0x581  &&   (int)Message_cmc.ID <= 0x588 && (Message_cmc.DATA[1]==0x6c))
		{
			#if 0 
				{
					ROS_INFO("%d--Receive( ID=%x ):\nDATA[0] = %x\tDATA[1] = %x\tDATA[2] = %x\tDATA[3] = %x\nDATA[4] = %x\tDATA[5] = %x\tDATA[6] = %x\tDATA[7] = %x\n",
						Status_cmc,(int)Message_cmc.ID,Message_cmc.DATA[0],Message_cmc.DATA[1],Message_cmc.DATA[2],
					 	Message_cmc.DATA[3],Message_cmc.DATA[4],Message_cmc.DATA[5],Message_cmc.DATA[6],Message_cmc.DATA[7]);
				}
			#endif
			//if(Message_cmc.DATA[1]==0x6c)
			{
				if((int)Message_cmc.ID== 0x0581&&AGV_position_Get()>0)
					{
						speed_buff[0]= ((float)AGV_Speed_Get()/2013);
					}
				else if((int)Message_cmc.ID== 0x0582&&AGV_position_Get()>0)
					{
						speed_buff[1]=((float)AGV_Speed_Get()/2013);
					}
				else if((int)Message_cmc.ID== 0x0583&&AGV_position_Get()>0)
					{
						speed_buff[2]=((float)AGV_Speed_Get()/2013);
					}
				else if((int)Message_cmc.ID== 0x0584&&AGV_position_Get()>0)  
					{
						speed_buff[3]=((float)AGV_Speed_Get()/2013);  
					}
				else if((int)Message_cmc.ID== 0x0585&&AGV_position_Get()>0)  
					{
						speed_buff[4]=(AGV_Speed_TK()*1.41176/100000);  
					}
				else if((int)Message_cmc.ID== 0x0586&&AGV_position_Get()>0)  
					{
						speed_buff[5]=(AGV_Speed_TK()*1.41176/100000);  
					}
				else if((int)Message_cmc.ID== 0x0587&&AGV_position_Get()>0)  
					{
						speed_buff[6]=(AGV_Speed_TK()*1.41176/100000);  
					}
				else if((int)Message_cmc.ID== 0x0588&&AGV_position_Get()>0)  
					{
						speed_buff[7]=(AGV_Speed_TK()*1.41176/100000);  
					}
				//if(Message_cmc.ID-0x581 == 0)
				//	printf("\n");
				//printf("%6f  ",Message_cmc.ID,speed_buff[Message_cmc.ID-0x581]);
				if(((int)Message_cmc.ID-0x581) == 7)
				{
					if((log_fp=open("./speed12345678.txt",O_RDWR|O_CREAT|O_APPEND))<0)
					{
						ROS_INFO("ERROR:Can not open current.txt!");
					}
					int num=sprintf(tmp,"%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\n",speed_buff[0],speed_buff[1],speed_buff[2],speed_buff[3],speed_buff[4],speed_buff[5],speed_buff[6],speed_buff[7]);
					// ROS_INFO("%s",tmp);
					write(log_fp,tmp,num);
					close(log_fp);
				}
			}		 
		}
	//memset(&Message_cmc,0,sizeof(TPCANMsg));
}

void AGV_log_Current(void)
{
	char tmp[]="motorCurrent[0]\tmotorCurrent[1]\tmotorCurrent[2]\tmotorCurrent[3]\tmotorCurrent[4]\tmotorCurrent[5]\tmotorCurrent[6]\tmotorCurrent[7]\n";
	
	int log_fp;	
	//printf("运行电流数据读取函数 ID = %X\n",Message_cmc.ID);
	if((int)Message_cmc.ID >= 0x581  &&   (int)Message_cmc.ID <= 0x588)
		{
			//if((int)Message_cmc.ID >= 0x585  &&   (int)Message_cmc.ID <= 0x588)
			#if 0
				{
					ROS_INFO("%d--Receive( ID=%x ):\nDATA[0] = %x\tDATA[1] = %x\tDATA[2] = %x\tDATA[3] = %x\nDATA[4] = %x\tDATA[5] = %x\tDATA[6] = %x\tDATA[7] = %x\n",
						Status_cmc,(int)Message_cmc.ID,Message_cmc.DATA[0],Message_cmc.DATA[1],Message_cmc.DATA[2],
					 	Message_cmc.DATA[3],Message_cmc.DATA[4],Message_cmc.DATA[5],Message_cmc.DATA[6],Message_cmc.DATA[7]);
				}
			#endif
			if(Message_cmc.DATA[0]==0x4b && ((Message_cmc.DATA[1]==0x78)||(Message_cmc.DATA[1]==0x1c)))
			{
				log_buff[(int)Message_cmc.ID-0x581]=(int)Message_cmc.DATA[4]+(int)(Message_cmc.DATA[5]<<8);
				#if 1
				if(Message_cmc.DATA[5] >>7 == 1)
				{
					log_buff[(int)Message_cmc.ID-0x581]=(int)((~((short)Message_cmc.DATA[4]+(short)(Message_cmc.DATA[5]<<8)))+1)*(-1);
				}
				#endif
				if(((int)Message_cmc.ID-0x581) == 7) 
				{
					if((log_fp=open("./current12345678.txt",O_RDWR|O_CREAT|O_APPEND))<0)
					{
						ROS_INFO("ERROR:Can not open current.txt!");
					}
					log_buff[4] = log_buff[4]*10;
					log_buff[5] = log_buff[5]*10;
					log_buff[6] = log_buff[6]*10;
					log_buff[7] = log_buff[7]*10;
					int num=sprintf(tmp,"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",log_buff[0],log_buff[1],log_buff[2],log_buff[3],log_buff[4],log_buff[5],log_buff[6],log_buff[7]);
					//ROS_INFO("%s",tmp);
					write(log_fp,tmp,num);
					close(log_fp);
				}
			}		 
		}
}

float TK_position_Get(void)
{
	 unsigned char position_buff[8];
	  int position_temp;
	 float angle= 0.0;

	if(((int)Message_cmc.ID >= 0x585  && (int)Message_cmc.ID <= 0x588)&&(Message_cmc.DATA[1] == 0x63))
		{
			 position_buff[4]=Message_cmc.DATA[4];
			 position_buff[5]=Message_cmc.DATA[5];
			 position_buff[6]=Message_cmc.DATA[6];
			 position_buff[7]=Message_cmc.DATA[7];
			 
			 position_temp=Message_cmc.DATA[4];
			 position_temp+=Message_cmc.DATA[5]<<8;
			 position_temp+=Message_cmc.DATA[6]<<16;
			 position_temp+=Message_cmc.DATA[7]<<24;
			if(Message_cmc.DATA[7] == 0xff)
				{
					position_temp = (int)((~(Message_cmc.DATA[4]+(Message_cmc.DATA[5]<<8)+
						(Message_cmc.DATA[6]<<16)+(Message_cmc.DATA[7]<<24)))+1)*(-1);
				}
			angle = position_temp/7083.3;
			switch(Message_cmc.ID - 0x585)
				{
					case 0:
						wheel_data.a1 = angle;	
					break;
					case 1:
						wheel_data.a2 = angle;
					break;
					case 2:
						wheel_data.a3 = angle;
					break;
					case 3:
						wheel_data.a4 = angle;
					break;
					default:
						break;
				}
			//printf("%x#转向盘当前角度:%f\n",(int)Message_cmc.ID,angle);
		}
	 return angle;
}

void *CAN_pending(void *)
{
	while(1)
	{
		memset(&Message_cmc,0,sizeof(TPCANMsg));
		Status_cmc=CAN_Read(PCAN_USBBUS1,&Message_cmc,NULL);
		if((int)Message_cmc.ID!=0)     
		{
			AGV_log_speed();
			AGV_log_Current();
			TK_position_Get();
			canRxFlag = 1;
			if(isSlaveBoardMsg(&Message_cmc))
			{
				Message_cmc.ID -= 0x580;
				canSlaveBoardRxFlag = 1;//节点板发来的消息
				//解析消息
				CMC_slaveBoardMsgPaser();
			}
			if(Message_cmc.ID<=0x058c && Message_cmc.ID>=0x0581)//查询motec电机的速度或者位置
			{	
				//举升电机角度
					if((int)Message_cmc.ID == 0x0589 && AGV_position_Get()>0)
							Get_Angle9 = (0xFFFFFFFF - AGV_position_Get())/21111; 
					else if((int)Message_cmc.ID == 0x058A&&AGV_position_Get()>0)
							Get_Angle10 = (0xFFFFFFFF- AGV_position_Get())/21111;
					else if((int)Message_cmc.ID == 0x058B&&AGV_position_Get()>0)
							Get_Angle11 = (0xFFFFFFFF - AGV_position_Get())/21111;	
					else if((int)Message_cmc.ID== 0x058C&&AGV_position_Get()>0)  
							Get_Angle12 = (0xFFFFFFFF - AGV_position_Get())/21111;				
				//驱动电机速度	
					if((Message_cmc.DATA[1]==0x6c)&&(int)Message_cmc.ID== 0x0581&&AGV_position_Get()>0)
						{
							Get_Speed1= ((float)AGV_Speed_Get()/2013);
							FB_speed_flag = 1 ; 
							//printf("1#电机速度：%f\n",Get_Speed1);
						}
					else if((Message_cmc.DATA[1]==0x6c)&&(int)Message_cmc.ID== 0x0582&&AGV_position_Get()>0)
						{
							Get_Speed2=((float)AGV_Speed_Get()/2013);
							FB_speed_flag = 1 ; 
							//printf("2#电机速度：%f\n",Get_Speed2);
						}
					else if((Message_cmc.DATA[1]==0x6c)&&(int)Message_cmc.ID== 0x0583&&AGV_position_Get()>0)
						{
							Get_Speed3=((float)AGV_Speed_Get()/2013);
							FB_speed_flag = 1 ; 
							//printf("3#电机速度：%f\n",Get_Speed3);
						}
					else if((Message_cmc.DATA[1]==0x6c)&&(int)Message_cmc.ID== 0x0584&&AGV_position_Get()>0)  
						{
							Get_Speed4=((float)AGV_Speed_Get()/2013);  
							FB_speed_flag = 1 ; 
							//printf("4#电机速度：%f\n",Get_Speed4);
						}
				//ROS_INFO("speed1:%f speed2:%f speed3:%f speed4:%f",Get_Speed1,Get_Speed2,Get_Speed3,Get_Speed4);
				Get_data=AGV_position_Get();//查询得到的位置数据，也可以得到速度数据
				
			}
			else
				canRxFlag = 1;
		}
	}
    
}
