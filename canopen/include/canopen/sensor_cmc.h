#ifndef ___SENSORCMC_H___
#define ___SENSORCMC_H___

#include "canopen/PCANBasic.h"
#include "canopen/type.h"


typedef enum
{
	CMC_DATAT_U8=0x00,
	CMC_DATAT_S8=0x01,
	CMC_DATAT_U16=0x02,
	CMC_DATAT_S16=0x03,
	CMC_DATAT_U32=0x04,
	CMC_DATAT_S32=0x05,
	CMC_DATAT_FLOAT_IEE32=0x06,
}CMC_DATAT;

//寄存器列表数组
typedef struct 
{
	CMC_DATAT dataType;
	void *p;
}CMComReg;

typedef int (CMC_Slave_Fun)(CMComReg*);


typedef struct 
{
	u8 functionCode;
	CMC_Slave_Fun *function;
}CMCom_FunS;


//从板对象结构体
typedef struct
{
	u8 boardId;//从板ID(canopen的id,COBID减去SDO命令码之后的的ID)
	CMComReg *regArray;//寄存器列表对象指针
}SlaveBoardComObj;


extern float Get_Speed1;			//1号电机运行速度
extern float Get_Speed2;			//2号电机运行速度
extern float Get_Speed3;			//3号电机运行速度
extern float Get_Speed4;			//4号电机运行速度

extern u32 Get_Angle9;		//9号电机举升的角度
extern u32 Get_Angle10;		//10号电机举升的角度
extern u32 Get_Angle11;		//11号电机举升的角度
extern u32 Get_Angle12;		//12号电机举升的角度

extern s16 laser_car_len; //传感器测量到的车身长度
//extern int motorCurrent[4];

extern int log_buff[8]; //电机电流数据
extern float speed_buff[8]; //电机速度

int CMC_grabSlaveBoardData(u8 id,u8 regaddr,u8 regType,int wait);

int CMC_slaveBoardMsgPaser(void);

static int CMC_MaterFun04H(CMComReg *regArray);

static int CMC_MaterFun05H(CMComReg *regArray);

extern void CMC_SendData_05(u8 id,u8 regaddr,u8 regType);

int isSlaveBoardMsg(TPCANMsg *header);

u32 AGV_position_Get(void);

short AGV_Speed_Get(void);

void laser_open_close(u8 cmd);

void *CAN_pending(void *);

#endif