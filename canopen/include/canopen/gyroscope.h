#ifndef  _USART_H
#define  _USART_H
 
//串口相关的头文件    
#include<stdio.h>      /*标准输入输出定义*/    
#include<stdlib.h>     /*标准函数库定义*/    
#include<unistd.h>     /*Unix 标准函数定义*/    
#include<sys/types.h>     
#include<sys/stat.h>       
#include<fcntl.h>      /*文件控制定义*/    
#include<termios.h>    /*PPSIX 终端控制定义*/    
#include<errno.h>      /*错误号定义*/    
#include<string.h>    
     
     
//宏定义    
#define FALSE  -1    
#define TRUE   0
#define MAX_LENGTH_INSERT_SORT 2 /* 用于快速排序时判断是否选用插入排序阙值 */
#define MAXSIZE 1000  /* 用于要排序数组个数最大值，可根据需要修改 */

typedef struct
{
	float r[MAXSIZE+1];	/* 用于存储要排序数组，r[0]用作哨兵或临时变量 */
	int length;			/* 用于记录顺序表的长度 */
}SqList;

typedef struct QNode	/* 结点结构 */
{
   float data;
   struct QNode *next;
}QNode,*QueuePtr;

typedef struct			/* 队列的链表结构 */
{
   QueuePtr front,rear; /* 队头、队尾指针 */
}LinkQueue;

/*小车里程结构体*/
typedef struct 
{
	float FlWheel_mileage; //左前轮里程
	float FrWheel_mileage; 
	float RlWheel_mileage;
	float RrWheel_mileage;
	float car_mileage; //小车平均里程
	float car_tmp_mileage; //小车阶段性里程 
	float car_offset_x; //小车平均里程
	float car_tmp_offset_x; //小车阶段性里程 
	float offset_x; //X 方向上的偏移
	float offset_y; //Y 方向上的偏移
	float offset_theta; //角度偏移量
}car_mileage_struct; 

typedef struct{
	float s1; //speed1
	float s2;
	float s3;
	float s4;
	float a1; //theta1
	float a2;
	float a3;
	float a4;
}wheel_data_struct;

/*小车角度记录结构体*/
typedef struct{
	float GyrNow_angle; //陀螺仪当前输出角度
	float GyrLast_angle; //陀螺仪上一次输出角度,360
	float QrNow_angle; //当前扫到QR码时陀螺仪的角度
	float QrLast_angle; //扫到上一个QR码时陀螺仪的角度运动状态(指令)
	float last_CodeNum; //上一个二维码编号
	float now_CodeCmd; //当前二维码运动状态（指令）
	float now_CodeNum; //当前二维码编号
	float GyrQR_angle; //二维码纠正过后的陀螺仪角度
	float theta1_bjf_QR; //小车相对于QR码的夹角
	float theta2_GY; //陀螺实时输出的角度
	float theta3_GY_deviation; //陀螺仪相对于码的误差
	float theta4_real_angle; //小车当前与二维码之间的偏移角
	float car_angle; //小车偏移角度
}car_angle_struct;

extern LinkQueue center_speed;
int UART0_Open(int fd,char*port);
void UART0_Close(int fd) ; 
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity) ;
int UART0_Recv(int fd, char *rcv_buf,int data_len);
int UART0_Send(int fd, char *send_buf,int data_len);
void* send_ask_speed(void* A);

void gyroscope_cor_test(void);
extern void odometer_stop(void);
extern int odometer_start(void);
int car_walk_correction(int cmd_mode);
extern int gyroscope_fd;
extern int odometer_flag;
extern float get_gyroscope_angle(void);
extern void gyroscope_init(void);
extern void* gyroscope_pending(void*); 
extern float gyroscope_angle;
extern wheel_data_struct wheel_data;
#endif

