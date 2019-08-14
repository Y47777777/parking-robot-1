

/*******************************************************************************
file naime	:auto_lift.c	
function	:实现自动夹车
author		:ZWL
date		:20180809
*******************************************************************************/
#include "ros/ros.h"
#include <string.h>
#include <math.h>
#include "canopen/sport_control.h"
#include "canopen/auto_lift.h"
#include "canopen/car_driver.h"
#include "canopen/sensor_cmc.h"
#include "canopen/MOTEC_motorCtl.h"
#include "canopen/type.h"

#define moto_select 4
   
#define O_KEY_BIT					0x0800UL
#define R1_KEY_BIT					0x1000UL

#define CAR_SPEED_Wheel 180 			//车底车速 mm/s    
#define get_WheelStatus_delay 100 	//获取轮子运动状态和速度延迟时间
#define PO_mode_speed 0.1 			//小车在位置模式运动时的速度: 0.1m/s  =   0.1mm/ms 


ST_Rob_DimensionPara Rob_DimensionPara; //机器人尺寸参数 
ST_Rob_DimensionPara_float Rob_DimensionPara_float; //机器人尺寸参数 
volatile u32 timer_count=0;
volatile u32 can_tim6_count=0;
u8 car_out_flag = 0; //小车车底测量结束 出来标志 1 测量中， 0正在测量中
ST_OneTrigerTime wheel_time_buff[4] = {0}; //LF RF LR RR;
ST_OneTrigerTime Laser_signal; //激光传感器信号，用于非匀速曲线模式
u8 timer_mode = 0; //定时器工作模式: 0x00--> 车底测距模式  0x01--> 轮前矫正模式
ST_Set_right_time Set_right_time; //姿态矫正计时器
ST_PID_arg PID; //PID参数
u16 car_len = 0; //车身长度
u16 car_len_buff[3] = {0};
u8 car_len_count = 0;  
/*
tim6_mode[0]: 0 laser 关； 1 laser开
tim6_mode[1]: 0 can 关；  1 can 开
*/
u8 tim6_mode[2] = {0};
u8 tim6_work_mode = 0;//同上
u8 FB_speed_flag = 0; //得到速度反馈为1，否则为0
u16 set_WheelSpeedBuff_count = 0;
u32 send_speed_count = 0; //已经发送的速度值  


extern u8 aRxBuffer_u3;			//接收中断缓冲

    
/*******************************************************************
name	:err_code_process
funtion	:错误码处理
input	:err_code 错误码
output	:none
*******************************************************************/
void err_code_process(u8 err_code)
{
	switch(err_code)
		{
			case CAR_LEN_ERR:
			break;

			default:
			break;
		}
}




/*******************************************************************
name	:car_move_x
funtion	:小车在X轴方向横向水平移动
input	:flag_x 0 x轴正向移动， 1 负向移动
		:distance 移动的距离
output	:成功返回0 失败返回1
*******************************************************************/
u8 car_move_x(u8 flag_x, u32 distance)
{
	/*四个转向轮顺时针旋转90度*/
	wheel_turn(0,90);
	if(flag_x == 0) 
		{
			car_GoAhead(PO_MODE, distance);
		}
	else if(flag_x == 1)
		{
			car_GoBack(PO_MODE,distance);
		}
	else
		{
			wheel_turn(1,90);
			return 1;
		}
	wheel_turn(1,90);
	return 0;
}


/*******************************************************************
name	:wheel_data_process
funtion	:车轮数据处理
input	:triger_time 触发时间
		:wheel_data 车轮数据
output	:车辆信息合法返回0，非法返回1
*******************************************************************/
static u8 wheel_data_process(ST_triger_time triger_time, ST_wheel_data* wheel_data)
{
	u32 car_speed = 0; 
	ST_wheel_data car_wheel_data[2] = {0};
	u32 temp1 = 0; //中间计算结果
	u32 temp2 = 0; 
	u32 temp3 = 0; 
	u32 temp4 = 0; 
	u32 dlt_21 = 0;
	u32 dlt_43 = 0;
	u32 dlt_32 = 0;
	//double temp5 = 0.0;

	car_speed = CAR_SPEED_Wheel; //get_car_speed();  mm/s
	car_wheel_data[0].front_wheel_len = (triger_time.FL.t4 - triger_time.FL.t3)*car_speed/1000;
	car_wheel_data[0].rear_wheel_len = (triger_time.FL.t2 - triger_time.FL.t1)*car_speed/1000;
	car_wheel_data[0].car_len = (triger_time.FL.t3 - triger_time.FL.t2)*car_speed/1000 + (car_wheel_data[0].front_wheel_len + car_wheel_data[0].rear_wheel_len)/2;
      //car_speed = get_car_speed();
	
	car_wheel_data[1].front_wheel_len = ((triger_time.FR.t4 - triger_time.FR.t3)*car_speed)/1000;
	car_wheel_data[1].rear_wheel_len = ((triger_time.FR.t2 - triger_time.FR.t1)*car_speed)/1000;
	car_wheel_data[1].car_len = ((triger_time.FR.t3 - triger_time.FR.t2)*car_speed)/1000 +  (car_wheel_data[1].front_wheel_len + car_wheel_data[1].rear_wheel_len)/2;
	dlt_21 = (triger_time.FL.t2 - triger_time.FL.t1);
	dlt_43 = (triger_time.FL.t4 - triger_time.FL.t3);  // ms  
	dlt_32 = (triger_time.FL.t3 - triger_time.FL.t2); 
	ROS_INFO("dlt_21 = %d  ,dlt_43 = %d  ,dlt_32 = %d  \n",dlt_21,dlt_43,dlt_32);
	ROS_INFO("len_F = %d,  len_R = %d  ,car_len = %d  \r\n",car_wheel_data[0].rear_wheel_len,car_wheel_data[0].front_wheel_len,car_wheel_data[0].car_len); 
	#if 1  //前期只有一个轮子求半径
	memcpy(&car_wheel_data[1],&car_wheel_data[0],sizeof(car_wheel_data[0])); 
	#endif
	wheel_data->front_wheel_len = (car_wheel_data[0].front_wheel_len + car_wheel_data[1].front_wheel_len)/2;
	wheel_data->rear_wheel_len = (car_wheel_data[0].rear_wheel_len + car_wheel_data[1].rear_wheel_len)/2;
	wheel_data->car_len = (car_wheel_data[0].car_len + car_wheel_data[1].car_len)/2;
	/*前轮半径*/
	temp1 = Rob_DimensionPara.ROB_laser_height*Rob_DimensionPara.ROB_laser_height; //h2
	temp2 = wheel_data->front_wheel_len*wheel_data->front_wheel_len; //L2
	temp3 = temp2/4; // 1/4
	temp4 = Rob_DimensionPara.ROB_laser_height*2;
	wheel_data->front_wheel_radius = (u16)((temp1 + temp3)/temp4);
	ROS_INFO("前轮半径: %d\r\n",wheel_data->front_wheel_radius);   


	/*后轮半径*/
	temp1 = Rob_DimensionPara.ROB_laser_height*Rob_DimensionPara.ROB_laser_height; //h2
	temp2 = wheel_data->rear_wheel_len*wheel_data->rear_wheel_len; //L2
	temp3 = temp2/4; // 1/4
	temp4 = Rob_DimensionPara.ROB_laser_height*2;
	wheel_data->rear_wheel_radius = (u16)((temp1 + temp3)/temp4);
	ROS_INFO("后轮半径: %d\r\n",wheel_data->rear_wheel_radius);
	
	if((wheel_data->car_len < CAR_LEN_MIN)||(wheel_data->car_len > CAR_LEN_MAX))
		{
			err_code_process(CAR_LEN_ERR);
			return 1;
		}
	return 0;
}

/*******************************************************************
name	:car_wheel_aim
funtion	:车轮对准
input	:none
output	:none
*******************************************************************/
static void car_wheel_aim(ST_wheel_data wheel_data)
{
	int FrontWheel_MoveMode = 0;
	/*前轮对准,未考虑车轮直径小于25CM的情况*/
	FrontWheel_MoveMode = (wheel_data.front_wheel_len/2) - Rob_DimensionPara.laser_RobWheel_distance;
	if(FrontWheel_MoveMode > 0)
		{//机器人前轮靠前，后移
			car_GoBack(PO_MODE, FrontWheel_MoveMode);
			usleep(5000);
			//car_GoBack(PO_MODE, FrontWheel_MoveMode);
			//usleep(5000);
			//1先不伸缩RearWheel_GoBack(PO_MODE,(wheel_data.car_len - CAR_LEN_MIN));	
		}
	else if((FrontWheel_MoveMode < 0))
		{
			FrontWheel_MoveMode += 50;
			car_GoAhead(PO_MODE, -FrontWheel_MoveMode);
			usleep(5000);
			//car_GoAhead(PO_MODE, Rob_DimensionPara.laser_RobWheel_distance -(wheel_data.front_wheel_len/2));
			//usleep(5000);
			//1先不伸缩RearWheel_GoBack(PO_MODE,(wheel_data.car_len - CAR_LEN_MIN));	
		}
	else
		{
			//1先不伸缩RearWheel_GoBack(PO_MODE,(wheel_data.car_len - CAR_LEN_MIN));
		}
}

/*******************************************************************  
name	:car_pose_redress
funtion	:实现车尾姿态矫正
input	:x，汽车中心对车位中心在X轴上的偏移量
		:y,汽车中心对车位中心在y轴上的偏移量
		:dirgee 机器人中轴和汽车中轴在y方向上的夹角
		约定机器人在汽车的左侧，夹角为正，机器人需要向右平移，在汽车的右侧，夹角为负，机器人需要向左平移
		:l,机器人中心位置和车位中心位置之间的距离
output	:none
*******************************************************************/
static void car_pose_redress(int x, int y, int l,double digree)
{/*
A:按视频出的算法
1、约定机器人前向中轴与汽车后向中轴之间的第三象限角夹角为θ角，
2、约定该θ角小于90度为负数，大于90度为真实角度减90。
3、按照1和2的约定，则有:θ< 0:机器人在汽车的中心位置偏左，θ> 0:机器人在汽车的中心位置偏右
B:按杨思琦出的算法:
1、约定机器人在汽车的左侧，夹角为正，机器人需要向右平移，在汽车的右侧，夹角为负，机器人需要向左平移
*/

/*以下代码按照杨思琦算法来实现，尚未对x、y和θ进行滤波处理*/

	u32 move_distance = 0;
	double temp1 = 0.0; 
	double temp2 = 0.0;
	
	temp1 = tan(digree);
	temp2 = temp1*(l+y);
	temp2 = temp2 + x;
	move_distance = (u32)temp2;
	if(digree < 0)
		{
			car_move_x(1,move_distance);
			car_spin(0,(u16)digree);
		}
	else if(digree > 0)
		{
			car_move_x(0,move_distance);
			car_spin(1,(u16)digree);
		}
	else
		{
			;
		}
}

/*******************************************************************
name	:car_lift
funtion	:实现整车举起行走和释放
input	:none
output	:夹车成功返回0，夹车失败返回1
*******************************************************************/
void car_lift_test(void)
{
	/*收缩*/
	wheel_lift(1, 70);usleep(5000);wheel_lift(1, 70);usleep(5000);
	wheel_lift(2, 70);usleep(5000);wheel_lift(2, 70);usleep(5000);
	wheel_lift(3, 70);usleep(5000);wheel_lift(3, 70);usleep(5000);
	wheel_lift(4, 70);usleep(5000);wheel_lift(4, 70);usleep(5000);

	set_acc(10);  
	sleep(1.2);
	car_GoAhead(PO_MODE, 2000);car_GoAhead(PO_MODE, 2000);
	
	//car_stop();car_stop();
	sleep(2.8);
	car_GoBack(PO_MODE, 2000); car_GoBack(PO_MODE, 2000); 
	
	//car_stop();car_stop();
	sleep(2.5);
	/*释放*/
	wheel_lift(1, -70);usleep(5000);wheel_lift(1, -70);usleep(5000);
	wheel_lift(2, -70);usleep(5000);wheel_lift(2, -70);usleep(5000);
	wheel_lift(3, -70);usleep(5000);wheel_lift(3, -70);usleep(5000);
	wheel_lift(4, -70);usleep(5000);wheel_lift(4, -70);usleep(5000);
	sleep(1.3);
	car_GoBack(SP_MODE, 20);
	sleep(2.4);
	set_acc(10);  
	car_stop();car_stop();
} 


/*夹车*/
void car_lift(void)
{
	/*收缩*/
	#if 1
	wheel_lift(1, 80);usleep(5000);wheel_lift(1, 80);usleep(5000);
	wheel_lift(2, 80);usleep(5000);wheel_lift(2, 80);usleep(5000);
	wheel_lift(3, 80);usleep(5000);wheel_lift(3, 80);usleep(5000);
	wheel_lift(4, 80);usleep(5000);wheel_lift(4, 80);usleep(5000);
	#else
	ROS_INFO("Jia Che Zhong .......................");
	#endif
} 
 

/*释放*/
void car_lay_down(void)
{
	#if 1
	wheel_lift(1, -80);usleep(5000);wheel_lift(1, -80);usleep(5000);
	wheel_lift(2, -80);usleep(5000);wheel_lift(2, -80);usleep(5000);
	wheel_lift(3, -80);usleep(5000);wheel_lift(3, -80);usleep(5000);
	wheel_lift(4, -80);usleep(5000);wheel_lift(4, -80);usleep(5000); 
	#else
	ROS_INFO("Shi Fang Jia Gan ......................."); 
	#endif
}


/*******************************************************************
name	:laser_open_close
funtion	:激光测距开关
input	:cmd = 0,关闭； cmd = 1, 开启
output	:none
*******************************************************************/   
void laser_open_close(u8 cmd)
{
		// 发送05启动测距
	CMC_SendData_05(0x0f,0x05,cmd);
	CMC_SendData_05(0x10,0x05,cmd);
}


/*******************************************************************
name	:SongKai_ChiLun
funtion	:松开刹车齿轮
input	:select : 1 松开前端齿轮，2松开后端齿轮
output	:;
*******************************************************************/
void SongKai_ChiLun(u8  select)
{
	if(select  == 1)
    ;
	 	// HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET) ;
	 else
	  	// HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET) ;
          ;
}

/*******************************************************************
name	:SuoJIng_ChiLun
funtion	:锁紧齿轮
input	:
output	:
*******************************************************************/
void SuoJIng_ChiLun(u8  select)
{

	if(select  == 1)
	 	// HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET) ;
         ;
	 else
	  	// HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET) ;
          ;
}


/*******************************************************************
name	:
funtion	:计算AGV和汽车水平轴之间的夹角 
input	:
output	:
*******************************************************************/

/*******************************************************************
name	:float_to_int
funtion	:实现四舍五入取整数 
input	:浮点数
output	:四舍五入后的整数
*******************************************************************/
int float_to_int(double data)
{
	int res = 0;
		
	res = (int)data;
	if(data - res > 0.5)
		res = ceil(data);
	else
		res = floor(data);
	return res;
}


void laser_time_start(void)
{
	u16 res = 0;
	
	tim6_mode[0] = 1;
	res = tim6_mode[0] + (tim6_mode[1] << 8);
	switch(res)
		{
			case 0:
				tim6_work_mode = tim6_no_work;
				break;
			case 1:
				tim6_work_mode = tim6_laser_work;
				break;
			case 256:
				tim6_work_mode = tim6_can_work;
				break;
			case 257:
				tim6_work_mode = tim6_all_work;
				break;
			default:
				tim6_work_mode = tim6_no_work;
				break;
		}
}

void laser_time_stop(void)
{
	u16 res = 0;
	
	tim6_mode[0] = 0;
	switch(res)
	{
		case 0:
			tim6_work_mode = tim6_no_work;
			break;
		case 1:
			tim6_work_mode = tim6_laser_work;
			break;
		case 256:
			tim6_work_mode = tim6_can_work;
			break;
		case 257:
			tim6_work_mode = tim6_all_work;
			break;
		default:
			tim6_work_mode = tim6_no_work;
			break;
	}
}

void can_time_start(void)
{
	u16 res = 0;
	
	tim6_mode[1] = 1;
	switch(res)
	{
		case 0:
			tim6_work_mode = tim6_no_work;
			break;
		case 1:
			tim6_work_mode = tim6_laser_work;
			break;
		case 256:
			tim6_work_mode = tim6_can_work;
			break;
		case 257:
			tim6_work_mode = tim6_all_work;
			break;
		default:
			tim6_work_mode = tim6_no_work;
			break;
	}
}

void can_time_stop(void)
{
	u16 res = 0;
	
	tim6_mode[1] = 0;
	switch(res)
	{
		case 0:
			tim6_work_mode = tim6_no_work;
			break;
		case 1:
			tim6_work_mode = tim6_laser_work;
			break;
		case 256:
			tim6_work_mode = tim6_can_work;
			break;
		case 257:
			tim6_work_mode = tim6_all_work; 
			break;
		default:
			tim6_work_mode = tim6_no_work;
			break;
	}
}



/*******************************************************************
name	:AGVPose_SetRight_Ahead
funtion	:AGV姿态矫正，实现轮前角度矫正,前进方向
input	:
output	:
*******************************************************************/
#if 0
void AGVPose_SetRight_Ahead(void)
{
	u32 wait_time = 0;

	u32 L_y = 0; //偏移角的对边
	u32 L_x = 0; //偏移角的邻边
	double hu_du  = 0.0;
	double Jiao_du = 0;
	u32 Hui_Tui_JuLi = 0; //回退距离
	
	timer_count = 0; 
	timer_mode = 0x01;
	memset(&Set_right_time,0x00,sizeof(Set_right_time));
	laser_open_close(1);
	laser_time_start();
	wait_time = tickclock;
	car_GoAhead(SP_MODE,CAR_SPEED_Wheel/10);  
	while(!((Set_right_time.L_time > 0)&&(Set_right_time.R_time > 0)))
	{
		if((tickclock - wait_time) >( 1500000/CAR_SPEED_Wheel)) //1.5M
			{
				car_GoBack(PO_MODE,1500);
				break;
			}
	}
	car_stop();
	laser_open_close(0);
	laser_time_stop(); 
	/*偏移角计算*/
	if(Set_right_time.L_time < Set_right_time.R_time)
		{// AGV左轮靠前
			L_y = (Set_right_time.R_time - Set_right_time.L_time)*CAR_SPEED_Wheel/1000;
		}
	else
		{
			L_y = (Set_right_time.L_time - Set_right_time.R_time)*CAR_SPEED_Wheel/1000;
		}
	L_x = Rob_DimensionPara.Rob_width + Set_right_time.L_distance;
	hu_du = atan(L_y/L_x);
	Jiao_du = hu_du*180/3.14159;

	/*角度纠正*/
	if(Set_right_time.L_time < Set_right_time.R_time)
		{// AGV左轮靠前，逆时针旋转
			Hui_Tui_JuLi = CAR_SPEED_Wheel*Set_right_time.R_time/1000;
			car_GoBack(PO_MODE,Hui_Tui_JuLi);
			car_spin(1,float_to_int(Jiao_du));
		}
	else
		{
			Hui_Tui_JuLi = CAR_SPEED_Wheel*Set_right_time.L_time/1000;
			car_GoBack(PO_MODE,Hui_Tui_JuLi);
			car_spin(1,float_to_int(Jiao_du)); 
		}
}


/*******************************************************************
name	:AGVPose_SetRight_Back
funtion	:AGV姿态矫正，实现轮前角度矫正,后退方向,AGV钻出车底时使用
input	:
output	:
*******************************************************************/
void AGVPose_SetRight_Back(ST_triger_time triger_time)
{
	//u32 wait_time = 0;

	u32 L_y = 0; //偏移角的对边
	u32 L_x = 0; //偏移角的邻边
	double hu_du  = 0.0;
	double Jiao_du = 0;
	//u32 Hui_Tui_JuLi = 0; //回退距离
	

	/*偏移角计算*/
	if(triger_time.FL.t4 < triger_time.FR.t4)
		{// AGV左轮靠前
			L_y = (triger_time.FR.t4 - triger_time.FL.t4)*CAR_SPEED_Wheel/1000;
		}
	else
		{
			L_y = (triger_time.FL.t4 - triger_time.FR.t4)*CAR_SPEED_Wheel/1000;
		}
	L_x = Rob_DimensionPara.Rob_width + Set_right_time.L_distance;
	hu_du = atan(L_y/L_x);
	Jiao_du = hu_du*180/3.14159;

	/*角度纠正*/
	if(triger_time.FL.t4 > triger_time.FR.t4)
		{// AGV右轮靠前，顺时针旋转
			//Hui_Tui_JuLi = CAR_SPEED_Wheel*Set_right_time.L_time/1000;
			//car_GoBack(PO_MODE,Hui_Tui_JuLi);
			/*要考虑将旋转角度转化为以AGV中心角度的旋转*/ 
			car_spin(1,float_to_int(Jiao_du)); 
		}
	else
		{
			//Hui_Tui_JuLi = CAR_SPEED_Wheel*Set_right_time.R_time/1000;
			//car_GoBack(PO_MODE,Hui_Tui_JuLi);
			car_spin(1,float_to_int(Jiao_du));
		}
}


/*******************************************************************
name	:auto_lift
funtion	:实现自动夹车
input	:none
output	:夹车成功返回0，夹车失败返回1
*******************************************************************/
/*
调试要点:
1、小车匀速进入的速度未计算
*/
u8 auto_lift(void)
{
	u32 wait_time = 0;
	ST_triger_time wheel_triger_time; //车轮触发传感器时间
	ST_wheel_data car_wheel_data; //小车和车轮信息 轴距和弦长
	
	/*车尾位置矫正*/

	/*车尾姿态矫正*/
	//car_pose_redress(int x,int y,int l,double digree);
	/*测距开始,前进*/
	memset(wheel_time_buff,0x00,sizeof(wheel_time_buff));
	laser_open_close(1);
	/* 在中断模式下启动定时器 */
	timer_count = 0; 
	timer_mode = 0x00;
  	laser_time_start();
	memset(&wheel_triger_time,0x00,sizeof(ST_triger_time)); 
	memset(&car_wheel_data,0x00,sizeof(ST_wheel_data));
	wait_time = tickclock;
	car_GoAhead(SP_MODE,CAR_SPEED_Wheel/10);  
	while(!(wheel_time_buff[0].t0_count == 2)) 
	//while(!((wheel_time_buff[0].t0_count == 2)&&(wheel_time_buff[1].t0_count == 2))) //前轮两次测到车轮
	{
		if(wheel_triger_time.FL.t4 > 0)
			break;
		if(wheel_triger_time.FR.t4 > 0)
			break;
		if((tickclock - wait_time) >( 4000000/CAR_SPEED_Wheel))
			break;
	}
	/*停车，测距数据处理*/
	//laser_open_close(0);
	laser_time_stop(); 
	car_stop();car_stop();
	memcpy(&wheel_triger_time.FL,&wheel_time_buff[0],sizeof(wheel_time_buff[0]));
	memcpy(&wheel_triger_time.FR,&wheel_time_buff[1],sizeof(wheel_time_buff[0]));
	memcpy(&wheel_triger_time.RL,&wheel_time_buff[2],sizeof(wheel_time_buff[0]));
	memcpy(&wheel_triger_time.RR,&wheel_time_buff[3],sizeof(wheel_time_buff[0]));
	wheel_data_process(wheel_triger_time, &car_wheel_data);
	while(ps2_scan())
		{
			if (((ps2KeyBitmap & O_KEY_BIT) == O_KEY_BIT) && ((ps2KeyBitmap & R1_KEY_BIT) == R1_KEY_BIT))
				{
					controlMode = 3;
					return 0;   
				} 
		}   
	/*车轮对准*/
	car_wheel_aim(car_wheel_data);
	/*车底姿态矫正*/
	/*夹车*/  
	car_lift();
	return 0;
}    

/*******************************************************************
name	:auto_lift_test
funtion	:自动夹车测试程序
input	:none 
output	:none
*******************************************************************/
void auto_lift_test(void)
{
	auto_lift();
	#if 0
	car_GoBack(PO_MODE, 300);
	delay_us(10000);
	car_GoAhead(PO_MODE, -300);
	delay_us(10000);
	#endif
}


/**
  * 函数功能: 非阻塞模式下定时器的回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	switch(tim6_work_mode)
		{
			case tim6_no_work:
				break;
			case tim6_laser_work:
				timer_count++;
				break;
			case tim6_can_work:
				//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);	
				if(can_tim6_count %15 == 0)
					{//发送速度
     						//AGV_SPEED_Enquire(moto_select); 
     						//if(send_speed_count < set_WheelSpeedBuff_count)
     							{
								//set_motor_speed(2, set_WheelSpeedBuff[send_speed_count++]);
							}
						//get_car_speed();
						send_flag = 1;
					}
				//if(can_tim6_count %20 == 0)
					{//发送速度读取指令
						//AGV_SPEED_Enquire(1);  
					}
				can_tim6_count++;
				if(can_tim6_count >= 1000)
					can_tim6_count = 0;
				break;
			case tim6_all_work:
				timer_count++;
				if(can_tim6_count %10 == 0)
					{//发送速度
     						;
					}
				else
					{//发送速度读取指令
						;
					}
				can_tim6_count++;
				if(can_tim6_count >= 1000)
					can_tim6_count = 0;
		
				break;
		}
}

#endif
/*******************************************************************
name	:
funtion	:
input	:
output	:
*******************************************************************/



/*******************************************************************
name	:open_ShengSuo_laser
funtion	:开启伸缩激光测距传感器
input	:
output	:
*******************************************************************/
void open_ShengSuo_laser(void)
{
	;
}

/*******************************************************************
name	:close_ShengSuo_laser
funtion	:关闭伸缩激光测距传感器
input	:
output	:
*******************************************************************/
void close_ShengSuo_laser(void)
{
	;  
}

/*******************************************************************
name	:car_ShengZhan
funtion	:小车向后伸展
input	:len 伸展的长度,单位:MM
output	:
*******************************************************************/
void car_ShengZhan(u16 len)
{
    #if 0
	SongKai_ChiLun(1);
	SongKai_ChiLun(2);	
	RearWheel_GoBack(PO_MODE,len);
	open_ShengSuo_laser();
    #endif
    ;
}

/*******************************************************************
name	:car_ShouSuo
funtion	:小车向后伸展
input	:len 伸展的长度,单位:MM
output	:
*******************************************************************/
void car_ShouSuo(u16 len)
{
    #if 0
	SongKai_ChiLun(1);
	SongKai_ChiLun(2);	
	RearWheel_GoAhead(PO_MODE,len);
	//open_ShengSuo_laser();
    #endif
    ;
}


/*******************************************************************
name	:
funtion	:
input	:
output	:
*******************************************************************/
/******************************************************************* 
name	:get_FrontalWheel_status
funtion	:获取前轮运动状态
input	:LR_flag : 0--左前， 1--右前
output	:返回0表示轮子没有转动，大于0表示轮子当前的转速
*******************************************************************/
float get_FrontalWheel_status(u8 LR_flag)
{
	float Wheel_Speed = 0.0;
	if(LR_flag)
		{
			AGV_SPEED_Enquire(2);
			Wheel_Speed = Get_Speed2;
			//HAL_Delay(100);
		}
	else
		{
			AGV_SPEED_Enquire(1);
			Wheel_Speed = Get_Speed1;
			//HAL_Delay(100);
		}
		return Wheel_Speed;
}

/******************************************************************* 
name	:get_RearWheel_status
funtion	:获取后轮运动状态
input	:LR_flag : 0--左后， 1--右后
output	:返回0表示轮子没有转动，大于0表示轮子当前的转速
*******************************************************************/
float get_RearWheel_status(u8 LR_flag) 
{
	float Wheel_Speed = 0.0;
	if(LR_flag)
		{
			AGV_SPEED_Enquire(4);
			Wheel_Speed = Get_Speed4;
			//HAL_Delay(100);
		}
	else
		{
			AGV_SPEED_Enquire(3);
			Wheel_Speed = Get_Speed3; 
			//HAL_Delay(100);
		}
		return Wheel_Speed;
}

/*******************************************************************
name	:sys_source_init 
funtion	:系统资源初始化
input	:none
output	:none
*******************************************************************/
void sys_source_init(void)
{
	/*机器人尺寸参数初始化*/
	Rob_DimensionPara.laser_RobWheel_distance = 522;    
	Rob_DimensionPara.Rob_JiaGan_Banjing = 530;
	Rob_DimensionPara.Rob_JiaGan_GaoDu = 45;
	Rob_DimensionPara.ROB_laser_height = 88; 
	Rob_DimensionPara.Rob_WheelDistance_max = 3000;  
	Rob_DimensionPara.Rob_WheelDistance_min = 2400;
	Rob_DimensionPara.Rob_len = 3700;
	Rob_DimensionPara.Rob_width = 1200;

	/*机器人尺寸参数初始化*/
	Rob_DimensionPara_float.laser_RobWheel_distance = 0.522;    
	Rob_DimensionPara_float.Rob_JiaGan_Banjing = 0.53;
	Rob_DimensionPara_float.Rob_JiaGan_GaoDu = 0.045;
	Rob_DimensionPara_float.ROB_laser_height = 0.088; 
	Rob_DimensionPara_float.Rob_WheelDistance_max = 3;  
	Rob_DimensionPara_float.Rob_WheelDistance_min = 2.4;
	Rob_DimensionPara_float.Rob_len = 3.7;
	Rob_DimensionPara_float.Rob_width = 1.2;
}


/*******************************************************************
name	:get_car_len
funtion	:获取小车长度
input	:none
output	:返回0:错误值， 大于0 :小车长度
*******************************************************************/
u16 get_car_len(void)
{
//	int tmp1, tmp2, tmp3;
	//u16 res = 0;

	#if 0
	tmp1 = car_len_buff[0] - car_len_buff[1];
	if((tmp1 > 20)||(tmp1 < -20))
		{
			res = 0;
			return res;
		}
	tmp2 = car_len_buff[1] - car_len_buff[2];
	if((tmp2 > 20)||(tmp2 < -20))
		{
			res = 0;
			return res;
		}
	tmp3 = car_len_buff[0] - car_len_buff[2];
	if((tmp2 > 20)||(tmp2 < -20))
		{
			res = 0;
			return res;
		}
	return (car_len_buff[0] + car_len_buff[1] + car_len_buff[2])/3;
	#endif
	if((car_len >= 2390)&&(car_len < 3200))
		return car_len;
	else
		return 0;
}

/*******************************************************************
name	:car_status_init
funtion	:小车状态初始化
input	:none
output	:none
*******************************************************************/
void car_status_init(void)
{
	u32 tick_timer;
	u16 res_car_len = 0;  
	u16 move_size = 0;
	tick_timer = tickclock; 
	/*将小车处于收缩状态*/
	open_ShengSuo_laser();
	while(!res_car_len)
		{
			res_car_len = get_car_len();
			if(tickclock - tick_timer >10000)
				{
					/*超时错误处理*/
					break;
				}
		}
	move_size = res_car_len - Rob_DimensionPara.Rob_WheelDistance_min;
	tick_timer = tickclock; 
	while(move_size > 30) //小车缝隙
		{
			res_car_len = get_car_len();
			if(!res_car_len)
				continue;
			if(res_car_len < 30)
				res_car_len -= 10;
			if(res_car_len <= 25)
				break;
			move_size = res_car_len - Rob_DimensionPara.Rob_WheelDistance_min;
			ROS_INFO("小车长度:%d  , 伸缩长度:%d\r\n",res_car_len, move_size);
			car_ShouSuo(move_size);
			usleep((move_size/PO_mode_speed)*1000);
			if(tickclock - tick_timer >10000)
				{
					/*超时错误处理*/
					break;
				}
		}

	/*齿轮处于锁死状态*/
	SuoJIng_ChiLun(0);
	SuoJIng_ChiLun(1);
	/*将夹杆处于收回状态*/

	/*避障传感器处于开启状态*/
}

/*
小车收缩n mm:
1、开启测距命令:
2、开启收缩n mm命令
3、等待动作执行完毕
4、读取车身距离，判断收缩到位没有
5、收缩到位则退出程序，否则执行6
6、
*/

/*******************************************************************
name	:car_ShouSuo_FB
funtion	:小车车体收缩,带反馈
input	:n 要收缩的毫米数
output	:成功返回0，超时返回1
*******************************************************************/
u8 car_ShouSuo_FB(u16 n)
{
	int L1 = 0; //车身初始长度
//	int L2 = 0; // 车身当前长度
//	int dL = 0; // L1 -L2  
	u32 t1 = 0; //  电机运动到三分之一路程
	u32 t2 = 0; //电机运动完全程
	u32 t3 = 0; // 超时时间，电机运动完全程的3倍时间
	u32 time_out = 0; //超时时间
	float wheel_speed = 0.0;

	open_ShengSuo_laser();
	t2 = n/PO_mode_speed;
	t1 = t2/3;
	t3 = t2*3;

	time_out = tickclock;
	while(!(get_car_len() > 0))
		{
			if(tickclock - time_out > 3000)
				{
					return 1;
				}
		}
	L1 = get_car_len();
	time_out = tickclock;
	while(1)
		{
			car_ShouSuo(n);
			if(tickclock - time_out > t1)
				{
					wheel_speed = get_RearWheel_status(0);
					if(wheel_speed < 0)
						wheel_speed = 0 - wheel_speed;
					if((wheel_speed > 0)&&(wheel_speed < 0.0001))  //轮子没转
						{
							continue;
						}
					wheel_speed = get_RearWheel_status(1);
					if(wheel_speed < 0)
						wheel_speed = 0 - wheel_speed;
					if((wheel_speed > 0)&&(wheel_speed < 0.0001))  //轮子没转
						{
							continue;
						}
				}
		}
}


/*******************************************************************
name	:test_check_car_len
funtion	:车身长度检测测试程序
input	:
output	:
*******************************************************************/
void test_check_car_len(void)
{
;
}


void test_car_lift(void)
{
	car_lift();
	#if 0
	wheel_lift(3, 70);
	wheel_lift(3, 70);

	wheel_lift(4, 70);
	wheel_lift(4, 70);
	
	HAL_Delay(8000);
	wheel_lift(3, -70);
	wheel_lift(3, -70);

	wheel_lift(4, -70);
	wheel_lift(4, -70);
	#endif
}

void test_Sheng_Suo(void)    
{
	#if 1
//	SongKai_ChiLun(1);
	FrontalWheel_GoAhead(PO_MODE, 450);
//	car_stop();car_stop();
//	SuoJIng_ChiLun(1);
//	SongKai_ChiLun(2);
	sleep(10);
	RearWheel_GoAhead(PO_MODE,  450);
	sleep(10);
	RearWheel_GoBack(PO_MODE,  450);
//	car_stop();car_stop();
//	SuoJIng_ChiLun(1);
//	SongKai_ChiLun(2);
	sleep(10);
	FrontalWheel_GoBack(PO_MODE, 450);
	sleep(20);
//	SuoJIng_ChiLun(2);
	#else
	SuoJIng_ChiLun(1);
	SuoJIng_ChiLun(2);
	//car_ShengZhan(400);
	HAL_Delay(10000);
	//car_ShouSuo(400);
	HAL_Delay(10000);
	#endif
}




/*******************************************************************
name	:PID_arg_init 
funtion	:实现pid参数初始化
input	:none
output	:none  
*******************************************************************/
void PID_arg_init(void)  
{
	PID.set_v = 0.6; //车身长度2.7m
	PID.Kp = 10;    
	PID.ctrl_T = 30; //控制周期为100ms 
	PID.Ti = 100000000;   
	PID.Td = 0; 
	PID.SEk =0.0;     
	PID.Out_0 = 0.0005;  
}


/*******************************************************************
name	:PID_SF
funtion	:实现PID算法处理  
input	:
output	:
*******************************************************************/
float PID_SF(void)  
{
	float DelTa_ek = 0.0; //der ta
	float ti,ki,kd;  
	float i_out;
	float p_out;
	float d_out;
	float td;

	PID.Ek = PID.set_v - PID.FB_v;
	PID.SEk += PID.Ek;   
	DelTa_ek = PID.Ek - PID.Ek_1;
  
	ti = PID.ctrl_T/PID.Ti;
	ki = ti*PID.Kp;
	i_out =ki* PID.SEk*PID.Kp;  

	p_out = PID.Kp *PID.Ek;
	 
	td = PID.Td/PID.ctrl_T;
	kd = PID.Kp * td;
	d_out = kd*DelTa_ek;

	PID.Ek_1 = PID.Ek;

	PID.PID_out = p_out + i_out +d_out + PID.Out_0;  
	if(PID.PID_out > 1.49)
		PID.PID_out = 1.49; 
	if(PID.PID_out < -1.49)   
		PID.PID_out = -1.49;
	return PID.PID_out;
}


void speed_pid(void)    
{
	u8 pid_flag = 0;
	u16 set_v_count = 0;
	double i = 0;
	while(1)
		{
			if(!FB_speed_flag) // 定时计数器位达到控制周期
				{
					//if(!pid_flag)   
						set_motor_speed(moto_select,PID.set_v + 0.0005);   
						//set_car_speed(PID.set_v + 0.0005);    
					continue;      
				}      
			else    
				{   
					FB_speed_flag = 0;     
					pid_flag = 1;
					PID.FB_v = Get_Speed4;    
					if(set_v_count >=  1161)
						set_v_count = 0;
					//PID.set_v = set_WheelSpeedBuff[set_v_count++];     
					PID.set_v = 0.4*sin(i)+0.4;  
					i += 0.0205;  
					if(i >=  3.5*3.14)  
						{
							//i = 0;
							car_stop();
							car_stop();
							return ;
						}
					PID_SF();
					PID.PID_out =PID.set_v; 
					set_motor_speed(moto_select,PID.PID_out);
					//set_car_speed(PID.PID_out);  
					//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);	  
					ROS_INFO("%8f  %8f  %8f \r\n",PID.PID_out,PID.FB_v,PID.set_v);
				}
		}
}


/*******************************************************************
name	:
funtion	:
input	:
output	:
*******************************************************************/





