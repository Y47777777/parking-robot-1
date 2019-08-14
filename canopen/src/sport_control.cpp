#include "ros/ros.h"
#include "canopen/sport_control.h"
#include "math.h"
#include "canopen/car_driver.h"
#include "canopen/auto_lift.h"
#include "canopen/MOTEC_motorCtl.h"
#include "string.h"
#include "canopen/type.h"
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "canopen/timer.h"
#include "canopen/sensor_cmc.h"
#include<time.h> 

#define Mode_ShouSuo 0
#define Mode_LaSheng 1

STR_five_Sa five_Sa; //定义多项式系数
u8 send_flag = 0;  // 数据发送标志
u8 car_go_v0_return = 0; //  退出当前行走轨迹，轨迹切换，== 1时有效
float car_go_v0_speed_now  = 0.0 ;//小车退出当前曲线轨迹时的当前速度
ST_wheel_data_pid wheel_data_pid;  //添加PID和曲线规划之后的车轮数据 
volatile u8 Cmd_Laser = 0; //激光传感器检测到轮胎发出的命令 
volatile u8 beep_time = 0; //蜂鸣器间隔时间
volatile u8 en_dis_beep_time = 0; //蜂鸣器计数使能
volatile int time_fd=0;
volatile char control_cycle=0;
volatile u32 tickclock=0;
  
/*******************************************************************
name	:get_Gui_Ji
funtion	:求解轨迹方程
input	:step 步长  
		:Vmax 轨迹最大速度1.5  
		:Amax  轨迹最大加速度0.4   
		:s  行走距离
		:V0 初始速度
		:Vt 结束速度
		:dir 方向1为前进 -1为后退
output	:成功返回0   失败返回1
*******************************************************************/
#if 0
void get_Gui_Ji(float vmax, float amax, float s, char dir, float V0, float Vt, float step)  
{
	//double t0 = 0.0;
	double t,t2;
	double pmax; //位移
	double a5,a4,a3,a2,a1,a0;
	double p1,v1,acc1,ti1; //加速阶段曲线
	double p2,v2,acc2,ti2; //匀速阶段
	double p3,v3,acc3,ti3; //减速阶段     
	double p,v,acc,ti; // 加速时间较短的情况
	//double i = 0;
	//double tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7;


	/*计算以最大加速度加速到最大速度的曲线，默认初始和结束的速度与加速度均为0*/
	a2=0;
	a1=0;
	a0=0;
	//t0=0;
	a5 =(16*amax*amax*amax*amax)/(405*vmax*vmax*vmax);
	a4 =-(8*amax*amax*amax)/(27*vmax*vmax);
	a3 =(16*amax*amax)/(27*vmax);
	t =(3*vmax)/(2*amax);
	pmax=a0+a1*pow(2*t,1)+a2*pow(2*t,2)+a3*pow(2*t,3)+a4*pow(2*t,4)+a5*pow(2*t,5);


	/*分类讨论 第一种情况：加速到最大速度后，以最大速度匀速行驶一段距离，最后减速到零*/
	if(s > pmax)
		{
			t2=(s-pmax)/vmax;
			/*第一阶段 加速*/
			//ti1=0:step:t;
			for(ti1 = 0; ti1 < t;)
			{
				if(send_flag)
					{
						send_flag = 0;
						p1=a0+a1*pow(ti1,1)+a2*pow(ti1,2)+a3*pow(ti1,3)+a4*pow(ti1,4)+a5*pow(ti1,5);  
						v1=a1+2*a2*pow(ti1,1)+3*a3*pow(ti1,2)+4*a4*pow(ti1,3)+5*a5*pow(ti1,4);
						acc1=2*a2+6*a3*pow(ti1,1)+12*a4*pow(ti1,2)+20*a5*pow(ti1,3);
						ti1 += step;
						if(dir > 0)
								set_car_speed(v);
							else
							  	set_car_speed(-v);
					}
			}

			/*第二阶段 匀速*/
			//ti2=t:step:(t+t2);
			for(ti2 = t; ti2 <= (t+t2);)  
				{
					if(send_flag)
						{
							send_flag = 0;
							p2=pmax/2+(ti2-t)*vmax;
							//s=length(ti2);
							v2=vmax; 
							acc2=0; 
							ti2 += step;
							if(dir > 0)
								set_car_speed(v);
							else
							  	set_car_speed(-v);
						}
				}

			/*第三阶段 减速阶段*/
			//ti3=(t+t2-step):step:(2*t+t2);
			for(ti3 = (t+t2); ti3 <= (2*t+t2);)
				{
					if(send_flag)
						{
							send_flag = 0;
							p3=a0+a1*pow((ti3-t2),1)+a2*pow((ti3-t2),2)+a3*pow((ti3-t2),3)+a4*pow((ti3-t2),4)+a5*pow((ti3-t2),5)+t2*vmax;
							v3=a1+2*a2*pow((ti3-t2),1)+3*a3*pow((ti3-t2),2)+4*a4*pow((ti3-t2),3)+5*a5*pow((ti3-t2),4);
							acc3=2*a2+6*a3*pow((ti3-t2),1)+12*a4*pow((ti3-t2),2)+20*a5*pow((ti3-t2),3);  
							ti3 += step;
							if(dir > 0)
								set_car_speed(v);
							else
							  	set_car_speed(-v);
						}
				}
		}//sqrt((8192*amax*amax*amax)/(3645*p))
	else /*第二种情况：加速到某一速度（该速度低于或等于最大速度）后立即减速*/
		{
			a5 =(4*amax*pow(((8192*amax*amax*amax)/(3645*s)),0.5))/(75*s);
			a4 = -(64*amax*amax)/(135*s);
			a3 =pow(((8192*amax*amax*amax)/(3645*s)),0.5)/2;
			t =(405*s*pow(((8192*amax*amax*amax)/(3645*s)),0.5))/(512*amax*amax);

			/*加速与减速曲线*/
			for(ti = 0; ti < 2*t; )
				{
					if(send_flag)
						{
							send_flag = 0; 
							p=a0+a1*pow(ti,1)+a2*pow(ti,2)+a3*pow(ti,3)+a4*pow(ti,4)+a5*pow(ti,5);
							v=a1+2*a2*pow(ti,1)+3*a3*pow(ti,2)+4*a4*pow(ti,3)+5*a5*pow(ti,4);
							acc=2*a2+6*a3*pow(ti,1)+12*a4*pow(ti,2)+20*a5*pow(ti,3); 
							//ROS_INFO("%8f  %8f  %8f \r\n",p,v,acc); 
							if(dir > 0)
								set_car_speed(v);
							else
							  	set_car_speed(-v);
							ti += step;
						}
				}
		}
}
#endif


/*******************************************************************
name	:sport_mode_analysis
funtion	:运动模式解析
input	:sport_mode 小车运动模式
output	:none
*******************************************************************/
void sport_mode_analysis(u8 car_sport_mode, float speed)
{
	float speed_temp = speed;
	switch(car_sport_mode)
		{
			case CarMode_GoBack://CarMode_GoAhead:
				set_car_speed(speed_temp);
				break;
			case CarMode_GoAhead://CarMode_GoBack:
				set_car_speed(-speed_temp);
				break;
			case CarMode_FrontWheel_GoAhead://CarMode_RearWheel_GoBack:
				set_motor_speed(4,speed_temp); 
				set_motor_speed(3,-speed_temp);
				break;
			case CarMode_RaarWheel_GoAhead://CarMode_FrontWheel_GoBack:
				set_motor_speed(2,speed_temp);
				set_motor_speed(1,-speed_temp);  
				break;
			case CarMode_FrontWheel_GoBack://CarMode_RaarWheel_GoAhead:
				set_motor_speed(4,-speed_temp); 
				set_motor_speed(3,speed_temp);
				break;
			case CarMode_RearWheel_GoBack://CarMode_FrontWheel_GoAhead:
				set_motor_speed(2,-speed_temp);
				set_motor_speed(1,speed_temp); 
			default:
				break;
		}
}

/****************************************************************
name	:laser_handle
function	:轮胎激光传感器处理
input	:cmd 模式命令
		:S_now 当前位移
		:V_now 当前速度
		:wheel_data 车轮数据
output	: 返回-1 :继续以当前曲线行驶
		:否则 停止检测 切换曲线
*****************************************************************/
float  laser_handle(u8 cmd, float S_now,ST_wheel_data_pid * wheel_data)
{
	float GoAhead_Distance = 0.0;
	float temp1 = 0.0; //中间计算结果
	float temp2 = 0.0; 
	float temp3 = 0.0;   
	float temp4 = 0.0; 
	
	switch(cmd)
		{   
			/*1、记录当前位置，将当前位置记为0*/
			case CmdLaser_CheckWheel_1:   
				wheel_data->last_p = S_now;
				ROS_INFO("Lun Tai 1_1 :S = %8f", S_now);
				//ROS_INFO("11111\r\n");
				Cmd_Laser = 0;
				break;
				 
			/*2、计算轮子弦长和直径*/
			case CmdLaser_CheckWheel_2:
				wheel_data->wheel_len = S_now - wheel_data->last_p;
				wheel_data ->last_p = S_now;
				//ROS_INFO("last p = %8f\r\n",wheel_data->last_p);
				//ROS_INFO("轮胎1_2 :S = %8f\r\n", S_now);
				temp1 = (Rob_DimensionPara_float.ROB_laser_height)*(Rob_DimensionPara_float.ROB_laser_height); //h2
				temp2 = wheel_data->wheel_len*wheel_data->wheel_len; //L2
				temp3 = temp2/4; // 1/4
				temp4 = Rob_DimensionPara_float.ROB_laser_height*2;
				wheel_data->wheel_radius = (temp1 + temp3)/temp4;  
				ROS_INFO("Hou Lun Ban Jin = %8f",wheel_data->wheel_radius);    
				//ROS_INFO("22222\r\n");
				Cmd_Laser = 0;
				break;
		
			case CmdLaser_CheckWheel_3:
				/*3、计算汽车轴距*/ 
				ROS_INFO("Lun Tai 2_1 :S = %8f", S_now);
				wheel_data->car_len = S_now - wheel_data->last_p + wheel_data->wheel_len ;
				/*4、计算下一步要行走的方向和距离*/
				GoAhead_Distance = wheel_data->wheel_len/2 + (Rob_DimensionPara_float.laser_RobWheel_distance); 
				wheel_data->last_p = S_now;
				GoAhead_Distance -= 0.05;
				ROS_INFO("Xiang Qian Yun Dong %8f Mi Zhi Hou Dui Zhun",GoAhead_Distance);
				//ROS_INFO("333333\r\n");
				Cmd_Laser = 0; 
				return GoAhead_Distance;
				//break;
			default :    
				break;
		}
	return -1;
}

/*******************************************************************
name	:Wheel_go_v0s
funtion	:小车从初速度为0开始走一段距离并停下来
input	:sport_mode 小车运动模式
		:s 想要移动的距离
		:vmax 最大速度
		:amax 最大加速度 
output	:成功返回1  失败返回0
*********************************************************************/
u8 Wheel_go_v0s(u8 sport_mode, float s, float vmax, float amax)
{
	double t0 = 0.0;
	double t,t2;
	double pmax; //位移
	double a5,a4,a3,a2,a1,a0;
	double p1,v1,acc1,ti1; //加速阶段曲线
	double p2,v2,acc2,ti2; //匀速阶段
	double p3,v3,acc3,ti3; //减速阶段   
	double p,v,acc,ti; // 加速时间较短的情况
	//double i = 0;
	double tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7;
	float GoAhead_Distance = -1;
    unsigned long data = 0;
	/*计算以最大加速度加速到最大速度的曲线，默认初始和结束的速度与加速度均为0*/
	a2=0;
	a1=0;
	a0=0;
	t0=0;
	a5 =(16*amax*amax*amax*amax)/(405*vmax*vmax*vmax);
	a4 =-(8*amax*amax*amax)/(27*vmax*vmax);
	a3 =(16*amax*amax)/(27*vmax);
	t =(3*vmax)/(2*amax);
	pmax=a0+a1*pow(2*t,1)+a2*pow(2*t,2)+a3*pow(2*t,3)+a4*pow(2*t,4)+a5*pow(2*t,5);
	//int chuankou = open ("/dev/ttyUSB1", O_RDWR|O_NOCTTY|O_NDELAY);
	/*分类讨论 第一种情况：加速到最大速度后，以最大速度匀速行驶一段距离，最后减速到零*/
	if(s > pmax)
		{
			t2=(s-pmax)/vmax;
			/*第一阶段 加速*/
			//ti1=0:step:t;
			for(ti1 = 0; ti1 < t;)
			{
				p1=a0+a1*pow(ti1,1)+a2*pow(ti1,2)+a3*pow(ti1,3)+a4*pow(ti1,4)+a5*pow(ti1,5);  
				v1=a1+2*a2*pow(ti1,1)+3*a3*pow(ti1,2)+4*a4*pow(ti1,3)+5*a5*pow(ti1,4);
				//acc1=2*a2+6*a3*pow(ti1,1)+12*a4*pow(ti1,2)+20*a5*pow(ti1,3);
				ti1 += SendV_Step;
				if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
				{
					ROS_INFO("Sport_control Timer Error");
				}
				sport_mode_analysis(sport_mode, v1);
			}
			/*第二阶段 匀速*/
			//ti2=t:step:(t+t2);
			for(ti2 = t; ti2 <= (t+t2);)     
				{
					p2=pmax/2+(ti2-t)*vmax;
					//s=length(ti2);
					v2=vmax; 
					acc2=0; 
					ti2 += SendV_Step;
					if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
						{
							ROS_INFO("Sport_control Timer Error");
						}
					sport_mode_analysis(sport_mode, v2);				
				}
			/*第三阶段 减速阶段*/
			//ti3=(t+t2-step):step:(2*t+t2);
			for(ti3 = (t+t2); ti3 <= (2*t+t2);)
				{
					send_flag = 0;
					p3=a0+a1*pow((ti3-t2),1)+a2*pow((ti3-t2),2)+a3*pow((ti3-t2),3)+a4*pow((ti3-t2),4)+a5*pow((ti3-t2),5)+t2*vmax;
					v3=a1+2*a2*pow((ti3-t2),1)+3*a3*pow((ti3-t2),2)+4*a4*pow((ti3-t2),3)+5*a5*pow((ti3-t2),4);
					//acc3=2*a2+6*a3*pow((ti3-t2),1)+12*a4*pow((ti3-t2),2)+20*a5*pow((ti3-t2),3);  
					ti3 += SendV_Step;
					if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
						{
							ROS_INFO("Sport_control Timer Error");
						}
					sport_mode_analysis(sport_mode, v3);	
				}
		}//sqrt((8192*amax*amax*amax)/(3645*p))
	else /*第二种情况：加速到某一速度（该速度低于或等于最大速度）后立即减速*/
		{
			a5 =(4*amax*pow(((8192*amax*amax*amax)/(3645*s)),0.5))/(75*s);
			a4 = -(64*amax*amax)/(135*s);
			a3 =pow(((8192*amax*amax*amax)/(3645*s)),0.5)/2;
			t =(405*s*pow(((8192*amax*amax*amax)/(3645*s)),0.5))/(512*amax*amax);

			/*加速与减速曲线*/
			for(ti = 0; ti < 2*t; )
				{ 
					p=a0+a1*pow(ti,1)+a2*pow(ti,2)+a3*pow(ti,3)+a4*pow(ti,4)+a5*pow(ti,5);
					v=a1+2*a2*pow(ti,1)+3*a3*pow(ti,2)+4*a4*pow(ti,3)+5*a5*pow(ti,4);
					//acc=2*a2+6*a3*pow(ti,1)+12*a4*pow(ti,2)+20*a5*pow(ti,3); 
					sport_mode_analysis(sport_mode, v);
					ti += SendV_Step;
					if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
						{
							ROS_INFO("Sport_control Timer Error");
						}
				}
		}   
}


/*******************************************************************
name	:car_go_v0s
funtion	:小车从初速度为0开始走一段距离并停下来
input	:sport_mode 小车运动模式
		:s 想要移动的距离
		:vmax 最大速度
		:amax 最大加速度 
output	:成功返回1  失败返回0
*********************************************************************/
u8 car_go_v0s(u8 sport_mode, float s, float vmax, float amax)
{
	double t0 = 0.0;
	double t,t2;
	double pmax; //位移
	double a5,a4,a3,a2,a1,a0;
	double p1,v1,acc1,ti1; //加速阶段曲线
	double p2,v2,acc2,ti2; //匀速阶段
	double p3,v3,acc3,ti3; //减速阶段   
	double p,v,acc,ti; // 加速时间较短的情况
	//double i = 0;
	double tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7;
	float GoAhead_Distance = -1;
    unsigned long data = 0;
	/*计算以最大加速度加速到最大速度的曲线，默认初始和结束的速度与加速度均为0*/
	a2=0;
	a1=0;
	a0=0;
	t0=0;
	a5 =(16*amax*amax*amax*amax)/(405*vmax*vmax*vmax);
	a4 =-(8*amax*amax*amax)/(27*vmax*vmax);
	a3 =(16*amax*amax)/(27*vmax);
	t =(3*vmax)/(2*amax);
	pmax=a0+a1*pow(2*t,1)+a2*pow(2*t,2)+a3*pow(2*t,3)+a4*pow(2*t,4)+a5*pow(2*t,5);
	//int chuankou = open ("/dev/ttyUSB1", O_RDWR|O_NOCTTY|O_NDELAY);
	/*分类讨论 第一种情况：加速到最大速度后，以最大速度匀速行驶一段距离，最后减速到零*/
	if(s > pmax)
		{
			t2=(s-pmax)/vmax;
			/*第一阶段 加速*/
			//ti1=0:step:t;
			for(ti1 = 0; ti1 < t;)
			{
				
				//if(send_flag)
					{
						//ROS_INFO("send_flag = %d",send_flag);
						send_flag = 0;
						p1=a0+a1*pow(ti1,1)+a2*pow(ti1,2)+a3*pow(ti1,3)+a4*pow(ti1,4)+a5*pow(ti1,5);  
						v1=a1+2*a2*pow(ti1,1)+3*a3*pow(ti1,2)+4*a4*pow(ti1,3)+5*a5*pow(ti1,4);
						//acc1=2*a2+6*a3*pow(ti1,1)+12*a4*pow(ti1,2)+20*a5*pow(ti1,3);
						ti1 += SendV_Step;
						if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
						{
							ROS_INFO("Sport_control Timer Error");
						}
						//write(chuankou,data,1);
						sport_mode_analysis(sport_mode, v1);
						//ROS_INFO("Cmd_Laser = %d  speed = %8f",Cmd_Laser,v1);
						ROS_INFO("Sport_mode = %d,  V1 = %8f",sport_mode,v1);
					}
				GoAhead_Distance = laser_handle(Cmd_Laser, p1, &wheel_data_pid);
				//Cmd_Laser = 0;
				if(GoAhead_Distance > 0)
					{
						//laser_open_close(0);
						//ROS_INFO("向前行走%8f后停下来\r\n",GoAhead_Distance);
						if(wheel_data_pid.car_len <= Rob_DimensionPara_float.Rob_WheelDistance_min-0.1)
							{
								
								if(v1 < 0.001)
									{
										continue;
									}
								else
									{
										car_stop();car_stop();
										usleep(100000);
										Wheel_go_v0s(CarMode_GoBack, wheel_data_pid.last_p, 0.4, 0.5);
										return 1;
									}
							}
						else
							{
								car_go_vxs(CarMode_GoAhead, GoAhead_Distance, v1, 0.5);
								return 0;
							}
					}
			}
			/*第二阶段 匀速*/
			//ti2=t:step:(t+t2);
			for(ti2 = t; ti2 <= (t+t2);)     
				{
					//if(send_flag)
						{
							send_flag = 0;
							p2=pmax/2+(ti2-t)*vmax;
							//s=length(ti2);
							v2=vmax; 
							acc2=0; 
							ti2 += SendV_Step;
							if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
								{
									ROS_INFO("Sport_control Timer Error");
								}
							//ROS_INFO("Cmd_Laser = %d  speed = %8f",Cmd_Laser,v2);
							sport_mode_analysis(sport_mode, v2);
						}
					GoAhead_Distance = laser_handle(Cmd_Laser, p2, &wheel_data_pid);
					//Cmd_Laser = 0;
					if(GoAhead_Distance > 0)
						{
							laser_open_close(0);
							 if(wheel_data_pid.car_len <= Rob_DimensionPara_float.Rob_WheelDistance_min-0.1)
							 	{
							 		car_stop();
							 		usleep(100000);
									Wheel_go_v0s(CarMode_GoBack, wheel_data_pid.last_p, 0.4, 0.5);
							 		return 1;
							 	}
							 else
								{
									//ROS_INFO("car_go_vxs  Yun Xing Kai Shi");
									car_go_vxs(CarMode_GoAhead, GoAhead_Distance, v2, 0.5);
									//ROS_INFO("car_go_vxs  Yun Xing Jie Shu");
									return 0;
								}
						}					
				}
			/*第三阶段 减速阶段*/
			//ti3=(t+t2-step):step:(2*t+t2);
			for(ti3 = (t+t2); ti3 <= (2*t+t2);)
				{
					//if(send_flag)
						{
							send_flag = 0;
							p3=a0+a1*pow((ti3-t2),1)+a2*pow((ti3-t2),2)+a3*pow((ti3-t2),3)+a4*pow((ti3-t2),4)+a5*pow((ti3-t2),5)+t2*vmax;
							v3=a1+2*a2*pow((ti3-t2),1)+3*a3*pow((ti3-t2),2)+4*a4*pow((ti3-t2),3)+5*a5*pow((ti3-t2),4);
							//acc3=2*a2+6*a3*pow((ti3-t2),1)+12*a4*pow((ti3-t2),2)+20*a5*pow((ti3-t2),3);  
							ti3 += SendV_Step;
							if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
								{
									ROS_INFO("Sport_control Timer Error");
								}
							sport_mode_analysis(sport_mode, v3);
							//ROS_INFO("Cmd_Laser = %d  speed = %8f",Cmd_Laser,v3);
						}
					GoAhead_Distance = laser_handle(Cmd_Laser, p3, &wheel_data_pid);
					//Cmd_Laser = 0;
					if(GoAhead_Distance > 0)
						{
							//laser_open_close(0);
							//ROS_INFO("向前行走%8f后停下来\r\n",GoAhead_Distance);
							if(wheel_data_pid.car_len <= Rob_DimensionPara_float.Rob_WheelDistance_min-0.1)
								{
									car_stop();car_stop();
									usleep(100000);
									Wheel_go_v0s(CarMode_GoBack, wheel_data_pid.last_p, 0.4, 0.5);
									return 1;
								}
							else
								{
									car_go_vxs(CarMode_GoAhead, GoAhead_Distance, v3, 0.5);
									return 0;
								}
						}	
				}
		}//sqrt((8192*amax*amax*amax)/(3645*p))
	else /*第二种情况：加速到某一速度（该速度低于或等于最大速度）后立即减速*/
		{
			a5 =(4*amax*pow(((8192*amax*amax*amax)/(3645*s)),0.5))/(75*s);
			a4 = -(64*amax*amax)/(135*s);
			a3 =pow(((8192*amax*amax*amax)/(3645*s)),0.5)/2;
			t =(405*s*pow(((8192*amax*amax*amax)/(3645*s)),0.5))/(512*amax*amax);

			/*加速与减速曲线*/
			for(ti = 0; ti < 2*t; )
				{
					//if(send_flag)
						{   
							send_flag = 0; 
							p=a0+a1*pow(ti,1)+a2*pow(ti,2)+a3*pow(ti,3)+a4*pow(ti,4)+a5*pow(ti,5);
							v=a1+2*a2*pow(ti,1)+3*a3*pow(ti,2)+4*a4*pow(ti,3)+5*a5*pow(ti,4);
							//acc=2*a2+6*a3*pow(ti,1)+12*a4*pow(ti,2)+20*a5*pow(ti,3); 
							sport_mode_analysis(sport_mode, v);
							ti += SendV_Step;
							if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
								{
									ROS_INFO("Sport_control Timer Error");
								}
							//ROS_INFO("Cmd_Laser = %d  speed = %8f",Cmd_Laser,v);
							//ROS_INFO   
						}
					GoAhead_Distance = laser_handle(Cmd_Laser, p, &wheel_data_pid);
					Cmd_Laser = 0;
					if(GoAhead_Distance > 0)
						{ 
							//laser_open_close(0);
							//ROS_INFO("向前行走%8f后停下来\r\n",GoAhead_Distance);
							if(wheel_data_pid.car_len <= Rob_DimensionPara_float.Rob_WheelDistance_min-0.1)
								{
									car_stop();car_stop();
									usleep(100000);
									Wheel_go_v0s(CarMode_GoBack, wheel_data_pid.last_p, 0.4, 0.5);
									return 1;   
								}
							else
								{
									car_go_vxs(CarMode_GoAhead, GoAhead_Distance, v, 0.5);
									return 0;
								}
						}
				}
		}   
}


void test_Sheng_Kai(void)
{
	CMC_SendData_05(0x0f,0x05,0x03);
	CMC_SendData_05(0x0f,0x05,0x06);
	usleep(100000);
	Wheel_go_v0s(CarMode_RearWheel_GoBack,(float)(wheel_data_pid.car_len - 2400)/1000,0.4,0.25);
	CMC_SendData_05(0x0f,0x05,0x04);
}

void test_Shou_Suo(void)
{
	u8 count = 0;
	
	CMC_SendData_05(0x0f,0x05,0x03);
	Wheel_go_v0s(CarMode_RaarWheel_GoAhead,(float)(wheel_data_pid.car_len - 2400)/1000,0.4,0.25);
	ROS_INFO("Shou Suo Han Shu Zhong :laser car len = %d",laser_car_len);
	if(laser_car_len >2650)
		{
			while(laser_car_len >2650)
				{
					Wheel_go_v0s(CarMode_RaarWheel_GoAhead,(float)(laser_car_len - 2400)/1000,0.4,0.25);
					ROS_INFO("Shou Suo Han Shu Zhong :laser car len = %d",laser_car_len);
					count ++;
					if((laser_car_len <=2650)||(count >3))
						{
							CMC_SendData_05(0x0f,0x05,0x05);
							CMC_SendData_05(0x0f,0x05,0x04);
							return ;
						}
				}
		} 
	ROS_INFO("Shou Suo Han Shu Zhong :laser car len = %d",laser_car_len);
	CMC_SendData_05(0x0f,0x05,0x05);
	CMC_SendData_05(0x0f,0x05,0x04);
}

void car_Sheng_Suo(u16  distance,  float Vmax, u8 mode)
{
	u16 Sd = distance; //目标距离
	u16 St = laser_car_len; //当前距离
	u16 S0 = laser_car_len; //初始位置
	float Vt = 0.0;
	float K = 0.0;
	u16 K_16 = 0;

	time_t begin; //实例化time_t结构    
	time_t now; //实例化time_t结构  
	time(&begin);   
	CMC_SendData_05(0x0f,0x05,0x03);
	CMC_SendData_05(0x0f,0x05,0x06);
	sleep(0.2);
	
	while(St != Sd)
		{
			time(&now);  
			if(now - begin > 30)
			{
				break;
			}
			if((fabs(Sd - laser_car_len) <= 2))
				{
					break;
				}
			if(fabs(S0 - Sd)== 0)
				break;
			K = (fabs(laser_car_len -Sd)/fabs(S0 - Sd));
			Vt = Vmax*K;
			//K = (float)K_16;
			// ROS_INFO("La Sheng:\nSt = %d\nSd = %d\nS0 = %d\n",St,Sd,S0);
			// ROS_INFO("La Sheng K_16 = %d",K_16);
			ROS_INFO("La Sheng laser_car_len = %d",laser_car_len);
			ROS_INFO("La Sheng K = %f",K);
			if(Vt > Vmax)
				{
					Vt = Vmax;
					ROS_INFO("La Sheng K = %8f",K);
				}
			if(mode == Mode_LaSheng)
				sport_mode_analysis(CarMode_RearWheel_GoBack, Vt);
			else
				{
					sport_mode_analysis(CarMode_RaarWheel_GoAhead, Vt);
				//	ROS_INFO("Xiao Che Shou Qi");
				}
			usleep(20000);
		//	ROS_INFO("La Sheng laser_car_len = %d",laser_car_len);
		//	ROS_INFO("La Sheng Vt = %8f",Vt);
		}
	car_stop();
	if(mode != Mode_LaSheng)
	{
		CMC_SendData_05(0x0f,0x05,0x04);
		CMC_SendData_05(0x0f,0x05,0x05);  
	}
}


/*******************************************************************
name	:wheel_aim
funtion	:轮胎对准
input	:
output	:成功返回0  失败返回1
*******************************************************************/
u8 wheel_aim(void) 
{
	u8 res = 0;
	float La_Sheng_size = 0.0;
	u16 la_Sheng_size_u16 = 0;

	car_GoAhead(0,1);
	sleep(0.2);
	car_stop();
	set_acc(100);
	laser_open_close(1);
	time_fd =timer_open();
	memset(wheel_time_buff,0x00,sizeof(wheel_time_buff));	
	
	res = car_go_v0s(CarMode_GoAhead, 5, 0.5, 0.5);
	if(res != 0)
	{
		ROS_INFO("Che Sheng Ce Liang Cuo Wu!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}
		
	ROS_INFO("Qian Lun Yi Dui Zhun");
	ROS_INFO("Kai Shi Dui Zhun Hou Lun");
	La_Sheng_size = wheel_data_pid.car_len-2.4;

	ROS_INFO("Qi Che Zhou Ju  float ==>>>>>>> = %f",wheel_data_pid.car_len);
	la_Sheng_size_u16 = La_Sheng_size*1000;
	
	ROS_INFO("La Sheng Ju Li ==>>>>>>> = %d",la_Sheng_size_u16);
	if(la_Sheng_size_u16 > 580)
		{
			laser_open_close(0);
			timer_close(time_fd);
			control_cycle=0;
			car_stop();  
			set_acc(8); 
			car_GoAhead(0,1);
			sleep(0.2);
			car_stop();
			return 1;
		}
	 if(res == 0)
	 	car_Sheng_Suo(la_Sheng_size_u16 - 110,0.15,Mode_LaSheng);	
	laser_open_close(0);
	timer_close(time_fd);
    control_cycle=0;
	car_stop();  
	set_acc(8); 
	car_GoAhead(0,1);
	sleep(0.2);
	car_stop();
	return res;
}




/*夹车测试程序*/
void tset_auto_lift_PID(void)
{
	u8 res = 0;
		
	res = wheel_aim();
	if (!res)
		{ //夹车
			car_lift();  
			sleep(7.5);
		}
}

  
/*******************************************************************
name	:car_go_vxs
funtion	:小车从初速度为x开始走一段距离并停下来
input	:sport_mode 小车运动模式
		:s 想要移动的距离
		:vmax 最大速度
		:amax 最大加速度 
output	:返回-1 :输入距离能正常走完
		:返回值> 0 :超出输入距离，返回超出的部分
*********************************************************************/   
float car_go_vxs(u8 sport_mode, float user_s, float v_now, float amax)
{   
	/*从初速度开始行驶一段距离*/
	float p = user_s;
	float vmax = v_now;
	//float amax=0.9;
	float step=0.015; 

	float a5,a4,a3,a2,a1,a0; 
	float t,t0,t2,ti,ti1,ti2;
	float pmax,p1,p2,p_over;
	float v,v1,v2;
	float acc,acc1,acc2;
	unsigned long data = 0;

	//u16 i = 0;

	//计算以最大加速度加速到最大速度的曲线，默认初始和结束的速度与加速度均为0
	a2=0;
	a1=0;
	a0=0;
	t0=0;
	a5 =(16*amax*amax*amax*amax)/(405*vmax*vmax*vmax);
	a4 =-(8*amax*amax*amax)/(27*vmax*vmax);
	a3 =(16*amax*amax)/(27*vmax);
	t =(3*vmax)/(2*amax);
	pmax=a0+a1*pow(2*t,1)+a2*pow(2*t,2)+a3*pow(2*t,3)+a4*pow(2*t,4)+a5*pow(2*t,5);


	//分类讨论
	//第一种情况：能够在该距离内减速到零
	if(p>=pmax/2)
		{
			//第一段匀速             
			t2=(p-pmax/2)/vmax; 
			//ti1=0:step:t2;
			for(ti1 = 0; ti1 < t2;)
				{
					//if(send_flag)
						{
							send_flag = 0; 
							p1= vmax*ti1;
							v1= vmax; 
							acc1=0;
							ti1 += step;
							//set_car_speed(v1);
							sport_mode_analysis(CarMode_GoAhead, v1);
							if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
								{
									ROS_INFO("Sport_control Timer Error");
								}
							ROS_INFO("Cmd_Laser = %d  1_1 xspeed = %8f",Cmd_Laser,v1);
							//ROS_INFO("%8f  %8f  %8f  \r\n",p1,v1,acc1);
						}
				}
			//第二段减速
			//ti2=t2:step:(t+t2);
			for(ti2 = t2; ti2 <(t+t2);)
				{
					//if(send_flag)
						{
							send_flag = 0; 
							p2=a0+a1*pow((ti2-t2+t),1)+a2*pow((ti2-t2+t),2)+a3*pow((ti2-t2+t),3)+a4*pow((ti2-t2+t),4)+a5*pow((ti2-t2+t),5)+t2*vmax-pmax/2;
							v2=a1+2*a2*pow((ti2-t2+t),1)+3*a3*pow((ti2-t2+t),2)+4*a4*pow((ti2-t2+t),3)+5*a5*pow((ti2-t2+t),4);
							acc2=2*a2+6*a3*pow((ti2-t2+t),1)+12*a4*pow((ti2-t2+t),2)+20*a5*pow((ti2-t2+t),3);
							ti2 += step;
							//set_car_speed(v2);
							sport_mode_analysis(CarMode_GoAhead, v2);
							if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
								{
									ROS_INFO("Sport_control Timer Error");
								}
							ROS_INFO("Cmd_Laser = %d  1_2_xspeed = %8f",Cmd_Laser,v2);
							//ROS_INFO("%8f  %8f  %8f  \r\n",p2,v2,acc2);
						}
				}
			ROS_INFO("car_go_vxs  Zheng Chang Tui Chu");
			return -1;
		}
	else //第二种情况：不能够在该距离内减速到零
		{
			p_over=pmax/2-p;
			//ti=0:step:t;
			for(ti = 0; ti < t;)
				{
					//if(send_flag)
						{
							send_flag = 0; 
							p=a0+a1*pow((ti+t),1)+a2*pow((ti+t),2)+a3*pow((ti+t),3)+a4*pow((ti+t),4)+a5*pow((ti+t),5)+t2*vmax-pmax/2;
							v=a1+2*a2*pow((ti+t),1)+3*a3*pow((ti+t),2)+4*a4*pow((ti+t),3)+5*a5*pow((ti+t),4);
							acc=2*a2+6*a3*pow((ti+t),1)+12*a4*pow((ti+t),2)+20*a5*pow((ti+t),3);
							ti += step;
							//set_car_speed(v); 
							sport_mode_analysis(CarMode_GoAhead, v);
							if(read(time_fd, &data, sizeof(long unsigned int)) <= 0)
								{
									ROS_INFO("Sport_control Timer Error");
								}
							ROS_INFO("Cmd_Laser = %d  2 xspeed = %8f",Cmd_Laser,v);  
							//ROS_INFO("%8f  %8f  %8f  \r\n",p,v,acc);
						}
				}
				ROS_INFO("速度过大而加速度过小，不能在短距离内停下！超出距离为%8f！/r/n",p_over);  
				return p_over;
		}
}




