/**
 * 小车控制主函数
 */
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sensor_msgs/Joy.h>       //手柄头文件
#include <stdio.h>    //usb_can函数
#include <unistd.h>
#include <asm/types.h>
#include <sys/socket.h>
#include <canopen/PCANBasic.h>
#include <canopen/resource.h>
#include <canopen/motorCAN.h>
#include <canopen/MOTEC_motorCtl.h>
#include <canopen/techMotorCtl.h> 
#include <canopen/sensor_cmc.h>
#include <canopen/joy_callback.h>
#include "canopen/sport_control.h"
#include "canopen/timer.h"
#include "canopen/auto_lift.h"
#include "canopen/motorMsg.h"
#include "canopen/gyroscope.h"

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作
#define PCAN_DEVICE		PCAN_USBBUS1
void mainTimerCallback(const ros::TimerEvent&);

void read_mote_current(void)
{
	current_read(1);
    current_read(2);
    current_read(3);
    current_read(4);
	current_read_TK(5);
    current_read_TK(6);
    current_read_TK(7);
    current_read_TK(8);
}

void read_moto_speed(void)
{
	AGV_SPEED_Enquire(1);
	AGV_SPEED_Enquire(2);
	AGV_SPEED_Enquire(3);
	AGV_SPEED_Enquire(4);
	AGV_SPEED_Enquire(5);
	AGV_SPEED_Enquire(6);
	AGV_SPEED_Enquire(7);
	AGV_SPEED_Enquire(8);
}



int main(int argc, char **argv)
{

    // ROS节点初始化
    ros::init(argc, argv, "car_control");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
    ros::Subscriber sub1 = n.subscribe("joy", 1, joyCallback);
    ros::Subscriber sub2 = n.subscribe("scanner", 1, scannerCallback);
    ros::Subscriber sub3 = n.subscribe("trace", 1, traceCallback);
    ros::Publisher pub1 =n.advertise<geometry_msgs::Twist>("agv_state", 1);
    ros::Publisher pub2 =n.advertise<canopen::motorMsg>("motor_msg", 1);
    // ros::Timer speedTimer = n.createTimer(ros::Duration(0.001),speedTimerCallback);
    // ros::Timer mainTimer = n.createTimer(ros::Duration(0.01),mainTimerCallback);
    // 设置循环的频率
    ros::Rate loop_rate(100);
    int i;
    int count = 0;
    int cmcReceive;
    int gyroscope_pthRes;
	int ask_pthRes;
    int once_flag = 0;
    int ShouSuo_flag = 0;
   
	timer_init();
    
    pthread_t cmc_id;
    pthread_t gyroscope_pthID;
    pthread_t AskSpeed_pthID;
    // cmcReceive=pthread_create(&cmc_id, NULL, CAN_pending, NULL);
    // Status = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);    //初始化usb-can
	//  gyroscope_init();
    //  gyroscope_pthRes=pthread_create(&gyroscope_pthID, NULL, gyroscope_pending, NULL);
    //  ask_pthRes=pthread_create(&AskSpeed_pthID, NULL, send_ask_speed, NULL);
    for(int i=1;i<5;i++)
    {
        Set_motor_acc(i , 20);
        Set_motor_decc_stop(i , 20);
    }
    printf("Initialize CAN: %x\n",(int)Status);
    motorCtlInit();   //电机初始化
    Stop_AGV();
    // laser_open_close(1);//
    sys_source_init();

            // tech_start_node(0x05);//------ceshi
        // tech_ready_switch_on(0x05);
        // tech_switch_on(0x05);
        // tech_enable_operation(0x05);
        // tech_set_control_mode(5,3);//初始化
        // tech_set_speed_mode(5, 0);
        // tech_start(5);
        
    
		Set_TKmotor_acc(6,2000000);
		Set_TKmotor_acc(8,2000000);
		Set_TKmotor_decc(6,2000000);
		Set_TKmotor_decc(8,2000000);
    while (ros::ok())
    {
        delay_us(100);

        ROS_ERROR("delay_sec test start :  100 us!");

        delay_us(10);

        ROS_ERROR("delay_sec test start :  10 us!");

        delay_sec(5);
        ROS_ERROR("delay_sec test start :  5 s!");
        // delay_sec(5);
        // ROS_INFO("delay_sec test end :  5 s!");

        geometry_msgs::Twist msg;
        msg.linear.x=laser_car_len+2400;
        msg.linear.y=AGVupdate_ready;
        pub1.publish(msg);
		
	//	printf("main:函数循环中\n");
         if(count>=10)
        {
            canopen::motorMsg motor;
            for(int k=0;k<8;k++)
            {
                motor.motorCurrent[k]=log_buff[k];
                motor.motorSpeed[k]=speed_buff[k];
            }
            pub2.publish(motor);

        //     while(i!=2 && once_flag==0)
        // {
        //     i=steering_motor_s_curve(0,-360,0,0,600,300,3000);
        //     if(i==2)
        //     {
        //         once_flag=1;
        //     }
                
        //     // ROS_INFO("====== i=%d  once=%d =====",i,once_flag);
        // }
            // max_current_read(1);
            //read_moto_speed();
            //read_mote_current();
			//TK_position_Enquire();
            // I2T_time_read(1);
			//ROS_INFO("wheel_angle:%f,%f,%f,%f",wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4);
            count=0;
        }
        count++; 

        //test_motor();
        if(!ShouSuo_flag)
        {
            //car_Sheng_Suo(0,0.15,0);
             ShouSuo_flag = 1;
        }
        if(mode_auto)   
        {
           
            //shift_correction_yTheta(0.2,0.04,50,3,1200,0);
           // Stop_AGV();
            // speed_test();
			#if 1
             //car_auto_run();
			//Speed_AGV(0.4);
			odometer_flag = 1;
            car_auto_run();
			//run_correction_xTheta2(0.2,-50,-5,1200,0);
			//Stop_AGV();
			#else
            if(once_flag==0)
            {
                Speed_AGV(0.4);
				gyroscope_cor_test();
                Speed_AGV(0);
                once_flag=1;
            }
			#endif
            // car_adjust_shift(0.2,20);
            // sleep(5);
            if(lift_cmd)
            {
                //car_lift();
                //car_lay_down();
                tset_auto_lift_PID();
                ROS_INFO("JIa Che Kai Shi +++++++++");
                 //wheel_aim();
                lift_cmd = 0;
            }
        }
		odometer_flag = 0;
        ros::spinOnce();
        // 按照循环频率延时
        loop_rate.sleep();
    }
	exit(0);
    return 0;
}

void mainTimerCallback(const ros::TimerEvent&)
{
   wheel_aim();
}

    // ROS_INFO("once_flag=%d",once_flag);
    // switch(once_flag){
    // case 0:
    //     Speed_AGV(0.2);
    //     sleep(1);
    //     once_flag=1;
    // break;
    // case 1:
    //     tech_turn_ex(5, 8);
    //     tech_turn_ex(6, 10);
    //     tech_turn_ex(7, -8);
    //     tech_turn_ex(8, -10);
    //     sleep(1);
    //     once_flag=2;
    // break;
    // case 2:
    //     if(speed_buff[4] < 1)
    //     {
    //         tech_turn_ex(5, 0);
    //         tech_turn_ex(6, 0);
    //         tech_turn_ex(7, 0);
    //         tech_turn_ex(8, 0);
    //         sleep(1);
    //         once_flag++;
    //     }  
    // break;
    // case 3:
    //     if(speed_buff[4] < 1)
    //     {
    //         once_flag++;
    //     } 
    // break;
    // case 4:
    //     Stop_AGV();
    //     once_flag++;
    // break;
    // }

