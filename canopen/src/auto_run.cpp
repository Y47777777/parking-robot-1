/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <string>
#include <ros/ros.h>                           // 包含ROS的头文件
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include "std_msgs/String.h"              //ros定义的String数据类型
#include <geometry_msgs/Twist.h>
#include <sys/socket.h>
#include "canopen/scanner.h"
#include "canopen/socketcon.h"
#include "canopen/lowerpc.h"

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

float last_orderNo_cur=0;
float last_pointNo_cur=0;

int main(int argc, char** argv) 
{

    ros::init(argc, argv, "auto_run");       //初始化节点
    ros::NodeHandle n;   
    ros::Publisher chatter_pub1 =n.advertise<geometry_msgs::Twist>("scanner", 1);
    ros::Publisher chatter_pub2 =n.advertise<geometry_msgs::Twist>("trace", 1);
    ros::Subscriber sub1 = n.subscribe("agv_state", 1, agvCallback);
    ros::Rate loop_rate(100);
    scanner_initial();  //扫码器初始化
    //lowerpc_initial();  //下位机初始化
    server_connect_receive();   //连接服务器接收路径
    server_connect_send();    //连接服务器更新agv状态
    ROS_INFO("11111");
    while (ros::ok()) 
    {
        scanner_data_process();
        //lowerpc_data_transmission(); 
        geometry_msgs::Twist msg;

        
            msg.linear.x=xPosition_cur;
            msg.linear.y=yPosition_cur;
            msg.linear.z=code_cur;
            msg.angular.x=angle_cur;
            msg.angular.y=last_orderNo_cur;
            msg.angular.z=last_pointNo_cur;//pointNo_cur;
            if(code_cur)
            {
                msg.angular.y=orderNo_cur;
                msg.angular.z=test>(test_num-1)?(test-test_num):test;//pointNo_cur;   
                last_orderNo_cur=msg.angular.y;
                last_pointNo_cur=msg.angular.z;
            }
            chatter_pub1.publish(msg);

        if(code_cur)
        {
            // msg.linear.x=AGV_run[orderNo_cur].Trace_point[pointNo_cur].code;
            // msg.linear.y=AGV_run[orderNo_cur].Trace_point[pointNo_cur].direction;
            // msg.linear.z=AGV_run[orderNo_cur].Trace_point[pointNo_cur].length;
            // msg.angular.x=AGV_run[orderNo_cur].Trace_point[pointNo_cur].run_mode;
            // msg.angular.y=AGV_run[orderNo_cur].Trace_point[pointNo_cur].speed;
            // msg.angular.z=AGV_run[orderNo_cur].Trace_point[pointNo_cur].reserve;
            msg.linear.x=test1[test][0];
            msg.linear.y=test1[test][1];
            msg.linear.z=test1[test][2];
            msg.angular.x=test1[test][3];
            msg.angular.y=test1[test][4];
            msg.angular.z=test1[test][5];
             ROS_INFO("---code:%d direction:%d length:%d run_mode:%d speed:%d---",msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y);
            chatter_pub2.publish(msg);   //发布消息
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
//    iosev_bjf.run(); 
    return 0;  
}





