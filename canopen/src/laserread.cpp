#include <string>
#include <ros/ros.h>    
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
using namespace std;
using namespace boost::asio;

int main(int argc, char** argv) 
{

    ros::init(argc, argv, "laserread");       //初始化节点
    ros::NodeHandle n;   
    ros::Rate loop_rate(20);

    io_service iosev_laser;
    serial_port laser(iosev_laser, "/dev/ttyUSB1"); 
    laser.set_option(serial_port::baud_rate(115200));   
    laser.set_option(serial_port::flow_control(serial_port::flow_control::none));
    laser.set_option(serial_port::parity(serial_port::parity::none));
    laser.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    laser.set_option(serial_port::character_size(8));

    unsigned char readCmd[8]={0x01,0x04,0x00,0x00,0x00,0x02,0x71,0xCB};
    unsigned char bufferRec[10]={0};

    while (ros::ok()) 
    {
        int wrttmp = write(laser,buffer(readCmd,8));
        int resTemp = read(laser, buffer(bufferRec, 9));
        int tmp = 0;
        tmp = (int)bufferRec[3]<<8;
        tmp += (int)bufferRec[4];
        float data = (float)tmp*10.0f/4095*5000/9972*1000+9;
        ROS_INFO("READ(%d-%d):-%d-%d data=%f",wrttmp,resTemp,bufferRec[3],bufferRec[4],data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;  
}


