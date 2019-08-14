#include <string>
#include <ros/ros.h>    
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <canopen/bms_com.h>
using namespace std;
using namespace boost::asio;

//--命令
unsigned char BMSProtectParaCMD[20] = ":000100000E09~";
unsigned char BMSStateMSGCMD[20] = ":000200000ee8~";
unsigned char BMSSetFETCMD[20] = ":000652001002AF~";
unsigned char BMSGetVersionCMD[20] = ":000900000ee1~";
unsigned char BMSGetCapacityCMD[20] = ":000900000ee1~";
unsigned char BMSGetSNdataCMD[20] = ":001200000ee7~";
unsigned char BMSGetPackSNdataCMD[20] = ":002200000ee6~";
unsigned char BMSGetCapacityPercentCMD[20] = ":001300000ee6~";
unsigned char BMSSetModeCMD[20] = ":001400000ee5~";
//--数据
BMSProtectParaData BMSProtectPara;
int BMSCapacityPercent=0;
//
int main(int argc, char** argv) 
{

    ros::init(argc, argv, "bmscom");       //初始化节点
    ros::NodeHandle n;   
    ros::Rate loop_rate(50);

    io_service iosev_bms;
    serial_port bms(iosev_bms, "/dev/ttyUSB0"); 
    bms.set_option(serial_port::baud_rate(9600));   
    bms.set_option(serial_port::flow_control(serial_port::flow_control::none));
    bms.set_option(serial_port::parity(serial_port::parity::none));
    bms.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    bms.set_option(serial_port::character_size(8));

    while (ros::ok()) 
    {
        unsigned char bufferRec[300]={0};
        int wrttmp = write(bms,buffer(BMSGetCapacityPercentCMD,15));
        int resTemp = read(bms, buffer(bufferRec, 16));
        ROS_INFO("READ(%d-%d):data = %s",wrttmp,resTemp,bufferRec);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;  
}

void BMSProtectParament_Process(unsigned char *databuf)
{
    BMSProtectPara.Addr = databuf[0];
}

void BMSGetCapacityPercent_Process(unsigned char *databuf)
{
    BMSProtectPara.Addr = databuf[10];
}

