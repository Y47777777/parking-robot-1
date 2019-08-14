#include "canopen/lowerpc.h"
#include "canopen/socketcon.h"
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include "ros/ros.h"
using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

io_service iosev_lowPC;
serial_port lowPC(iosev_lowPC, "/dev/lowerpc");         //定义传输的串口 连接下位机
int update_ready=0;


void lowerpc_initial(void)
{
  lowPC.set_option(serial_port::baud_rate(115200));   
  lowPC.set_option(serial_port::flow_control(serial_port::flow_control::none));
  lowPC.set_option(serial_port::parity(serial_port::parity::none));
  lowPC.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  lowPC.set_option(serial_port::character_size(8));
  ROS_INFO("Lowerpc_connect!");
}


//--下发到下位机,上传到调度中心
//--此时要做一个基本的校验，防止极端数据
//--14位分别是：开始0xFF，x方向，x绝对值，y方向，y绝对值，角度方向，角度整数，角度小数，码的内容，运动模式，车头朝向，路径长度,速度,结束0xFE
void lowerpc_data_transmission(int code,int xPosition,int yPosition,float angleValue)
{
  unsigned char bufferInt[14]={0};//下发数组
  int index=0;
  if(code>=0&&code<=990)
  {
      int xtemp=code%100;
      int ytemp=code/100;
      index=(char)xtemp+(char)ytemp*16;
  }
      //--计算buffer[0]-[4]的内容
      bufferInt[0]=0xFF;
      if(xPosition>=0)
          bufferInt[1]=0;
      else
          bufferInt[1]=1;
      bufferInt[2]=abs((int)xPosition);

      if(yPosition>=0)
          bufferInt[3]=0;
      else
          bufferInt[3]=1;
      bufferInt[4]=abs((int)yPosition);

      //--计算buffer[5]-[7]，即角度相关
      int angleInt=(int)angleValue;//zhengshu
      bufferInt[7]=abs((int)(angleValue*10-angleValue*10));//xiaoshu
      bufferInt[6]=abs(angleInt);
      if(angleInt>=0)
          bufferInt[5]=0;//theta fangxiang
      else
          bufferInt[5]=1;

      //--计算buffer[8]-[10]
      bufferInt[8]=index;//printf("%x\n", bufferInt[6]);
      bufferInt[9]=AGV_run[orderNo_cur].Trace_point[pointNo_cur].run_mode;
      bufferInt[10]=(AGV_task[orderNo_cur][pointNo_cur].PGoAngle+180)/180;
      bufferInt[11]=AGV_run[orderNo_cur].Trace_point[pointNo_cur].length/10;
      bufferInt[12]=AGV_task[orderNo_cur][pointNo_cur].PSpeed/10;
      bufferInt[13]=0xFE;

      //下发倍加福数据
      write(lowPC,buffer(bufferInt,14));
      
      ROS_INFO("TO lowerPC：%d %d %d %d %d %d %d %d %d %d %d %d %d %d",bufferInt[0],bufferInt[1],bufferInt[2],bufferInt[3],bufferInt[4],
      bufferInt[5],bufferInt[6],bufferInt[7],bufferInt[8],bufferInt[9],bufferInt[10],bufferInt[11],bufferInt[12],bufferInt[13]);
  }
}

