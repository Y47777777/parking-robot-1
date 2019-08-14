#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "canopen/scanner.h"
#include "canopen/socketcon.h"
#include "canopen/type.h"
using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

io_service iosev_bjf;
serial_port bjf(iosev_bjf, "/dev/bjf_serial");         //定义传输的串口 连接倍加福

int code_cur=0,xPosition_cur=0,yPosition_cur=0;//当前二维码信息
float angle_cur=0;
int orderNo_cur=0,pointNo_cur=0;//当前任务号，当前路径点号
int updateCode_ready=0;

int test1[][6]={
	{67,0,1200,4,200,0},{68,0,1200,0,200,0},
     {68,0,1200,1,200,0},{67,0,1200,0,200,0},
}; //测试路径往返信息
int test_num=2; //单条路径二维码数目
int test=255; 
                
//code;
//direction;
//length;
//run_mode;
//speed;
//reserve;

//----倍加福初始化函数
//--初始化成功返回0;失败返回1
u8 scanner_initial(void)
{
    unsigned char unitialBF[2]={0xE4,0x1B};//初始化命令
    unsigned char bufferRecBFTemp[10]={0};

    bjf.set_option(serial_port::baud_rate(115200));   
    bjf.set_option(serial_port::flow_control(serial_port::flow_control::none));
    bjf.set_option(serial_port::parity(serial_port::parity::even));
    bjf.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    bjf.set_option(serial_port::character_size(8));
    
    write(bjf,buffer(unitialBF,2));
    ROS_INFO("Scanner initial!");

    int resTemp=read(bjf, buffer(bufferRecBFTemp, 3));
    if(bufferRecBFTemp[0]==0x01)
    {
        ROS_INFO("Scanner initial success!");
        return 0;
    }
    else
    {
        ROS_INFO("Scanner initial wrong!");
        return 1;
    }
    
}


//----倍加福初始化函数
//--初始化成功返回0;失败返回1
void scanner_data_process(void)
{
    unsigned char bufferToBF[2]={0xC8,0x37}; //读取命令
    int nRecBF=0, nWriteToBF=0; //从BF读到多少字节、发送给BF多少字节
    unsigned char bufferRecBF[30] ;//暂存倍加福数据
    float theta=0;
    int tempControl,temp0;
    int code=0,xPosition=0,yPosition=0;
    static int code_bf=0,orderNo_bf=0,pointNo_bf=0;
    static int new_trace=0;

    //--发送命令读取数据
    if(nWriteToBF=write(bjf,buffer(bufferToBF,2))!=2)
    {
        ROS_INFO("Scanner data request failed!");
    }
    usleep(10000);
    nRecBF=read(bjf,buffer(bufferRecBF, 21));
    if(nRecBF!=21)
    {
        return;
    }
    bufferRecBF[nRecBF]='\0';
//     ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",bufferRecBF[0],bufferRecBF[1],bufferRecBF[2],
  //   bufferRecBF[3],bufferRecBF[4],bufferRecBF[5],bufferRecBF[6],bufferRecBF[7],bufferRecBF[8],bufferRecBF[9],bufferRecBF[10],
    // bufferRecBF[11],bufferRecBF[12],bufferRecBF[13],bufferRecBF[14],bufferRecBF[15],bufferRecBF[16],bufferRecBF[17],bufferRecBF[18],
     //bufferRecBF[19],bufferRecBF[20]);

    //--角度数据解析，由0-360转为-180-180
    theta=(bufferRecBF[10]*0x80+bufferRecBF[11])/10.0f;
    if(theta>270 && theta<=360)
    {
        theta=theta-360-90;
    }
    else if(theta>=0&&theta<=270)
    {
        theta=theta-90;
    }

    //--转译为xPosition和yPosition
    //已经验证16进制乘法和uint_8转换为int这个过程都没问题
    //--Y Position
    if((bufferRecBF[6] & 64)!=0)
    {
        short tempY=(short)bufferRecBF[6]<<8;//Left
        tempY+=bufferRecBF[7];
        tempY-=1;
        yPosition=~tempY;
        yPosition=yPosition & 32639;
        yPosition=yPosition*(-1);
    }
    else
    {
        short tempY=(short)bufferRecBF[6]<<8;
        tempY+=bufferRecBF[7];
        yPosition=tempY;//Left
    }

    //--X Position
    if((bufferRecBF[2] & 4)!=0)
    {
        int tempX=(int)bufferRecBF[2]<<24;
        tempX+=(int)bufferRecBF[3]<<16;
        tempX+=(int)bufferRecBF[4]<<8;
        tempX+=(int)bufferRecBF[5];
        tempX-=1;
        xPosition=~tempX;
        xPosition=xPosition & 58687359;
        xPosition=xPosition*(-1);
    }
    else
    {
        int tempX=(int)bufferRecBF[2]<<24;
        tempX+=(int)bufferRecBF[3]<<16;
        tempX+=(int)bufferRecBF[4]<<8;
        tempX+=(int)bufferRecBF[5];
        xPosition=tempX;
    }

    //--转译出码本身的内容,这部分里面对颜色码带的识别是魏工写的
    if(bufferRecBF[0]==0x00  &&  bufferRecBF[1]== 69 )   //二维码
    {
        tempControl=(short)bufferRecBF[16]<<8;
        tempControl+=bufferRecBF[17];
        code=tempControl;    
    }
    else if(bufferRecBF[0]==0x04 && bufferRecBF[1]==0x05)   //码带
    {
        code=998;//我们自己把颜色码带定义其控制码为98
        temp0=xPosition;
        xPosition=-yPosition;
        yPosition=temp0;
    }
    else if(bufferRecBF[0] == 12 && bufferRecBF[1]==0x05)   //控制码
    {
        //tempControl=(short)bufferRecBF[14]<<8;
        tempControl=bufferRecBF[15];
        code=tempControl+1000; //控制码带加1000
        temp0=xPosition;
        xPosition=yPosition;
        yPosition=temp0;
        
    }
    else if(bufferRecBF[0]==0x02 &&(bufferRecBF[4]!=0 ||bufferRecBF[5]!=0 ||bufferRecBF[6]!=0 ||bufferRecBF[7]!=0 ))//色带
    {
        code=999; //自定义999为颜色
        xPosition=yPosition;
    }
    else if(bufferRecBF[0]==0x02 && bufferRecBF[1]==0x05)
    {
        code=0;
    }

    //--数据解析完毕之后赋值给全局变量
    code_cur=code;
    xPosition_cur=xPosition;
    yPosition_cur=yPosition;
    angle_cur=theta;
    // if(fabs(angle_cur)<170 && code_cur==998)
    ROS_INFO("==== code:%d ---xPosition:%f ---yPosition:%f ---angle:%f ====",code_cur,xPosition_cur*1.251,yPosition_cur*1.251,angle_cur);

    static int next=0;//测试固定路径使用
    if(!next)
    {
            for(int i=0;i<test_num;i++)
            {
                if(code_cur==test1[i][0])
                {
                    test=i;
					#if 1
                    if(i >= (test_num-1))
                    next=1;
					#endif
                }
            }
    }
    else
    {
        for(int i=test_num;i<2*test_num;i++)
        {
            if(code_cur==test1[i][0])
            {
                test=i;
                if(i >= (2*test_num-1))
                next=0;
            }
        }
    }

    ROS_INFO("pointNo_cur:%d orderNo_cur:%d AGV_run[orderNo_cur].point:%d run_mode:%d",
            pointNo_cur,orderNo_cur,AGV_run[orderNo_cur].point,AGV_run[orderNo_cur].Trace_point[pointNo_cur].run_mode);
}
        








