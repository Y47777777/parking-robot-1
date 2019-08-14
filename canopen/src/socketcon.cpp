#include <sys/socket.h>
#include <sys/ioctl.h> 
#include <unistd.h> 
#include <netdb.h> 
#include <netinet/in.h>   
#include <arpa/inet.h>  
#include <string.h>
#include "ros/ros.h"
#include "canopen/socketcon.h"
#include "canopen/scanner.h"
#include "canopen/type.h"
#include "canopen/lowerpc.h"
#include <geometry_msgs/Twist.h>

#define MY_PORT 11111
#define REMOTE_PORT 22222
struct sockaddr_in server_addr;  //**服务器
struct sockaddr_in clientAddr;   //**Remote客户端
int addr_len = sizeof(clientAddr);
int serverSocket;   //**服务器
const char* g_remoteIP="192.168.100.35";//192.168.8.100
int RecvFromServer;  //**Remote客户端


AGV_Task AGV_task[20][100];//最多缓存20条路径*100个点
AGV_RUN AGV_run[20];//最多缓存20条路径
int receive_num=10240*1024;
char recvbuf[10240*1024] = { 0 };
char Running_mode[20][97]={0};//用来保存对应命令的各个点的指令
int task = 0;
int receiveTrace_ready=0;//
int AGVupdate_ready=0;//AGV动作完成反馈标志

int power=0;
int WheelSpacing=0;

char get_data[18][20] = 
{"<Tno>",
"<ToPriority>",
"<Tasktype>",
"<Tono>",
"<AGVno>",
"<WheelSpacing>",
"<Lno>",
"<Plength>",
"<PSpeed>",
"<PAGVAngle>",
"<PGoAngle>",
"<PCarAngle>",
"<PAction>",
"<Ptno>",
"<Ptono>",
"<Topst>",
"<Topet>",
"</AGVCOMMAND>"
};


void agvCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
        static int updateConfirm=0;
        WheelSpacing=msg->linear.x;
        if(!updateCode_ready)//二维码更新后接收到AGVupdate_ready三次以上确定agv动作确实完成，可以向服务器更新数据
        {
                AGVupdate_ready=0;
                updateConfirm=0;
        }
        else if(updateCode_ready && updateConfirm>3)
        {
                AGVupdate_ready=msg->linear.y;
        }
        else if(updateCode_ready && msg->linear.y )
        {
                updateConfirm++;
        }
        
        // ROS_INFO("WheelSpacing=%d",WheelSpacing);
}


//连接服务器，接收路径信息
void server_connect_receive(void)
{
        if((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
                printf("RevSocket initial error!");
        }
        bzero(&server_addr, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(MY_PORT);//INADDR_ANY);    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        // 设置套接字选项避免地址使用错误
        int on=1;
        if((setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on)))<0)
        {
                printf("Setsockopt failed!");
                exit(EXIT_FAILURE);
        }

        if(bind(serverSocket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
                printf("Connect failed!");
        }
        if(listen(serverSocket, 5) < 0)
        {
                printf("Listen failed!");
        }
                printf("Listening\n");
        
        pthread_t serverRCV;
        int rcReceive=pthread_create(&serverRCV, NULL, DecodeThread, NULL);//新建一个线程，在这个线程里面等待接收调度中心的任务

}

void server_connect_send(void)
{
        pthread_t serverSEND;
        int rcReceive=pthread_create(&serverSEND, NULL, update_to_server, NULL);//新建一个线程，在这个线程里面等待接收调度中心的任务
}

void *update_to_server(void *)
{
        char sendbuf[300]={0};

        while(1)
        {
                 if(receiveTrace_ready && updateCode_ready && AGVupdate_ready)
                {
                        struct sockaddr_in serverAddr;   //**客户端
                        int SendToServer;   //**客户端

                        //客户端socket初始化//
                        ///定义clientSocket
                        if((SendToServer = socket(AF_INET,SOCK_STREAM, 0))<0)
                        {
                                ROS_INFO("SendSocket initial error!");
                        }
                        serverAddr.sin_family = AF_INET;
                        serverAddr.sin_port = htons(REMOTE_PORT);  ///服务器端口
                        serverAddr.sin_addr.s_addr = inet_addr(g_remoteIP);  ///服务器ip

                        ///连接服务器，成功返回0，错误返回-1
                        if (connect(SendToServer, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0 )
                        {
                                printf("Connect to %s:%d failed ...\n" , g_remoteIP , REMOTE_PORT) ;
                                pthread_exit(0) ;
                        }
                        int j=sprintf(sendbuf,
                        "<reportAgvLocation><agvNo>XYAGV1</agvNo><Power>%d</Power><wheelSpacing>%d</wheelSpacing><orderNo>%s</orderNo><lno>%d</lno><speed>%d</speed><angle>%d</angle><status>%s</status></reportAgvLocation>eof",
                        power,WheelSpacing,AGV_task[orderNo_cur][pointNo_cur].Tono,AGV_task[orderNo_cur][pointNo_cur].Lno,
                        AGV_task[orderNo_cur][pointNo_cur].PSpeed,AGV_task[orderNo_cur][pointNo_cur].PGoAngle,AGV_task[orderNo_cur][pointNo_cur].PAction);
                        // if(pointNo_cur >= (AGV_run[orderNo_cur].point-1))
                        // {
                        //         int order_temp=(order_temp)<0?20:(order_temp);
                        //        int j=sprintf(sendbuf,
                        //         "<reportAgvLocation><agvNo>XYAGV1</agvNo><Power>%d</Power><wheelSpacing>%d</wheelSpacing><orderNo>%s</orderNo><lno>%d</lno><speed>%d</speed><angle>%d</angle><status>%s</status></reportAgvLocation>eof",
                        //         power,WheelSpacing,AGV_task[order_temp][pointNo_cur].Tono,AGV_task[order_temp][pointNo_cur].Lno,
                        //         AGV_task[order_temp][pointNo_cur].PSpeed,AGV_task[order_temp][pointNo_cur].PGoAngle,AGV_task[order_temp][pointNo_cur].PAction); 
                        // }
                        ROS_INFO("%s",sendbuf);
                        int temp=send(SendToServer,sendbuf,j,0);
                        ROS_INFO("SEND :%d ",temp);
                        j=0;
                        updateCode_ready=0;
                        close(SendToServer);
                }
        }
}



int PAGVAngle_bf=0;
void DecodeCompute(int task_num, int point)
{
    static int first_time=0;
    memset(&AGV_run[task_num], 0, sizeof(AGV_RUN)*1);
    if(first_time == 0)//首次接收路径时默认当前路径角度就是agv上次路径角度
    {
        PAGVAngle_bf = AGV_task[task_num][0].PAGVAngle;
        first_time=1;
    }
    AGV_run[task_num].point = point;
    for (int i = 0; i<point; i++)
    {
                AGV_run[task_num].Trace_point[i].code = AGV_task[task_num][i].Lno;//二维码
                AGV_run[task_num].Trace_point[i].direction = AGV_task[task_num][i].PAGVAngle; //车头朝向
                AGV_run[task_num].Trace_point[i].length = AGV_task[task_num][i].Plength;//到下一点的路径长度
        if (AGV_task[task_num][i].PAction[0] == 'S'
                &&AGV_task[task_num][i].PAction[1] == 't')//判断该点动作：停止
        {
                        AGV_run[task_num].Trace_point[i].run_mode = 0;
        }
        else if (AGV_task[task_num][i].PAction[0] == 'G'
                &&AGV_task[task_num][i].PAction[1] == 'o')//判断该点动作：行驶
        {
                if (PAGVAngle_bf == AGV_task[task_num][i].PAGVAngle)//判断该点是否需要转弯：对比上次AGV角度与本次AGV角度，相等则直行
                {
                        if (AGV_task[task_num][i].PAGVAngle == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相等直线前行
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 1;//直线前进
                        }
                        else if (AGV_task[task_num][i].PAGVAngle + 90 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle - 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差-90或270则左横移
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 2;//左横移
                        }
                        else if (AGV_task[task_num][i].PAGVAngle - 90 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle + 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差90或-270则右横移
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 3;//右横移
                        }
                        else if (AGV_task[task_num][i].PAGVAngle + 180 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle - 180 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差180或-180则直线后退
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 4;//直线后退
                        }
                }
                else if (PAGVAngle_bf + 90 == AGV_task[task_num][i].PAGVAngle
                        || PAGVAngle_bf - 270 == AGV_task[task_num][i].PAGVAngle)//判断该点是否需要转弯：对比上次AGV角度与本次AGV角度，相差-90或270则逆时针转
                {
                        if (AGV_task[task_num][i].PAGVAngle == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相等直线前行
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 5;//停止 逆时针转 直线前进
                        }
                        else if (AGV_task[task_num][i].PAGVAngle + 90 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle - 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差-90或270则左横移
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 6;//停止 逆时针转 左横移
                        }
                        else if (AGV_task[task_num][i].PAGVAngle - 90 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle + 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差90或-270则右横移
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 7;//停止 逆时针转 右横移
                        }
                        else if (AGV_task[task_num][i].PAGVAngle + 180 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle - 180 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差180或-180则直线后退
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 8;//停止 逆时针转 直线后退
                        }
                }
                else if (PAGVAngle_bf - 90 == AGV_task[task_num][i].PAGVAngle
                        || PAGVAngle_bf + 270 == AGV_task[task_num][i].PAGVAngle)//判断该点是否需要转弯：对比上次AGV角度与本次AGV角度，相差-90或270则顺时针转
                {
                        if (AGV_task[task_num][i].PAGVAngle == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相等直线前行
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 9;//停止 顺时针转 直线前进
                        }
                        else if (AGV_task[task_num][i].PAGVAngle + 90 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle - 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差-90或270则左横移
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 10;//停止 顺时针转 左横移
                        }
                        else if (AGV_task[task_num][i].PAGVAngle - 90 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle + 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差90或-270则右横移
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 11;//停止 顺时针转 右横移
                        }
                        else if (AGV_task[task_num][i].PAGVAngle + 180 == AGV_task[task_num][i].PGoAngle
                                || AGV_task[task_num][i].PAGVAngle - 180 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差180或-180则直线后退
                        {
                                        AGV_run[task_num].Trace_point[i].run_mode = 12;//停止 顺时针转 直线后退
                        }
                }
        }
        else if (AGV_task[task_num][i].PAction[0] == 'L'
                && AGV_task[task_num][i].PAction[4] == 'G')//判断该点动作：举升或夹车
        {
                if (AGV_task[task_num][i].PAGVAngle == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相等直线前行
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 13;//停止 举升或夹车 直线前进
                }
                else if (AGV_task[task_num][i].PAGVAngle + 90 == AGV_task[task_num][i].PGoAngle
                        || AGV_task[task_num][i].PAGVAngle - 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差-90或270则左横移
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 14;//停止 举升或夹车 左横移
                }
                else if (AGV_task[task_num][i].PAGVAngle - 90 == AGV_task[task_num][i].PGoAngle
                        || AGV_task[task_num][i].PAGVAngle + 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差90或-270则右横移
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 15;//停止 举升或夹车 右横移
                }
                else if (AGV_task[task_num][i].PAGVAngle + 180 == AGV_task[task_num][i].PGoAngle
                        || AGV_task[task_num][i].PAGVAngle - 180 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差180或-180则直线后退
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 16;//停止 举升或夹车 直线后退
                }
        }
        else if (AGV_task[task_num][i].PAction[0] == 'L'
                && AGV_task[task_num][i].PAction[4] == 'S')//判断该点动作：举升或夹车
        {
                        AGV_run[task_num].Trace_point[i].run_mode = 17;//停止 举升或夹车
        }
        else if (AGV_task[task_num][i].PAction[0] == 'U'
                && AGV_task[task_num][i].PAction[6] == 'G')//判断该点动作：卸载
        {
                if (AGV_task[task_num][i].PAGVAngle == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相等直线前行
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 18;//停止 卸载 直线前进
                }
                else if (AGV_task[task_num][i].PAGVAngle + 90 == AGV_task[task_num][i].PGoAngle
                        || AGV_task[task_num][i].PAGVAngle - 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差-90或270则左横移
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 19;//停止 卸载 左横移
                }
                else if (AGV_task[task_num][i].PAGVAngle - 90 == AGV_task[task_num][i].PGoAngle
                        || AGV_task[task_num][i].PAGVAngle + 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差90或-270则右横移
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 20;//停止 卸载 右横移
                }
                else if (AGV_task[task_num][i].PAGVAngle + 180 == AGV_task[task_num][i].PGoAngle
                        || AGV_task[task_num][i].PAGVAngle - 180 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差180或-180则直线后退
                {
                                AGV_run[task_num].Trace_point[i].run_mode = 21;//停止 卸载 直线后退
                }
        }
        else if (AGV_task[task_num][i].PAction[0] == 'U'
        && AGV_task[task_num][i].PAction[6] == 'S')//判断该点动作：卸载
        {
                        AGV_run[task_num].Trace_point[i].run_mode = 22;//停止 卸载
        }
        else if(AGV_task[task_num][i].PAction[0] == 'M'
        && AGV_task[task_num][i].PAction[7] == 'L')
        {
            if (AGV_task[task_num][i].PAGVAngle == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相等直线前行
            {
                            AGV_run[task_num].Trace_point[i].run_mode = 23;//测量 提升 直线前进
            }
            else if (AGV_task[task_num][i].PAGVAngle + 90 == AGV_task[task_num][i].PGoAngle
                    || AGV_task[task_num][i].PAGVAngle - 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差-90或270则左横移
            {
                            AGV_run[task_num].Trace_point[i].run_mode = 24;//测量 提升 左横移
            }
            else if (AGV_task[task_num][i].PAGVAngle - 90 == AGV_task[task_num][i].PGoAngle
                    || AGV_task[task_num][i].PAGVAngle + 270 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差90或-270则右横移
            {
                            AGV_run[task_num].Trace_point[i].run_mode = 25;//测量 提升 右横移
            }
            else if (AGV_task[task_num][i].PAGVAngle + 180 == AGV_task[task_num][i].PGoAngle
                    || AGV_task[task_num][i].PAGVAngle - 180 == AGV_task[task_num][i].PGoAngle)//判断该点是否需要横行：对比AGV角度与行驶角度，相差180或-180则直线后退
            {
                            AGV_run[task_num].Trace_point[i].run_mode = 26;//测量 提升 直线后退
            }
        }
        else if(AGV_task[task_num][i].PAction[0] == 'C'
        && AGV_task[task_num][i].PAction[4] == 'g')
        {
                        AGV_run[task_num].Trace_point[i].run_mode = 27;//充电
        }
        AGV_run[task_num].Trace_point[i].speed = AGV_task[task_num][i].PSpeed;
        AGV_run[task_num].Trace_point[i].reserve= 0;
        PAGVAngle_bf = AGV_task[task_num][i].PAGVAngle;
        ROS_INFO("code::%d---PAGVAngle::%d---Plength::%d---Pmode::%d---Pspeed::%d",
                AGV_run[task_num].Trace_point[i].code,
                AGV_run[task_num].Trace_point[i].direction,
                AGV_run[task_num].Trace_point[i].length,
                AGV_run[task_num].Trace_point[i].run_mode,
                AGV_run[task_num].Trace_point[i].speed);
    }
}


void *DecodeThread(void *)
{
    while(1)
    {
        //--在解码之前先接收一次数据
        ROS_INFO("Waiting for server!");
        memset(recvbuf,0,receive_num);
        RecvFromServer = accept(serverSocket, (struct sockaddr*)&clientAddr, (socklen_t*)&addr_len);
        printf("Listening on port: %d\n", MY_PORT);
        int iDataNum = 0;
        while(strstr(recvbuf,"</AGVCOMMAND>")==NULL)
        {
                int num=recv(RecvFromServer, &recvbuf[iDataNum], receive_num, 0);
                iDataNum=iDataNum+num;
        }
        recvbuf[iDataNum]='\0';
        if (iDataNum > 0)
        {
                ROS_INFO("recv  %d  data\n", iDataNum);
                ROS_INFO("get data ------%s",recvbuf);
                int point = 0;
                int i = 0;
                int j = 0;
                char temp[10] = { 0 };
                int flag = 0 , flag_out = 1;
                char *recvbuf1,*recvbuf2;
                memset(AGV_task[task], 0, sizeof(AGV_Task)*100);
                while(flag_out)
                {
                         switch (flag)
                        {
                                
                                case 0://任务订单号
                                         {
                                                recvbuf1=strstr(recvbuf,get_data[flag]);
                                                i = 5;
                                                j = 0;
                                           //     ROS_INFO("Ton:%s",recvbuf1);
                                                while (recvbuf1[i] != '<')
                                                {
                                              //          ROS_INFO(" recvbuf1: %c", recvbuf1[i]);
                                                        AGV_task[task][point].Ton[j++] = recvbuf1[i++];
                                                }
                                                ROS_INFO("Ton:  %s ", AGV_task[task][point].Ton);
                                                flag++;
                                        }
                                        break;
                                case 1://任务优先级                                      
                                        {
                                                 recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("ToPriority:%s",recvbuf2);
                                                int m;
                                                i = 12;
                                                j = 0;
                                                char temp1[2];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                               // sscanf(temp1,"%d",&m);
                                                AGV_task[task][point].ToPriority =atoi(temp1);
                                                ROS_INFO("ToPriority:  %d ",AGV_task[task][point].ToPriority);
                                                flag++;
                                        }break;
                                case 2://任务类型
                                        {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 10;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].Tasktype[j++] = recvbuf1[i++];
                                                ROS_INFO("Tasktype:  %s ", AGV_task[task][point].Tasktype);         
                                                flag++;
                                        }break;
                                case 3://任务订单步骤
                                         {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 6;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].Tono[j++] = recvbuf1[i++];
                                                ROS_INFO("Tono:  %s ", AGV_task[task][point].Tono);         
                                                flag++;
                                               
                                        }break;
                                case 4://小车编号
                                        {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 7;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].AGVno[j++] = recvbuf1[i++];
                                                ROS_INFO("AGVno:  %s ", AGV_task[task][point].AGVno);                                     
                                                flag++;
                                        }break;
                                case 5://汽车前后轮距
                                       {
                                               recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("Lno:%s",recvbuf2);
                                                int m;
                                                i = 14;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                if(temp1[0]=='N'|| temp1[0]=='n')
                                                {
                                                        AGV_task[task][point].WheelSpacing=999;
                                                }
                                                else
                                                {
                                                     sscanf(temp1,"%d",&m);
                                                     AGV_task[task][point].WheelSpacing=m;   
                                                }
                                                ROS_INFO("WheelSpacing:%d ",AGV_task[task][point].WheelSpacing);
                                                flag++;    
                                       }break;           
                                case 6://位置点
                                       {
                                               recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("Lno:%s",recvbuf2);
                                                int m;
                                                i = 5;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                sscanf(temp1,"%d",&m);
                                                AGV_task[task][point].Lno =m;
                                                ROS_INFO("Lno:  %d ",AGV_task[task][point].Lno);
                                                flag++;
                                        }break;
                                case 7://下段路程长度
                                          {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("Plength:%s",recvbuf2);
                                                int m;
                                                i = 9;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                sscanf(temp1,"%d",&m);
                                                AGV_task[task][point].Plength =m;
                                                ROS_INFO("Plength:  %d ",AGV_task[task][point].Plength);
                                                flag++;
                                        }break;
                                case 8://计划速度
                                        {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("PSpeed:%s",recvbuf2);
                                                int m;
                                                i = 8;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                sscanf(temp1,"%d",&m);
                                                AGV_task[task][point].PSpeed =m;
                                                ROS_INFO("PSpeed:  %d ",AGV_task[task][point].PSpeed);
                                                flag++;
                                        }break;
                                case 9://计划AGV车头朝向
                                         {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("PAGVAngle:%s",recvbuf2);
                                                int m;
                                                i = 11;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                if(temp1[0]=='N' && temp1[1]=='U')
                                                {
                                                        AGV_task[task][point].PAGVAngle=999;
                                                }
                                                else
                                                {
                                                     sscanf(temp1,"%d",&m);
                                                     AGV_task[task][point].PAGVAngle=m;   
                                                }
                                                ROS_INFO("PAGVAngle:  %d ",AGV_task[task][point].PAGVAngle);
                                                flag++;
                                        }break;
                                case 10://计划AGV行驶方向
                                        {

                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("PAGVAngle:%s",recvbuf2);
                                                int m;
                                                i = 10;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                sscanf(temp1,"%d",&m);
                                                AGV_task[task][point].PGoAngle =m;
                                                ROS_INFO("PGoAngle:  %d ",AGV_task[task][point].PGoAngle);
                                                flag++;
                                        }break;
                                case 11://计划汽车车头朝向
                                         {

                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                             //   ROS_INFO("PAGVAngle:%s",recvbuf2);
                                                int m;
                                                i = 11;
                                                j = 0;
                                                char temp1[10];
                                                while (recvbuf1[i] != '<')
                                                        temp1[j++] = recvbuf1[i++];
                                                        temp1[j++]='\0';
                                                sscanf(temp1,"%d",&m);
                                                AGV_task[task][point].PCarAngle =m;
                                                ROS_INFO("PCarAngle:  %d ",AGV_task[task][point].PCarAngle);
                                                flag++;
                                        }break;
                                case 12://计划动作
                                               {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 9;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].PAction[j++] = recvbuf1[i++];
                                                ROS_INFO("PAction:  %s ", AGV_task[task][point].PAction);                                     
                                                flag++;
                                         }
                                        
                                        break;
                                case 13://前置任务订单号
                                         {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 6;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].Ptno[j++] = recvbuf1[i++];
                                                ROS_INFO("Ptno:  %s ", AGV_task[task][point].Ptno);                                                                       
                                                flag++;
                                        }break;
                                case 14://前置任务步骤号
                                         {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 7;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].Ptono[j++] = recvbuf1[i++];
                                                ROS_INFO("Ptono:  %s ", AGV_task[task][point].Ptono);                                     
                                                flag++;
                                        }break; 
                                case 15://任务步骤计划开始时间
                                        {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 7;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].Topst[j++] = recvbuf1[i++];
                                                ROS_INFO("Topst:  %s ", AGV_task[task][point].Topst);                                     
                                                flag++;
                                        }break;
                                case 16://任务步骤计划完成时间
                                        {
                                                recvbuf1=strstr(recvbuf1,get_data[flag]);
                                                i = 7;
                                                j = 0;
                                                while (recvbuf1[i] != '<')
                                                        AGV_task[task][point].Topet[j++] = recvbuf1[i++];
                                                ROS_INFO("Topet:  %s ", AGV_task[task][point].Topet); 
                                                if(strstr(recvbuf1,get_data[0]))    
                                                {
                                                        flag=0;
                                                        strcpy(recvbuf,recvbuf1);
                                                }                             
                                                else
                                                        flag_out = 0;
                                                point++;
                                                ROS_INFO("point___%d___%s-----",point,AGV_task[task][point].Topet);
                                                ROS_INFO("i==%d",i);
                                        }break;
                        }
                }         
                DecodeCompute(task,point);
                receiveTrace_ready=1;
                task = ++task > 19 ? 0 : task;
        }
    }
}
