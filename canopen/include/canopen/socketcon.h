#ifndef ___SCOKETCON_H___
#define ___SCOKETCON_H___

#include "canopen/type.h"
#include <geometry_msgs/Twist.h>

struct AGV_Task
{
        char Ton[10];//任务订单号
        int ToPriority;//任务优先级
        char Tasktype[10];//任务类型
        char Tono[10];//任务订单步骤
        char AGVno[10];//AGV号
        char Carno[10];//车牌号
        int WheelSpacing;//前后轮距
        int Lno;//二维码
        int Plength;//下段路径长度：ｍｍ
        int PSpeed;//速度：ｍｍ/s
        int PAGVAngle;//ＡＧＶ车头朝向
        int PGoAngle;//预计行驶方向
        int PCarAngle;//预计汽车车头朝向
        char PAction[20];//动作
        char Ptno[10];//前置任务订单号
        char Ptono[10];//前置任务步骤号
        char Topst[30];//预计开始时间
        char Topet[30];//预计完成时间
};

struct POINT_MSG
{
        int code;
        int direction;
        int length;
        int run_mode;
        int speed;
        int reserve;
};

struct AGV_RUN
{
        POINT_MSG Trace_point[100];//最多缓存100个点
        int point;//该条路径的二维码数
};

extern AGV_Task AGV_task[20][100];//最多缓存20条路径*100个点
extern AGV_RUN AGV_run[20];//最多缓存20条路径
extern int receiveTrace_ready;
extern int AGVupdate_ready;//AGV动作完成反馈标志

void agvCallback(const geometry_msgs::Twist::ConstPtr& msg);
void server_connect_receive(void);
void server_connect_send(void);
void *update_to_server(void *);
void DecodeCompute(int task_num, int point);
void *DecodeThread(void *);

#endif
