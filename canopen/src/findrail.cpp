#include <memory.h>
#include <string.h>
#include <ros/ros.h>                           // 包含ROS的头文件
#include "std_msgs/String.h"              //ros定义的String数据类型
#include <sensor_msgs/LaserScan.h>
#include "canopen/findrail.h"
#include "canopen/type.h"
#include "math.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#define pi 3.1415

laser_scan_data *data_cur,timscan;;
int lms_receive=0,sick_receive=0;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "findrail");       //初始化节点
    ros::NodeHandle n;   
    ros::Publisher laser_pub =n.advertise<sensor_msgs::LaserScan>("findrail", 1);
    ros::Subscriber sub1 = n.subscribe("sick_scan", 1, sickCallback);
    ros::Rate loop_rate(100);
    while (ros::ok()) 
    {
        //ROS_INFO("wait");
        float offset=0,height=0;
        if(sick_receive)
        {
            data_cur = & timscan;
            findrail(&offset,&height);
            ros::Time scan_time = ros::Time::now();
            sensor_msgs::LaserScan msg;
            msg.header.stamp = scan_time;
            msg.header.frame_id = "laser";
            msg.angle_min=data_cur->angle_min;
            msg.angle_max=data_cur->angle_max;
            msg.angle_increment=data_cur->angle_increment; //测量的角度间的距离
            msg.time_increment=data_cur->time_increment; //测量间的时间
            msg.scan_time=data_cur->scan_time; //扫描间的时间
            msg.range_min=data_cur->range_min; //最小测量距离
            msg.range_max=data_cur->range_max; //最大测量距离
            msg.ranges.resize(811);
            msg.intensities.resize(811);
            for(int i=0;i<811;i++)
            {
                msg.ranges[i]=data_cur->ranges[i]; //测量的距离数据
                msg.intensities[i]=data_cur->intensities[i]; ; //强度数据
            }
            laser_pub.publish(msg);
            sick_receive=0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;  
}

void sickCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    timscan.angle_min=(float)msg->angle_min; //开始角度
    timscan.angle_max=(float)msg->angle_max; //结束角度
    timscan.angle_increment=(float)msg->angle_increment; //测量的角度间的距离
    timscan.time_increment=(float)msg->time_increment; //测量间的时间
    timscan.scan_time=(float)msg->scan_time; //扫描间的时间
    timscan.range_min=(float)msg->range_min; //最小测量距离
    timscan.range_max=(float)msg->range_max; //最大测量距离
    timscan.data_num=(int)((timscan.angle_max-timscan.angle_min)/timscan.angle_increment+1);
    for(int i=0;i<timscan.data_num;i++)
    {
        timscan.ranges[i]=(float)msg->ranges[i]; //测量的距离数据
        timscan.intensities[i]=(float)msg->intensities[i]; //强度数据
    }
    ROS_INFO("sickCallback: receive %d data",timscan.data_num);
    sick_receive=1;
}


void findrail(float *offset,float *height)
{
    ROS_INFO("findrail");
    int pfile = 0;
    int data_num=data_cur->data_num;
    float angle[data_num]={},dis[data_num]={},x[data_num]={},y[data_num]={},x_aver=0,y_aver=0;
    static float off[11]={};static int record=0;
    
    //计算宽度范围
    float max_limit = 0;
    for(int i=306; i<506; i++){
        max_limit += atan(0.4 / (data_cur->ranges[i]*cos(data_cur->angle_min+data_cur->angle_increment*i)));
    }
    max_limit /= 200;
    int start = floor((-max_limit-data_cur->angle_min)/data_cur->angle_increment)+1;
    int end = floor((max_limit-data_cur->angle_min)/data_cur->angle_increment);
    //ROS_INFO("findrail");

    //--原始数据处理
    if((pfile=open("./laserdata/data.txt",O_RDWR|O_CREAT|O_APPEND))<0){
        ROS_WARN("ERROR:Can not open ~/laserdata/data.txt!");
    }
    for(int i=0;i<data_num;i++){
        angle[i] = data_cur->angle_min+data_cur->angle_increment*i;//单位为弧度
        dis[i] = data_cur->ranges[i];//单位为毫米
        if(angle[i]<max_limit && angle[i]>-max_limit){
            x[i] = dis[i]*sin(angle[i]);
            y[i] = dis[i]*cos(angle[i]);
            x_aver += x[i];
            y_aver += y[i];
        }
        else
            data_cur->ranges[i]=0;
        // printf("(%0.6f,%0.6f)",angle[i]*180/pi,dis[i]);
        char tmp[50] = {};
        int num=sprintf(tmp,"%d\t%0.6f\t%0.6f\t%0.6f\t%0.6f\n",i,angle[i]*180/pi,dis[i],x[i],y[i]);
        write(pfile,tmp,num);
    }
    close(pfile);
    y_aver = y_aver/(end-start+1);
    x_aver = x_aver/(end-start+1);
    ROS_INFO("start = %d end = %d y_aver = %f",start,end,y_aver);
    //--拟合地面直线
    float slope[20]={},slope_line=0,slope_aver=0;
    float sumx_line=0,sumy_line=0,sumxy_line=0,sumx2_line=0;
    int k = 0;
    float jihe = 30;
    for(int i=start; i<end-jihe; i+=jihe){
        float sumx=0,sumy=0,sumxy=0,sumx2=0;
        for(int j=0; j<jihe; j++){
            sumx += x[i+j];
            sumy += y[i+j];
            sumxy += x[i+j] * y[i+j];
            sumx2 += x[i+j] * x[i+j];
        }
        float tmp1=sumy *sumx - sumxy*jihe;
        float tmp2=sumx *sumx - jihe*sumx2;
        slope[k] = tmp1 / tmp2;
        //printf("angle[i] = %f angle[i+10] = %f slope = %f\n",angle[i]*180/pi,angle[i+10]*180/pi,slope[k]);
        if(fabs(slope[k]) < 0.5){
            sumx_line += sumx;
            sumy_line += sumy;   
            sumxy_line += sumxy; 
            sumx2_line += sumx2;         
            slope_aver += slope[k];
            k++;
        }
    }
    slope_line = (sumy_line*sumx_line - sumxy_line*jihe*k) / (sumx_line*sumx_line - k*jihe*sumx2_line);
    slope_aver = slope_aver/k;
    float b_line = sumy_line/jihe/k-slope_line*sumx_line/jihe/k;
    //--计算点到直线的距离
    float dis2line[data_num] = {};
    int mark[20][2]={},tmp=0;
    for(int i=start; i<end; i++){
        dis2line[i] = fabs((y[i]-slope_line*x[i]-b_line)/sqrt(slope_line*slope_line+1));
       //printf("%f\t",dis2line[i]);
    }
    //printf("\n");
    float dislimit=0.02;
    for(int i=start; i<end; i++){
        if(dis2line[i-1] <dislimit && dis2line[i] >= dislimit)
            mark[tmp][0]=i;
        if(dis2line[i+1] <dislimit && dis2line[i] >= dislimit && mark[tmp][0])
            mark[tmp++][1]=i;       
    }
    int edge[2]={},edge_max=0;
    for(int i=0; i<20; i++){
        if(fabs(mark[i][1] - mark[i][0]) > edge_max ){
            edge_max = fabs(mark[i][1] - mark[i][0]);
            edge[0] = mark[i][0];
            edge[1] = mark[i][1];
        }
    }
    float len = fabs(x[edge[0]]-x[edge[1]]+slope_line*(y[edge[0]]-y[edge[1]]))/sqrt(slope_line*slope_line+1);
    if(len > 0.6)
        ROS_ERROR("!!! === can not find the rail === !!!");
    float offset1 = (x[edge[0]]+slope_line*y[edge[0]])/sqrt(slope_line*slope_line+1);
    float offset2 = (x[edge[1]]+slope_line*y[edge[1]])/sqrt(slope_line*slope_line+1);

    if(fabs(offset2) > fabs(offset1)){
        if(offset2>0)
            offset1 = offset2 - 0.045;
        else
            offset1 = offset2 + 0.045;
    }
    else{
        if(offset1>0)
            offset2 = offset1 - 0.045;
        else
            offset2 = offset1 + 0.045;
    }
        
    float offset_tmp = (offset1 + offset2)/2;
    for(int i=edge[0]; i<=edge[1];i++){
        data_cur->intensities[i]=20;
    }
    
    off[record++]=offset_tmp;
    record=record>9?0:record;
    for(int i=0; i<11; i++){
        if(off[i])
            *offset+=off[i];
        else{
            *offset=*offset / i;
            break;
        }
    }

    
    ROS_WARN("--edge_0: i=%d dis=%f angle=%f x=%f y=%f--\n",edge[0],dis[edge[0]],angle[edge[0]]*180/pi,x[edge[0]],y[edge[0]]);
    ROS_WARN("--edge_1: i=%d dis=%f angle=%f x=%f y=%f--\n",edge[1],dis[edge[1]],angle[edge[1]]*180/pi,x[edge[1]],y[edge[1]]);
    ROS_WARN("-- kuandu = %f --\n",(x[edge[0]]-x[edge[1]])*1000);
    ROS_WARN("-- offset1=%f  offset2=%f offset_tmp=%f offset_aver=%f kuandu=%f--\n",offset1,offset2,offset_tmp,*offset,offset2-offset1);
    ROS_INFO("slope_line = %f",slope_line);

    return;
}

