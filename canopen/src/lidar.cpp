#include <memory.h>
#include <string.h>
#include <ros/ros.h>                           // 包含ROS的头文件
#include "std_msgs/String.h"              //ros定义的String数据类型
#include <sensor_msgs/LaserScan.h>
#include "canopen/lidar.h"
#include "canopen/type.h"
#include "math.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h> 
#include <unistd.h> 
#include <netdb.h> 
#include <netinet/in.h>   
#include <arpa/inet.h>  
#define pi 3.1415

#define REMOTE_PORT 22222
const char* g_remoteIP="192.168.10.101";//192.168.8.100

int location_mode=0;
laser_scan_data timscan;
laser_scan_data timscan2;
laser_scan_data lmsscan;
laser_scan_data *data_cur;
int lms_receive=0,sick_receive=0,sick_receive2=0,process=0;
int mark[2][100]={};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "lidar");       //初始化节点
    ros::NodeHandle n;   
    ros::Publisher laser_pub =n.advertise<sensor_msgs::LaserScan>("laser", 1);
    ros::Subscriber sub1 = n.subscribe("sick1", 1, sickCallback);
    ros::Subscriber sub2 = n.subscribe("sick2", 1, sick2Callback);
    ros::Subscriber sub3 = n.subscribe("lms1xx_scan", 1, lms1xxCallback);
    ros::Rate loop_rate(100);
    float dis_=0, theta_=0;
    float center_x=0, center_theta=0, center_dis=0, side_dis=0, side_theta=0;
    float center_x2=0, center_theta2=0, center_dis2=0, side_dis2=0, side_theta2=0;
    while (ros::ok()) 
    {
        // update_to_server(dis_ , theta_);
        // ROS_INFO("wait");
        //ROS_INFO("sick_receive=%d sick_receive2=%d",sick_receive,sick_receive2);
        if(sick_receive || lms_receive || sick_receive2)
        {
            if(sick_receive){
                data_cur = & timscan;
                //--初次矫正位置
                // lidar_calibration(&dis_,&theta_);
                // ROS_WARN("sick1 : dis = %f angle = %f",dis_,theta_);
                //--测量车体
                laser_location(&center_x,&center_theta,&center_dis,&side_dis,&side_theta);
                process|=1;
                sick_receive=0;
            }
            if(sick_receive2){
                data_cur = & timscan2;
                //--初次矫正位置
                // lidar_calibration(&dis_,&theta_);
                // ROS_WARN("sick2 : dis = %f angle = %f",dis_,theta_);
                //--测量车体
                laser_location(&center_x2,&center_theta2,&center_dis2,&side_dis2,&side_theta2);
                float theta_ = -side_theta2;
                float dis_ = side_dis2-0.9-4.2*tan(theta_*3.1415/180);
                update_to_server(dis_ , theta_);
                process|=2;
                sick_receive2=0;
            }
            if((process & 3) == 3){
                float dis_ = (side_dis-side_dis2)/2;
                float theta_ = (side_theta+side_theta2)/2;
                ROS_WARN("dis = %f angle = %f",(side_dis-side_dis2)/2,(side_theta+side_theta2)/2);
                update_to_server(dis_ , theta_);
                process=0;
            }
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
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;  
}

void lms1xxCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    lmsscan.angle_min=(float)msg->angle_min; //开始角度
    lmsscan.angle_max=(float)msg->angle_max; //结束角度
    lmsscan.angle_increment=(float)msg->angle_increment; //测量的角度间的距离
    lmsscan.time_increment=(float)msg->time_increment; //测量间的时间
    lmsscan.scan_time=(float)msg->scan_time; //扫描间的时间
    lmsscan.range_min=(float)msg->range_min; //最小测量距离
    lmsscan.range_max=(float)msg->range_max; //最大测量距离
    lmsscan.data_num=(int)((lmsscan.angle_max-lmsscan.angle_min)/lmsscan.angle_increment+1);
    for(int i=0;i<lmsscan.data_num;i++)
    {
        lmsscan.ranges[i]=(float)msg->ranges[i]; //测量的距离数据
        lmsscan.intensities[i]=(float)msg->intensities[i]; //强度数据
    }
    ROS_INFO("lms1xxCallback: receive %d data",lmsscan.data_num);
    lms_receive=1;
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

void sick2Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    timscan2.angle_min=(float)msg->angle_min; //开始角度
    timscan2.angle_max=(float)msg->angle_max; //结束角度
    timscan2.angle_increment=(float)msg->angle_increment; //测量的角度间的距离
    timscan2.time_increment=(float)msg->time_increment; //测量间的时间
    timscan2.scan_time=(float)msg->scan_time; //扫描间的时间
    timscan2.range_min=(float)msg->range_min; //最小测量距离
    timscan2.range_max=(float)msg->range_max; //最大测量距离
    timscan2.data_num=(int)((timscan2.angle_max-timscan2.angle_min)/timscan2.angle_increment+1);
    for(int i=0;i<timscan2.data_num;i++)
    {
        timscan2.ranges[i]=(float)msg->ranges[i]; //测量的距离数据
        timscan2.intensities[i]=(float)msg->intensities[i]; //强度数据
    }
    ROS_INFO("sick2Callback: receive %d data",timscan2.data_num);
    sick_receive2=1;
}

void laser_location(float *center_x,float *center_theta,float *center_dis,float *side_dis,float *side_theta)
{
    const float x_tol_limit=0.02,angle_max_limit=60,angle_min_limit=20;
    float angle_aver[2]={},dis_aver[2]={};
    float wheel_angle[2][100]={},wheel_dis[2][100]={};
    int count[2][2]={},count_limit=5,mark[2][100]={};
    float wheel_x[2][100]={},wheel_y[2][100]={},x_aver[4]={},y_aver[4]={};
    float sumx[4]={},sumy[4]={},sumxy[4]={},sumx2[4]={},sumy2[4]={},variance[4]={};
    float A=0,B=0,C=0,A_side=0,B_side=0,k_line[2]={};
    int color[3]={100,200,300};

    int data_num=data_cur->data_num;
    float angle[data_num]={},dis[data_num]={};
    int pfile = 0;
    //--原始数据处理
    // if((pfile=open("./laserdata/data.txt",O_RDWR|O_CREAT|O_APPEND))<0){
    //     ROS_WARN("ERROR:Can not open ~/laserdata/data.txt!");
    // }
    for(int i=0;i<data_num;i++){
        angle[i]=data_cur->angle_min+data_cur->angle_increment*i;//单位为弧度
        dis[i]=data_cur->ranges[i];//单位为毫米
        // printf("(%0.6f,%0.6f)",angle[i]*180/pi,dis[i]);
        char tmp[20]={};
        int num=sprintf(tmp,"%0.6f\t%0.6f\n",angle[i]*180/pi,dis[i]);
        //write(pfile,tmp,num);
    }
    //close(pfile);
    //--提取后轮轮胎数据：聚类算法
    // ROS_INFO("dbscan");
    wheel_find_by_dbscan(angle,dis,wheel_angle,wheel_dis,count);
    for(int i=0;i<2;i++){
        for(int j=0;j<count[0][i];j++){
            wheel_x[i][j]=wheel_dis[i][j]*sin(wheel_angle[i][j]);
            wheel_y[i][j]=wheel_dis[i][j]*cos(wheel_angle[i][j]);
            // ROS_INFO("wheel_angle=%f wheel_dis=%f",wheel_angle[i][j]*180/3.1415,wheel_dis[i][j]);
        }
    }
    ROS_INFO("count_0=%d count_1=%d",count[0][0],count[0][1]);
    if(count[0][0]<count_limit || count[0][1]<count_limit){
        ROS_WARN("laser_location error:can not find the wheel!");
        *center_x=0;
        *center_theta=0;
        *center_dis=0;
        *side_dis=0;
        *side_theta=0;
        return;
    }
    else
    {
        //--根据线性拟合程度取出轮胎的四条线段
        int use[4]={},line_end[4]={};
        if(linearity_calculation(wheel_x,wheel_y,count[0],line_end)<0){
            *center_x=0;
            *center_theta=0;
            *center_dis=0;
            *side_dis=0;
            *side_theta=0;
            return;
        }
        for(int m=0;m<4;m++){
            int startlimit=0,endlimit=0;
            switch(m){
                case 0:
                    startlimit=0;
                    endlimit=line_end[0];
                break;
                case 1:
                    startlimit=count[0][0]-line_end[1];
                    endlimit=count[0][0];
                break;
                case 2:
                    startlimit=0;
                    endlimit=line_end[2];
                break; 
                case 3:
                    startlimit=count[0][1]-line_end[3];
                    endlimit=count[0][1];
                break;
            }   
            for(int n=startlimit;n<endlimit;n++){
                // ROS_INFO("m=%d n=%d wheel_x[(int)m/2][n]=%f wheel_y[(int)m/2][n]=%f",m,n,wheel_x[(int)m/2][n],wheel_y[(int)m/2][n]);
                // ROS_INFO("sumy=%f sumx=%f sumxy=%f sumx2=%f",sumy[m],sumx[m],sumxy[m],sumx2[m]);
                data_cur->intensities[count[1][(int)(m/2)]+n]=280;
                sumx[m] += wheel_x[(int)m/2][n];
                sumy[m] += wheel_y[(int)m/2][n];
                sumxy[m] += wheel_x[(int)m/2][n]*wheel_y[(int)m/2][n];
                sumx2[m] += pow(wheel_x[(int)m/2][n],2);
                sumy2[m] += pow(wheel_y[(int)m/2][n],2);
                use[m]++;
            }
            // ROS_INFO("m=%d sumx=%f sumy=%f sumxy=%f sumx2=%f sumy2=%f",m,sumx[m],sumy[m],sumxy[m],sumx2[m],sumy2[m]);
            x_aver[m]=sumx[m]/(endlimit-startlimit);
            for(int n=startlimit;n<endlimit;n++){
                variance[m] += pow((wheel_x[(int)m/2][n]-x_aver[m]),2);
            }
            variance[m]=variance[m]/(endlimit-startlimit);
            // ROS_INFO("m=%d use[m]=%d x_aver[m]=%f variance[m]=%f",m,use[m],x_aver[m],variance[m]);
        }
        //--判断采集到的数据是否为轮子数据：判断两组平行线是否平行，相差小于3度则认为平行
        for(int i=0;i<4;i++){
            if(i==0 || i==3){
                float tmp1=sumy[i]*sumx[i] - sumxy[i]*(float)use[i];
                float tmp2=sumx[i]*sumx[i] - (float)use[i]*sumx2[i];
                k_line[i]=tmp1 / tmp2;
                ROS_INFO("-- i=%d use_i=%d k_line_i=%f angel_i=%f --",i,use[i],k_line[i],atan(1/k_line[i])*180/pi);
            }
            else{
                float tmp1=sumy[i]*sumx[i] - sumxy[i]*(float)use[i];
                float tmp2=sumy[i]*sumy[i] - (float)use[i]*sumy2[i];
                k_line[i]=tmp1 / tmp2;
                ROS_INFO("-- i=%d use_i=%d k_line_i=%f angel_i=%f --",i,use[i],k_line[i],atan(k_line[i])*180/pi);
            }
        }
        if(fabs(fabs(atan(k_line[1])) - fabs(atan(k_line[2]))) > 30*3.14/180
        || fabs(fabs(atan(1/k_line[0])) - fabs(atan(1/k_line[3]))) > 30*3.14/180){
            ROS_WARN("laser_location error:can not find the parallel wheel!");
            *center_x=0;
            *center_theta=0;
            *center_dis=0;
            *side_dis=0;
            *side_theta=0;
            return;
        }
        else if(variance[1]<pow(0.0005,2) && variance[2]<pow(0.0005,2)){
            //--计算车轮与泊车机器人相对位置
            *center_theta=0;
            *center_x=(variance[1]+variance[2])/2;
            *center_dis=fabs(x_aver[1])+fabs(x_aver[2]);
            ROS_INFO("-- theta=%f dis=%f wheel_dis=%f --",*center_theta,*center_x,*center_dis);
        }
        else{
            //--计算车轮与泊车机器人相对位置：拟合两内侧边缘平行线
            //--y=ax+b
            // A =(use[0]*use[1]*sumxy[0] + use[0]*use[1]*sumxy[1] - use[1]*sumx[0]*sumy[0]-use[0]*sumx[1]*sumy[1])
            // /(use[1]*pow(sumx[0],2) + use[0]*pow(sumx[1],2) - use[0]*use[1]*sumx2[0] - use[0]*use[1]*sumx2[1]);
            // B =(sumy[0]*pow(sumx[1],2) - sumx[0]*sumy[1]*sumx[1] - use[1]*sumy[0]*sumx2[0] - use[1]*sumy[0]*sumx2[1] + use[1]*sumx[0]*sumxy[0] + use[1]*sumx[0]*sumxy[1])
            // /(use[1]*pow(sumx[0],2) + use[0]*pow(sumx[1],2) - use[0]*use[1]*sumx2[0] - use[0]*use[1]*sumx2[1]);
            // C =(sumy[1]*pow(sumx[0],2) - sumx[1]*sumy[0]*sumx[0] - use[0]*sumy[1]*sumx2[0] - use[0]*sumy[1]*sumx2[1] + use[0]*sumx[1]*sumxy[0] + use[0]*sumx[1]*sumxy[1])
            // /(use[1]*pow(sumx[0],2) + use[0]*pow(sumx[1],2) - use[0]*use[1]*sumx2[0] - use[0]*use[1]*sumx2[1]);
            //--x=ay+b
            A = -(use[1]*use[2]*sumxy[1] + use[1]*use[2]*sumxy[2] - use[2]*sumx[1]*sumy[1] - use[1]*sumx[2]*sumy[2])
                /(use[1]*sumy[2]*sumy[2] + use[2]*sumy[1]*sumy[1] - use[1]*use[2]*sumy2[1] - use[1]*use[2]*sumy2[2]);
            B = (sumx[1]*sumy[2]*sumy[2] - use[2]*sumx[1]*sumy2[1] - use[2]*sumx[1]*sumy2[2] + use[2]*sumy[1]*sumxy[1] + use[2]*sumy[1]*sumxy[2] - sumx[2]*sumy[1]*sumy[2])
                /(use[1]*sumy[2]*sumy[2] + use[2]*sumy[1]*sumy[1] - use[1]*use[2]*sumy2[1] - use[1]*use[2]*sumy2[2]);
            C = (sumx[2]*sumy[1]*sumy[1] - use[1]*sumx[2]*sumy2[1] - use[1]*sumx[2]*sumy2[2] + use[1]*sumy[2]*sumxy[1] + use[1]*sumy[2]*sumxy[2] - sumx[1]*sumy[1]*sumy[2])
                /(use[1]*sumy[2]*sumy[2] + use[2]*sumy[1]*sumy[1] - use[1]*use[2]*sumy2[1] - use[1]*use[2]*sumy2[2]);
            *center_theta=atan(A)*180/pi;
            float L1=fabs(B/sqrt(A*A+1));
            float L2=fabs(C/sqrt(A*A+1));
            float dis_offset1=(L1-L2)/2;
            ROS_INFO("-- L1=%f L2=%f dis_offset1=%f --",L1,L2,dis_offset1);
            *center_x=(B+C)/2/cos(atan(A));
            *center_dis=fabs((B-C)/cos(atan(A)));
            ROS_INFO("-- A=%f B=%f C=%f theta=%f dis=%f wheel_dis=%f --",A,B,C,*center_theta,*center_x,*center_dis);
            float num_0_3=use[0]+use[3];
            float sumx_0_3=sumx[0]+sumx[3];
            float sumy_0_3=sumy[0]+sumy[3];
            float sumxy_0_3=sumxy[0]+sumxy[3];
            float sumx2_0_3=sumx2[0]+sumx2[3];
            A_side=(num_0_3*sumxy_0_3-sumx_0_3*sumy_0_3)/(num_0_3*sumx2_0_3-pow(sumx_0_3,2));
            B_side=(sumy_0_3*sumx2_0_3-sumx_0_3*sumxy_0_3)/(num_0_3*sumx2_0_3-pow(sumx_0_3,2));
            *side_theta=atan(A_side)*180/3.1415;
            *side_dis=fabs(B_side);
            //*side_dis=fabs(B_side/sqrt(pow(A_side,2)+1));
            ROS_INFO("-- A_side=%f B_side=%f side_theta=%f side_dis=%f --",A_side,B_side,*side_theta,*side_dis);
        } 
    }
    return;
}

int linearity_calculation(float x[][100],float y[][100],int *num,int *line_num)
{
    const int linearity_num=10;
    float linearity[4][linearity_num]={};
    int max[4]={};float max_linearity[4]={};
    for(int i=0;i<linearity_num;i++){
        float startpercent=0,endpercent=0.2+0.5*i/linearity_num;
        float startnum=0,endnum=0;
        float sumx[4]={},sumy[4]={},x_bar[4]={},y_bar[4]={};
        float u11[4]={},u20[4]={},u02[4]={};
        for(int q=0;q<4;q++){
            switch(q){
                case 0:
                    startnum=0;
                    endnum=floor(endpercent*num[0]);
                break;
                case 1:
                    startnum=floor(num[0]*(1-endpercent));
                    endnum=floor(num[0]*(1-startpercent));
                break;
                case 2:
                    startnum=0;
                    endnum=floor(endpercent*num[1]);
                break;
                case 3:
                    startnum=floor(num[1]*(1-endpercent));
                    endnum=floor(num[1]*(1-startpercent));
                break;
            }
            for(int p=startnum;p<endnum;p++){
                // ROS_INFO("q=%d p=%d x[(int)q/2][p]=%f y[(int)q/2][p]=%f",q,p,x[(int)q/2][p],y[(int)q/2][p]);
                // ROS_INFO("sumx[q]=%f sumy[q]=%f",sumx[q],sumy[q]);
                sumx[q]=sumx[q]+x[(int)q/2][p];
                sumy[q]=sumy[q]+y[(int)q/2][p];
            }
            x_bar[q]=sumx[q]/(endnum-startnum);
            y_bar[q]=sumy[q]/(endnum-startnum);
            for(int p=startnum;p<endnum;p++){
                u11[q]=u11[q]+(x[(int)q/2][p]-x_bar[q])*(y[(int)q/2][p]-y_bar[q]);
                u20[q]=u20[q]+(x[(int)q/2][p]-x_bar[q])*(x[(int)q/2][p]-x_bar[q]);
                u02[q]=u02[q]+(y[(int)q/2][p]-y_bar[q])*(y[(int)q/2][p]-y_bar[q]);
            }
            linearity[q][i]=sqrt(4*pow(u11[q],2.0)+pow((u20[q]-u02[q]),2.0))/(u20[q]+u02[q]);
            // ROS_INFO("q=%d i=%d linearity[q][i]=%f",q,i,linearity[q][i]);
        }
    }
    //--寻找最大线性拟合度
    for(int i=0;i<4;i++){
        for(int j=0;j<linearity_num;j++){
            // ROS_INFO("i=%d j=%d linearity[i][j]=%f",i,j,linearity[i][j]);
            if(linearity[i][j] >= max_linearity[i]){
                max_linearity[i]=linearity[i][j];
                max[i]=j;
            }
        }
    }
    for(int i=0;i<4;i++){
        if(max_linearity[i] < 0.9){
            ROS_WARN("linearity_calculation error:can not find linear!(max_linearity=%f)",max_linearity[i]);
            return -1;
        }
    }
    line_num[0]=floor((0.2+0.05*max[0])*num[0]);
    line_num[1]=floor((0.2+0.05*max[1])*num[0]);
    line_num[2]=floor((0.2+0.05*max[2])*num[1]);
    line_num[3]=floor((0.2+0.05*max[3])*num[1]);
    // ROS_INFO("max=%d %d %d %d max_linearity=%f %f %f %f",max[0],max[1],max[2],max[3],max_linearity[0],max_linearity[1],max_linearity[2],max_linearity[3]);
    return 1;
}

void wheel_find_by_dbscan(float *angle,float *dis,float wheel_angle[][100],float wheel_dis[][100],int num[][2])
{
    float dis_limit_percent=0.02,angle_max_limit=80*3.1415/180,angle_min_limit=20*3.1415/180;
    int data_num=data_cur->data_num;
    int cluster_max=30,len_max=120,cluster_len[30]={},mark[30]={};
    float angle_cluster[30][120]={},dis_cluster[30][120]={};
    float angle_aver[30]={},dis_aver[30]={};
    int m=0,n=0,line[6]={};
    //--聚类
    for(int i=0;i<data_num;i++){
        if(!i){
            angle_cluster[m][n]=angle[i];
            dis_cluster[m][n]=dis[i];
            angle_aver[m]=angle_aver[m]+angle[i];
            dis_aver[m]=dis_aver[m]+dis[i];
            data_cur->intensities[i]=15*(m+1);
            mark[m]=i;
            n++;
        }
        else{
            if(fabs(dis[i]-dis[i-1])<dis[i]*0.02){
                angle_cluster[m][n]=angle[i];
                dis_cluster[m][n]=dis[i];
                angle_aver[m]=angle_aver[m]+angle[i];
                dis_aver[m]=dis_aver[m]+dis[i];
                data_cur->intensities[i]=15*(m+1);
                n++;
            }
            else if(n<10){
                memset(angle_cluster[m],0,sizeof(angle_cluster)/cluster_max);
                memset(dis_cluster[m],0,sizeof(dis_cluster)/cluster_max);
                for(int k=i;k>i-n;k--){
                    data_cur->intensities[k]=1;
                }
                angle_aver[m]=0;
                dis_aver[m]=0;
                n=0;
                angle_cluster[m][n]=angle[i];
                dis_cluster[m][n]=dis[i];
                angle_aver[m]=angle_aver[m]+angle[i];
                dis_aver[m]=dis_aver[m]+dis[i];
                data_cur->intensities[i]=15*(m+1);
                mark[m]=i;
                n++;
            }
            else{
                angle_aver[m]=angle_aver[m]/n;
                dis_aver[m]=dis_aver[m]/n;
                cluster_len[m]=n;
                // ROS_INFO("m=%d  n=%d",m,n);
                m++;n=0;
                angle_cluster[m][n]=angle[i];
                dis_cluster[m][n]=dis[i];
                angle_aver[m]=angle_aver[m]+angle[i];
                dis_aver[m]=dis_aver[m]+dis[i];
                data_cur->intensities[i]=15*(m+1);
                mark[m]=i;
                n++;
            }
        }
    }
    //--提取轮胎数据
    int k=0;float dis_minus[3]={99,99,99};
    for(int i=0;i<m;i++){
        // ROS_INFO("i=%d angle_aver=%f dis_aver=%f len=%d mark=%d",i,angle_aver[i]*180/3.1415,dis_aver[i],cluster_len[i],mark[i]);
        if(dis_aver[i]>1 && dis_aver[i]<2.5){
            if(fabs(angle_aver[i])<angle_max_limit && fabs(angle_aver[i])>angle_min_limit){
                line[k++]=i;
                //ROS_INFO("i=%d  k=%d",i,k);
            }
        }
    }
    if(k == 2){
        for(int i=0;i<2;i++){
            memcpy(wheel_angle[i],angle_cluster[line[i]],100*sizeof(float));
            memcpy(wheel_dis[i],dis_cluster[line[i]],100*sizeof(float));
            num[0][i]=(int)cluster_len[line[i]];
            num[1][i]=mark[line[i]];
            for(int s=num[1][i];s<num[1][i]+num[0][i];s++){
                    data_cur->intensities[s]=200;
            }
        }
        // ROS_INFO("num[0]=%d num[1]=%d",num[0],num[1]);
    }
    else{
        for(int i=0;i<k;i++){
            for(int j=i+1;j<k;j++){
                if(angle_aver[line[i]]*angle_aver[line[j]] < 0
                && dis_aver[line[i]] <= dis_aver[line[i]-1]
                && dis_aver[line[j]] <= dis_aver[line[j]+1]
                && fabs(dis_aver[line[i]]-dis_aver[line[j]]) < dis_minus[0]){
                    dis_minus[0]=fabs(dis_aver[line[i]]-dis_aver[line[j]]);
                    dis_minus[1]=line[i];
                    dis_minus[2]=line[j];
                }
            }
        }
        if(dis_minus[1]!=99 && dis_minus[2]!=99){
            for(int i=0;i<2;i++){
                memcpy(wheel_angle[i],angle_cluster[(int)dis_minus[i+1]],100*sizeof(float));
                memcpy(wheel_dis[i],dis_cluster[(int)dis_minus[i+1]],100*sizeof(float));
                num[0][i]=cluster_len[(int)dis_minus[i+1]];
                num[1][i]=mark[(int)dis_minus[i+1]];
                for(int s=num[1][i];s<num[1][i]+num[0][i];s++){
                    data_cur->intensities[s]=200;
                }
            }
        }
    }
    return;
}


void lidar_calibration(float *dis_,float *theta_)
{
    float plane_angle[200]={},plane_dis[200]={};
    int num[2]={};
    float sumx=0,sumy=0,sumx2=0,sumxy=0,sumy2=0;
    
    int data_num=data_cur->data_num;
    float angle[data_num]={},dis[data_num]={},x[data_num]={},y[data_num]={};
    int pfile = 0;
    //--原始数据处理
    // if((pfile=open("./laserdata/data.txt",O_RDWR|O_CREAT|O_APPEND))<0){
    //     ROS_WARN("ERROR:Can not open ~/laserdata/data.txt!");
    // }
    for(int i=0;i<data_num;i++){
        angle[i]=data_cur->angle_min+data_cur->angle_increment*i;//单位为弧度
        dis[i]=data_cur->ranges[i];//单位为毫米
        x[i]=dis[i]*sin(angle[i]);
        y[i]=dis[i]*cos(angle[i]);
        // printf("(%0.6f,%0.6f)",angle[i]*180/pi,dis[i]);
        char tmp[20]={};
        int num=sprintf(tmp,"%0.6f\t%0.6f\n",angle[i]*180/pi,dis[i]);
        //write(pfile,tmp,num);
    }
    //close(pfile);
    //--找到校准平面
    plane_find_by_dbscan(angle,dis,num);
    for(int i=num[1]; i<num[0]+num[1]; i++){
        sumx += x[i];
        sumy += y[i];
        sumxy += x[i]*y[i];
        sumx2 += x[i]*x[i];
        sumy2 += y[i]*y[i];
        //printf("dis=%f angle=%f x=%f y=%f\n",dis[i],angle[i]*180/pi,x[i],y[i]);
    }
    ROS_INFO("num[0]=%d num[1]=%d sumx=%f sumy=%f sumxy=%f sumx2=%f sumy2=%f",num[0],num[1],sumx,sumy,sumxy,sumx2,sumy2);
    float A = (num[0]*sumxy-sumx*sumy)/(num[0]*sumx2-pow(sumx,2));
    float B = (sumy*sumx2-sumx*sumxy)/(num[0]*sumx2-pow(sumx,2));
    *theta_ = atan(A);
    *dis_ = fabs(B/sqrt(pow(A,2)+1));
    ROS_INFO("A = %f B = %f plane_theta = %f plane_dis = %f",A,B,*theta_*180/3.1415,*dis_);
    return;
}


void plane_find_by_dbscan(float *angle,float *dis,int num[2])
{
    float dis_limit_percent=0.02,angle_max_limit=30*3.1415/180;
    int data_num=data_cur->data_num;
    int cluster_max=30,len_max=120,cluster_len[30]={},mark[30]={};
    float angle_cluster[30][120]={},dis_cluster[30][120]={};
    float angle_aver[30]={},dis_aver[30]={};
    int m=0,n=0,line[6]={};
    //--聚类
    for(int i=0;i<data_num;i++){
        if(!i){
            angle_cluster[m][n]=angle[i];
            dis_cluster[m][n]=dis[i];
            angle_aver[m]=angle_aver[m]+angle[i];
            dis_aver[m]=dis_aver[m]+dis[i];
            data_cur->intensities[i]=15*(m+1);
            mark[m]=i;
            n++;
        }
        else{
            if(fabs(dis[i]-dis[i-1])<dis[i]*0.02){
                angle_cluster[m][n]=angle[i];
                dis_cluster[m][n]=dis[i];
                angle_aver[m]=angle_aver[m]+angle[i];
                dis_aver[m]=dis_aver[m]+dis[i];
                data_cur->intensities[i]=15*(m+1);
                n++;
            }
            else if(n<10){
                memset(angle_cluster[m],0,sizeof(angle_cluster)/cluster_max);
                memset(dis_cluster[m],0,sizeof(dis_cluster)/cluster_max);
                for(int k=i;k>i-n;k--){
                    data_cur->intensities[k]=1;
                }
                angle_aver[m]=0;
                dis_aver[m]=0;
                n=0;
                angle_cluster[m][n]=angle[i];
                dis_cluster[m][n]=dis[i];
                angle_aver[m]=angle_aver[m]+angle[i];
                dis_aver[m]=dis_aver[m]+dis[i];
                data_cur->intensities[i]=15*(m+1);
                mark[m]=i;
                n++;
            }
            else{
                angle_aver[m]=angle_aver[m]/n;
                dis_aver[m]=dis_aver[m]/n;
                cluster_len[m]=n;
                // ROS_INFO("m=%d  n=%d",m,n);
                m++;n=0;
                angle_cluster[m][n]=angle[i];
                dis_cluster[m][n]=dis[i];
                angle_aver[m]=angle_aver[m]+angle[i];
                dis_aver[m]=dis_aver[m]+dis[i];
                data_cur->intensities[i]=15*(m+1);
                mark[m]=i;
                n++;
            }
        }
    }
    //--提取平面数据
    int k=0;float dis_minus[3]={99,99,99};
    for(int i=0;i<m;i++){
        //ROS_INFO("i=%d angle_aver=%f dis_aver=%f len=%d mark=%d",i,angle_aver[i]*180/3.1415,dis_aver[i],cluster_len[i],mark[i]);
        if(dis_aver[i]>0.4 && dis_aver[i]<1.5){
            if(fabs(angle_aver[i])<angle_max_limit && num[0]<(int)cluster_len[i]){
                num[0]=(int)cluster_len[i];
                num[1]=mark[i];
                for(int s=num[1];s<num[1]+num[0];s++){
                        data_cur->intensities[s]=300;
                }
            }
        }
    }
    return;
}


void update_to_server(float dis,float theta)
{
        ROS_INFO("update_to_server!");
        char sendbuf[300]={0};
        struct sockaddr_in serverAddr;   //**客户端
        int SendToServer;   //**客户端

        //客户端socket初始化//
        ///定义clientSocket
        if((SendToServer = socket(AF_INET,SOCK_STREAM, 0))<0){
                ROS_INFO("SendSocket initial error!");
        }
        ROS_INFO("SendSocket initial ok!");
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(REMOTE_PORT);  ///服务器端口
        serverAddr.sin_addr.s_addr = inet_addr(g_remoteIP);  ///服务器ip

        //将socket建立为非阻塞，此时socket被设置为非阻塞模式
        // int flags = fcntl(SendToServer,F_GETFL,0);//获取建立的sockfd的当前状态（非阻塞）
        // fcntl(SendToServer,F_SETFL,flags|O_NONBLOCK);//将当前sockfd设置为非阻塞 

        ///连接服务器，成功返回0，错误返回-1
        if (connect(SendToServer, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0 ){
            printf("Connect to %s:%d failed ...\n" , g_remoteIP , REMOTE_PORT) ;
            close(SendToServer);
            return;
        }
        int j=sprintf(sendbuf,"<LIDAR><DIS>%f</DIS><THETA>%f</THETA></LIDAR>",dis,theta);
        ROS_INFO("SEND %d : %s",j,sendbuf);
        int temp=send(SendToServer,sendbuf,j,0);
        ROS_INFO("SEND %d :%d %s",j,temp,sendbuf);
        j=0;
        close(SendToServer);
        return;
}
