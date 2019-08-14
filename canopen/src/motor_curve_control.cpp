#include "canopen/motor_curve_control.h"
#include "string.h"
#include "ros/ros.h"
#include "math.h"
#include "canopen/timer.h"

S_CURVE_PARAM Param;

//--转向电机速度s形曲线实时速度函数
//--输入当前时间:s
float velocity(float t)
{
    float dq=0;
    if(t>=0 && t<Param.Tj1)
        dq=Param.v0+Param.jmax*pow(t,2)/2;
    else if(t>=Param.Tj1 && t<Param.Ta-Param.Tj1)   
        dq=Param.v0+Param.alima*(t-Param.Tj1/2);
    else if(t>=Param.Ta-Param.Tj1 && t<Param.Ta)
        dq=Param.vlim+Param.jmin*pow((Param.Ta-t),2)/2;
    else if(t>=Param.Ta && t<Param.Ta+Param.Tv)
        dq=Param.vlim;
    else if(t>=Param.T-Param.Td && t<Param.T-Param.Td+Param.Tj2)
        dq=Param.vlim-Param.jmax*pow((t-Param.T+Param.Td),2)/2;
    else if(t>=Param.T-Param.Td+Param.Tj2 && t<Param.T-Param.Tj2)
        dq=Param.vlim+Param.alimd*(t-Param.T+Param.Td-Param.Tj2/2);
    else if(t>=Param.T-Param.Tj2 && t<Param.T)
        dq=Param.v1+Param.jmax*pow((Param.T-t),2)/2;
    // %计算（3.33）
    dq=Param.sigma*dq;
    return dq;
}

//--s形曲线规划
//--输入:初末位置:°,初末速度:°/s,最大速度:°/s,最大加速度:°/s^2,最大加加速度:°/s^3
//--输出:无轨迹返回0,有轨迹返回1
int s_curve_calc(float q0,float q1,float v0,float v1,float v_max,float a_max,float j_max)
{
    memset(&Param,0,sizeof(S_CURVE_PARAM)); 
    float tj1=0,ta=0,tv=0,tj2=0,tjs=0,td=0,t=0,vlim=0,alima=0,alimd=0,jmax=0,jmin=0,sigma=0;
    Param.q0=q0;
    Param.q1=q1;
    Param.v0=v0;
    Param.v1=v1;
    Param.jmax=j_max;
    Param.jmin=-j_max;
    if(q1-q0>0)
        Param.sigma=1;
    else
        Param.sigma=-1;
    ROS_INFO("q0=%f q1=%f v0=%f v1=%f j_max=%f sigma=%f",q0,q1,v0,v1,j_max,Param.sigma);

    // %计算（3.17）和（3.18）
    float t1=sqrt(abs(v1-v0)/j_max);
    float t2=a_max/j_max;
    if(t1<=t2)
    {
        tjs=t1;
        float Dq=fabs(q1-q0);
        if(Dq<tjs*(v0+v1))
        {
            printf("位移过小，不存在满足始末速度的轨迹！程序退出。");
            return 0;
        }
    }  
    else
    {
        tjs=t2;
        float Dq=fabs(q1-q0);
        if(Dq<0.5*(v0+v1)*(tjs+abs(v1-v0)/a_max))
        {
            printf("位移过小，不存在满足始末速度的轨迹！程序退出。");
            return 0;
        }     
    }

    // %输入参数正确误（即轨迹存在），分类讨论
    if(( v_max-v0)*j_max<pow(a_max,2)) // %(3.19)满足,amax不能达到
    {
        tj1=sqrt((v_max-v0)/j_max);
        ta=2*tj1;
        Param.alima=j_max*tj1;
    }
    else // %(3.19)不满足，amax能达到
    {
        tj1=a_max/j_max;
        ta=tj1+(v_max-v0)/a_max;
        Param.alima=a_max;
    }
    if(( v_max-v1)* j_max<pow(a_max,2)) // %(3.20)满足,amin不能达到
    {
        tj2=sqrt((v_max-v1)/j_max);
        td=2*tj2;
        Param.alimd=-j_max*tj2;
    } 
    else // %(3.20)不满足,amin能达到
    {
        tj2=a_max/j_max;
        td=tj2+(v_max-v1)/a_max;
        Param.alimd=-a_max;
    } 
    // %计算（3.25）
    tv=(q1-q0)/v_max-ta/2*(1+v0/ v_max)-td/2*(1+v1/ v_max);
    if(tv>0)// %case1,最大速度能达到
    {
        Param.vlim= v_max;
        Param.Tj1=tj1;
        Param.Ta=ta;
        Param.Tj2=tj2;
        Param.Td=td; 
        Param.T=ta+tv+td;
        Param.Tv=tv;
        ROS_INFO("case1:Tj1=%f Tj2=%f Ta=%f Td=%f Tv=%f T=%f alima=%f alimd=%f vlim=%f",Param.Tj1,Param.Tj2,Param.Ta,Param.Td,Param.Tv,Param.T,Param.alima,Param.alimd,Param.vlim);
        return 1;
    }     
    else // % case2,最大速度不能达到
    {
        tv=0;
        Param.Tv=tv;
        // %计算（3.26a）,(3.27),（3.26b）,（3.26c）
        float tj=a_max/j_max;
        tj1=tj;
        tj2=tj;
        float delta=pow(a_max,4)/pow(j_max,2)+2*(pow(v0,2)+pow(v1,2))+a_max*(4*fabs(q1-q0)-2*a_max/j_max*(v0+v1));
        ta=(pow(a_max,2)/j_max-2*v0+sqrt(delta))/(2*a_max);
        td=(pow(a_max,2)/j_max-2*v1+sqrt(delta))/(2*a_max);
        if(ta>2*tj && td>2*tj)
        {
            // %加速段和减速段都能达到最大加速度
            Param.Tj1=tj1;
            Param.Tj2=tj2;
            Param.Ta=ta;
            Param.Td=td;
            Param.T=ta+tv+td;
            Param.alima=a_max;
            Param.alimd=-a_max;
            Param.vlim=v0+(ta-tj1)*Param.alima;
            ROS_INFO("case2.1:Tj1=%f Tj2=%f Ta=%f Td=%f Tv=%f T=%f alima=%f alimd=%f vlim=%f",Param.Tj1,Param.Tj2,Param.Ta,Param.Td,Param.Tv,Param.T,Param.alima,Param.alimd,Param.vlim);
            return 1;
        }   
        else
        {
            // %至少有一段不能达到最大加速度
            float gamma=0.99;
            float amax=a_max;
            // %逐渐减小最大加速度约束
            while(ta<2*tj || td<2*tj)
            {
                if(ta>0 && td>0)
                {
                    amax=gamma*amax;
                        // %循环计算（3.26a）,(3.27),（3.26b）,（3.26c）
                    tj=amax/j_max;
                    tj1=tj;
                    tj2=tj;
                    delta=pow(amax,4)/pow(j_max,2)+2*(pow(v0,2)+pow(v1,2))+amax*(4*fabs(q1-q0)-2*amax/j_max*(v0+v1));
                    ta=(pow(a_max,2)/j_max-2*v0+sqrt(delta))/(2*amax);
                    td=(pow(a_max,2)/j_max-2*v1+sqrt(delta))/(2*amax); 
                }         
                else
                {
                    // %出现ta或td小于0
                    if(ta<=0)
                    {
                        ta=0;
                        tj1=0;
                        // %计算（3.28a）
                        td=2*(q1-q0)/(v0+v1);
                        // %计算（3.28b）
                        float num=j_max*(q1-q0)-sqrt(j_max*(j_max*pow(fabs(q1-q0),2)+pow((v1+v0),2)*(v1-v0)));
                        float den=j_max*(v1+v0);
                        tj2=num/den;
                    }  
                    else if(td<=0)
                    {
                        td=0;
                        tj2=0;
                        // %计算（3.29a）
                        ta=2*(q1-q0)/(v0+v1);
                        // %计算（3.29b）
                        float num=j_max*(q1-q0)-sqrt(j_max*(j_max*pow(fabs(q1-q0),2)-pow((v1+v0),2)*(v1-v0)));
                        float den=j_max*(v1+v0);
                        tj1=num/den;
                    }
                }  
            }
            Param.Tj1=tj1;
            Param.Tj2=tj2;
            Param.Ta=ta;
            Param.Td=td;
            Param.T=ta+tv+td;
            Param.alima=j_max*tj1;
            Param.alimd=-j_max*tj2;
            Param.vlim=v0+(ta-tj1)*Param.alima;
            ROS_INFO("case2.３:Tj1=%f Tj2=%f Ta=%f Td=%f Tv=%f T=%f alima=%f alimd=%f vlim=%f",Param.Tj1,Param.Tj2,Param.Ta,Param.Td,Param.Tv,Param.T,Param.alima,Param.alimd,Param.vlim);
            return 1;
        }          
    }
                  
}


//--转向电机s形曲线规划
//--输入:初末位置:°,初末速度:rpm,最大速度:rpm,最大加速度,最大加加速度
//--输出:无轨迹返回0,正在进行返回1,轨迹结束返回2
int finish = 0,existence=0;
float time_cur=0;//单位:s
int timer_fd=0;
int steering_motor_s_curve(float q0,float q1,float v0,float v1,float v_max,float a_max,float j_max)
{

    float speed=0.0;
    unsigned long data = 0;
    // 单位换算
    float ratio=50;//传动比
    v0 = v0/ratio*360/60;//rpm--°/s
    v1 = v1/ratio*360/60;//rpm--°/s
    v_max = v_max/ratio*360/60;//rpm--°/s
    a_max = a_max/ratio*360;//rps^2--°/s^2
    j_max = j_max/ratio*360;//rps^3--°/s^3
    //曲线计算
    if(finish == 0)
    {
        existence=s_curve_calc(q0,q1,v0,v1,v_max,a_max,j_max);
    }  
    if(existence)
    {
        if(finish == 0)
        {
            finish = 1;
            timer_fd = timer_open();//打开计时器
        }
        if(time_cur < Param.T)
        {
            int _read=read(timer_fd, &data, sizeof(long unsigned int));
            if(_read <= 0)
            {
                // ROS_INFO("Steering_motor_s_curve timer wait");
                return 1;
            }
            else
            {
                time_cur = time_cur+15.0/1000;
                speed=velocity(time_cur)*ratio/360*60;
                //-----给泰克电机速度speed-----
                // tech_set_speed_mode(5, speed);
                // tech_set_speed_mode(6, speed);
                // tech_set_speed_mode(7, speed);
                // tech_set_speed_mode(8, speed);
                ROS_INFO("time_cur = %f speed = %f finish = %d",time_cur,speed,finish);
                return 1;
            }
        }
        else if(time_cur >= Param.T)
        {
            ROS_INFO("Steering_motor_s_curve over");
            timer_close(timer_fd);//关闭计时器
            //-----给泰克电机速度0-----
            // tech_set_speed_mode(5, 0);
            // tech_set_speed_mode(6, 0);
            // tech_set_speed_mode(7, 0);
            // tech_set_speed_mode(8, 0);
            timer_fd=0;
            time_cur=0;
            finish=0;
            existence=0;
            return 2;
        } 
    }
    else
    {
        finish=0;
        return 0; 
    }

}