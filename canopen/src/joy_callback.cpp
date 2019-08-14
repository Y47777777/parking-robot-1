#include <stdio.h>    //usb_can函数
#include <unistd.h>
#include <asm/types.h>
#include <canopen/PCANBasic.h>
#include <canopen/MOTEC_motorCtl.h>//MOTEc电机
#include <canopen/techMotorCtl.h> //泰科电机
#include <pthread.h> 
#include <canopen/joy_callback.h>
#include "string.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "canopen/sport_control.h"
#include "canopen/auto_lift.h"
#include "canopen/car_driver.h"
#include "canopen/gyroscope.h"
#include "canopen/timer.h"
#define pi 3.1415
#define du2rad(x) x*3.1415/180
#define rad2du(x) x/3.1415*180
/*
开始自动行走　左按键左　
停止自动行走　左按键右
整车前进/后退　　左摇杆前/后 && RB
１２轮前进/后退　　左按键上/下 && RB
３４轮前进/后退　　左按键左/右 && RB
轮子转回零点　ＢＡＣＫ  && RB
轮子横移准备　左摇杆左　右遥感右
轮子对中心　左遥感右　右遥感左
开始左/右横移　左摇杆左/右 && RB
开始左/右转　　左按键左/右 && LB
点动单独旋转车轮 １/2/3/4　  Ａ/B/X/Y  ＆＆  ＬＢ/ＲＢ
连动单独旋转车轮 １/2/3/4　  Ａ/B/X/Y  ＆＆  ＬT/ＲT
整体夹车/松开　　左上/下 ＆＆ ＬB
单独夹车/松开 １/2/3/4　  Ａ/B/X/Y　＆＆　左上（前后则为AB/XY组合）
*/

int codeCur=0,xPositionCur=0,yPositionCur=0;//当前二维码信息
float angleCur=0;
int orderNoCur=0,pointNoCur=0;
int AGVupdate_ready=0;
POINT_MSG point_cur;
int xPositionT_agv=0;//以目标车头方向建立坐标系的XY值 
int yPositionT_agv=0;
int xPositionCur_agv=0;//以当前车头方向建立坐标系的XY值
int yPositionCur_agv=0;
float angleCur_agv=0;

int wheel_direction=0;   //轮组方向
int AGV_direction=0;   //轮组方向
int motec_disable_enable=1; 
int mode_auto=0;

float slow_y=0.1;         //?y?ùDD×?
float adjust_turn=0.08;  //矫正角速度  0.2----0.05
float adjust_slow_turn=0.06;  //矫正角速度
float run_w=0.2;       //×a?ò?ù?è
float slow_w=0.1;
float hand_x=0.2;          //ê??ˉμ÷???ù?è
float hand_y=0.5;
float hand_w=0.4;
float hand_up=210000;        //10度
float Qwalk_speed = 0.5;
int angle_dir = 1; //陀螺角度方向系数:前进为1，后退为-1
int lift_cmd = 0;
float Bjf_angle;//倍加福偏移角度 
float Q_DuoLun_angle;//转向盘转角

//整体轮组转动角度,横移时调整输入90，轮组回正时输入0
void wheels_rotate_bfrun(float angle,int disable,int time)
{
    if(disable)
    {
        Motec_Disable();
        motec_disable_enable = 0;     //驱动电机失效标志
    }
    
    tech_turn_ex(5, angle);
    tech_turn_ex(8, angle);
    tech_turn_ex(6, -angle); 
    tech_turn_ex(7, -angle);

    if(time)//根据当前轮组位置计算所需要的延时时间
    {
        if(angle == READY_RUN)
        {
            if(wheel_direction == READY_TURN)
            {
                delay_sec(ROTARY_TIME);
            }
            else
            {
                delay_sec(SHIFT_TIME);
            }
        }
        else if(angle == READY_SHIFT)
        {
            if(wheel_direction == READY_TURN)
            {
                delay_sec(SHIFT_TIME-ROTARY_TIME);
            }
            else
            {
                delay_sec(SHIFT_TIME);
            }
        }
    }

    wheel_direction=angle;

}

//整体轮组同向动角度,用于直线行走过程中调整：右转为正，左转为负
void wheels_syntropy_rotate(float angle,int mode)
{
    switch(mode)
    {
        case 0:
            tech_turn_ex(5, angle);
            tech_turn_ex(8, angle);
            tech_turn_ex(6, angle); 
            tech_turn_ex(7, angle);
        break;
        case 1:
            tech_turn_ex(5, angle);
            tech_turn_ex(6, angle);
        break;
        case 2:
            tech_turn_ex(7, angle);
            tech_turn_ex(8, angle);
        break;
   }
    wheel_direction=RUN_ADJUST;
}

//橫移过程中轮组同向转动一定角度，相对于橫移状态而言，角度不超过90度，顺时针方向为正
void wheels_rotate_shift90(float angle,int disable)
{
    if(disable)
    {
        Motec_Disable();
        motec_disable_enable = 0;     //驱动电机失效标志
    }
    tech_set_arr_fast(5);
    tech_turn_ex(5, 90+angle);
    tech_set_arr_fast(8);
    tech_turn_ex(8, 90-angle);
    tech_set_arr_fast(6);
    tech_turn_ex(6, -90+angle);
    tech_set_arr_fast(7);
    tech_turn_ex(7, -90+angle);
    wheel_direction=SHIFT_ADJUST;
}

//橫移过程中轮组前后异向转动一定角度，相对于橫移状态而言，角度不超过90度，5,7号顺时针方向为正
void wheels_rotate_adjust(float angle,int disable)
{
    if(disable)
    {
        Motec_Disable();
        motec_disable_enable = 0;     //驱动电机失效标志
    }
    tech_set_arr_fast(5);
    tech_turn_ex(5, 90+angle);
    tech_set_arr_fast(8);
    tech_turn_ex(8, 90+angle);
    tech_set_arr_fast(6);
    tech_turn_ex(6, -90-angle);
    tech_set_arr_fast(7);
    tech_turn_ex(7, -90+angle);
    wheel_direction=SHIFT_ADJUST;
}

//准备旋转:time=1 进行延时  time=0：不进行延时
void wheels_rotate_bfturn(int time)      
{
    Motec_Disable(); 
    motec_disable_enable = 0;     //驱动电机失效标志

    tech_set_arr(5);
    tech_turn_ex(5, 78);
    tech_set_arr(8);
    tech_turn_ex(8, 78);
    //delay_sec_nsec(3,500000);
    //delay_sec(5);
    tech_set_arr(6);
    tech_turn_ex(6, -78); 
    tech_set_arr(7);
    tech_turn_ex(7, -78);

    if(time)//根据当前轮组位置计算所需要的延时时间
    {
        if(wheel_direction == READY_SHIFT  || wheel_direction == SHIFT_ADJUST)
        {
            delay_sec(SHIFT_TIME-ROTARY_TIME);
        }
        else
        {
            delay_sec(ROTARY_TIME);
        }
    }

    wheel_direction=READY_TURN;		
}

//车辆前后行走函数：前进速度为正
void car_direct_run(float v,int disable,int time)
{
    int delay_flag=0;
    if(wheel_direction != READY_RUN)
    {
        wheels_rotate_bfrun(0,disable,time);
    }
    if(motec_disable_enable == 0)
    {
        ROS_INFO("car_direct_run");
        Motec_Enable();
        motec_disable_enable = 1;
    }
    Speed_AGV(v);
}

//车辆前后行走调整函数：前进速度为正
//v为速度，前进为正，w为角速度，圆心在右时为正
void car_adjust_run(float v,float w)
{

    float v1,v2,a1,a2,R;
    if(wheel_direction != READY_RUN)
    {
        wheels_rotate_bfrun(0,0,0);
    }
    if(motec_disable_enable == 0)
    {
        Motec_Enable();
        motec_disable_enable = 1;
    }
    if(w==0)
    {
        Speed_AGV(v);
        return;
    }
    else if(v==0)
    {
        Stop_AGV();
        return;
    }
    else
    {   
        wheel_direction=RUN_ADJUST;
        R = fabs(v/w);
        a1 = (atan(CAR_L/(R-CAR_H)))*90/pi;//较大转角，内侧
        a2 = (atan(CAR_L/(R+CAR_H)))*90/pi;//较小转角，外侧
        v1 = fabs(w)*(-WHEEL_L+sqrt(pow(fabs(R)-CAR_L,2)+pow(CAR_H,2)));//较小速度，内侧
        v1=v1*v/fabs(v);
        v2 = fabs(w)*(WHEEL_L+sqrt(pow(fabs(R)+CAR_L,2)+pow(CAR_H,2)));//较大速度，外侧
        v2=v2*v/fabs(v);
        ROS_INFO("V1::%f---v2::%f  a1:%f   a2:%f  R:%f v:%f w:%f",v1,v2,a1,a2,R,v,w);
        if(w>0)//圆心在右侧
        {
            ROS_INFO("yuan xin zai you ce");
            tech_set_arr_fast(5);
            tech_turn_ex_slow(5, a2);
            tech_set_arr_fast(8);
            tech_turn_ex_slow(8, -a1);
            // tech_set_arr(6);
            tech_set_arr_fast(6);
            tech_turn_ex_slow(6, a1);
            tech_set_arr_fast(7);
            tech_turn_ex_slow(7, -a2);
            // tech_set_arr(8);
            set_motor_speed(1, v2);            
            set_motor_speed(2, -v1);           
            set_motor_speed(3, v2);          
            set_motor_speed(4, -v1);  
        }
        else//圆心在左侧
        {
            ROS_INFO("yuan xin zai zuo ce");
            // tech_set_arr(5);
            tech_set_arr_fast(5);
            tech_turn_ex_slow(5, -a1);
            tech_set_arr_fast(8);
            tech_turn_ex_slow(8, a2);
            tech_set_arr_fast(6);
            tech_turn_ex_slow(6, -a2);
            // tech_set_arr(7); 
            tech_set_arr_fast(7);
            tech_turn_ex_slow(7, a1);
            set_motor_speed(1, v1);                     
            set_motor_speed(2, -v2);               
            set_motor_speed(3, v1);              
            set_motor_speed(4, -v2); 
        }
        return;
    }  
}

void car_adjustSpeed_run(float v,float w)
{

    float v1,v2,a1,a2,R;
    if(wheel_direction != READY_RUN)
    {
        wheels_rotate_bfrun(0,0,0);
    }
    if(motec_disable_enable == 0)
    {
        Motec_Enable();
        motec_disable_enable = 1;
    }
    if(w==0)
    {
        Speed_AGV(v);
        return;
    }
    else if(v==0)
    {
        Stop_AGV();
        return;
    }
    else
    {   
        wheel_direction=RUN_ADJUST;
        R = fabs(v/w);
        a1 = (atan(CAR_L/(R-CAR_H)))*90/pi;//较大转角，内侧
        a2 = (atan(CAR_L/(R+CAR_H)))*90/pi;//较小转角，外侧
        v1 = fabs(w)*(-WHEEL_L+sqrt(pow(fabs(R)-CAR_L,2)+pow(CAR_H,2)));//较小速度，内侧
        v1=v1*v/fabs(v);
        v2 = fabs(w)*(WHEEL_L+sqrt(pow(fabs(R)+CAR_L,2)+pow(CAR_H,2)));//较大速度，外侧
        v2=v2*v/fabs(v);
        ROS_INFO("V1::%f---v2::%f  a1:%f   a2:%f  R:%f v:%f w:%f",v1,v2,a1,a2,R,v,w);
        if(w>0)//圆心在右侧
        {
            ROS_INFO("yuan xin zai you ce");
            set_motor_speed(1, v2);            
            set_motor_speed(2, -v1);           
            set_motor_speed(3, v2);          
            set_motor_speed(4, -v1);  
        }
        else//圆心在左侧
        {
            ROS_INFO("yuan xin zai zuo ce");
            set_motor_speed(1, v1);                     
            set_motor_speed(2, -v2);               
            set_motor_speed(3, v1);              
            set_motor_speed(4, -v2); 
        }
        return;
    }  
}

//车辆左右横移函数：左移速度为正
void car_direct_shift(float v,int disable,int time)
{
    int delay_flag=0;
    if(wheel_direction!=90 && wheel_direction!=-90)
    {
        wheels_rotate_bfrun(90,disable,time);
    }
    if(motec_disable_enable == 0)
    {
        Motec_Enable();
        motec_disable_enable = 1;
    }
    float speed=v*wheel_direction/90.0;
    Shift_AGV(speed);
}

//车辆左右横移调整函数：左移速度为正
//v为速度，左移为正，w为角速度，圆心在后时为正
void car_adjustSpeedAngle_shift(float v,float w)
{
    float v1,v2,a1,a2,R;
    int delay_flag=0;
    if(wheel_direction != READY_SHIFT)
    {
        if(wheel_direction == SHIFT_ADJUST)
            wheels_rotate_bfrun(90,0,0);
        else
            wheels_rotate_bfrun(90,0,1); 
    }
    if(motec_disable_enable == 0)
    {
        Motec_Enable();
        motec_disable_enable = 1;
    }
    if(w==0)
    {
        car_direct_shift(v,0,0);
        return;
    }
    else
    {   
        wheel_direction=SHIFT_ADJUST;
        R = fabs(v/w);
        a1 = (atan(CAR_L/(R-CAR_H)))*180/pi;//较大转角，内侧
        a2 = (atan(CAR_L/(R+CAR_H)))*180/pi;//较小转角，外侧
        v1 = fabs(w)*(-WHEEL_L+sqrt(pow(fabs(R)-CAR_H,2)+pow(CAR_L,2)));//较小速度，内侧
        v1=v1*v/fabs(v);
        v2 = fabs(w)*(WHEEL_L+sqrt(pow(fabs(R)+CAR_H,2)+pow(CAR_L,2)));//较大速度，外侧
        v2=v2*v/fabs(v);
        ROS_INFO("a1:%f  a2:%f  V1::%f  v2::%f  R:%f  v:%f   w:%f",a1,a2,v1,v2,R,v,w);
        if(w>0)//圆心在后方
        {
            tech_set_arr(5);
            tech_turn_ex(5, 90-a2);
            tech_set_arr_fast(8);
            tech_turn_ex(8, 90+a1);
            tech_set_arr(6);
            tech_turn_ex(6, -90+a2);
            tech_set_arr_fast(7);
            tech_turn_ex(7, -90-a1);
            set_motor_speed(1, -v2);            
            set_motor_speed(2, -v2);           
            set_motor_speed(3, v1);          
            set_motor_speed(4, v1);  
        }
        else//圆心在前方
        {
            tech_set_arr_fast(5);
            tech_turn_ex(5, 90+a1);
            tech_set_arr(8);
            tech_turn_ex(8, 90-a2);
            tech_set_arr_fast(6);
            tech_turn_ex(6, -90-a1);
            tech_set_arr(7); 
            tech_turn_ex(7, -90+a2);
            set_motor_speed(1, -v1);                     
            set_motor_speed(2, -v1);               
            set_motor_speed(3, v2);              
            set_motor_speed(4, v2); 
        }
        return;
    }  
}

//车辆左右横移调整函数：左移速度为正
//v为速度，左移为正，w为角速度，圆心在后时为正
void car_adjustSpeed_shift(float v,float w)
{
    float v1,v2,a1,a2,R;
    int delay_flag=0;
    if(wheel_direction != READY_SHIFT)
    {
        wheels_rotate_bfrun(90,0,1);
    }
    if(motec_disable_enable == 0)
    {
        Motec_Enable();
        motec_disable_enable = 1;
    }
    if(w==0)
    {
        car_direct_shift(v,0,0);
        return;
    }
    else
    {   
        R = fabs(v/w);
        v1 = fabs(w)*(-WHEEL_L+sqrt(pow(fabs(R)-CAR_H,2)+pow(CAR_L,2)));//较小速度，内侧
        v1=v1*v/fabs(v);
        v2 = fabs(w)*(WHEEL_L+sqrt(pow(fabs(R)+CAR_H,2)+pow(CAR_L,2)));//较大速度，外侧
        v2=v2*v/fabs(v);
        ROS_INFO("a1:%f  a2:%f  V1::%f  v2::%f  R:%f  v:%f   w:%f",a1,a2,v1,v2,R,v,w);
        if(w > 0)//圆心在后方
        {
            ROS_INFO("yuan xin zai che wei");
            set_motor_speed(1, -v2);            
            set_motor_speed(2, -v2);           
            set_motor_speed(3, v1);          
            set_motor_speed(4, v1);  
        }
        else//圆心在前方
        {
            ROS_INFO("yuan xin zai che tou");
            set_motor_speed(1, -v1);                     
            set_motor_speed(2, -v1);               
            set_motor_speed(3, v2);              
            set_motor_speed(4, v2); 
        }
        return;
    }  
}

//停止：整车驱动轮停止
void car_stop(void)      
{
    motor_stop(1);
    motor_stop(2);
    motor_stop(3);
    motor_stop(4);
    motor_stop(5);
    motor_stop(6);
    motor_stop(7);
    motor_stop(8);
    motor_stop(9);
    motor_stop(10);
    motor_stop(11);
    motor_stop(12);
}

//整车绕中心旋转，旋转角速度为w,顺时针转w>0,逆时针转w<0
void car_turn(float w)
{
    if(wheel_direction!=READY_TURN)
    {
        wheels_rotate_bfturn(1);
        delay_sec(ROTARY_TIME);
    }
    if(motec_disable_enable == 0)
    {
        Motec_Enable();
        motec_disable_enable = 1;
    }
    set_motor_speed(1, w*(CAR_W+WHEEL_L));			
    set_motor_speed(2, w*(CAR_W+WHEEL_L));
    set_motor_speed(3, w*(CAR_W+WHEEL_L));			
    set_motor_speed(4, w*(CAR_W+WHEEL_L));
}

//--遥控手柄回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    
    if(mode_auto==0 && an_RB==1 && L_ganUD !=0 && R_ganLR == 0 && R_ganUD == 0)//RB+左摇杆前后：整车前进后退
    {
        Speed_AGV(L_ganUD);
    }
    else if(mode_auto==0  && an_RB ==1 && L_ganLR != 0 )//RB+左遥感左右：轮子左右橫移
    {
        car_direct_shift(L_ganLR,1,0);
    }
    else if(mode_auto==0 && an_LB==1 && (L_ganLR !=0 || L_ganUD!=0)) //LB+左摇杆：整车调整中前进后退
    {
        if(L_ganLR>0.2)
        {
            if(L_ganUD>0)
                car_adjust_run(0.4,-0.1);
            else
                car_adjust_run(-0.4,0.1);
        }
        else if(L_ganLR<-0.2)
        {
            if(L_ganUD>0)
                car_adjust_run(0.4,0.1);
            else
                car_adjust_run(-0.4,-0.1);
        }
        else
        {
            if(L_ganUD>0)
                car_adjust_run(0.4,0);
            else
                car_adjust_run(-0.4,0);
        }
    }
    else if(mode_auto==0 && L_anUD !=0 && an_RB == 1 && L_anLR == 0) //RB+左按键前后：1、2号轮前进后退
    {
        motor_stop(3);
        motor_stop(4);
        set_motor_speed(1, slow_y*L_anUD);
        set_motor_speed(2, -slow_y*L_anUD); 
    }
    else if(mode_auto==0 && L_anUD ==0 && an_RB == 1 && L_anLR != 0) //RB+左按键左右：3、4号轮前进后退
    {
        motor_stop(1);
        motor_stop(2);
        set_motor_speed(3, slow_y*L_anLR);
        set_motor_speed(4, -slow_y*L_anLR);
    }
    else if(mode_auto==0 && an_RB ==1 && an_BACK == 1)//RB+BACK：轮子转回零点
    {
        wheels_rotate_bfrun(0,1,0);
    }
    else if(mode_auto==0 && L_ganLR == 1 && R_ganLR == -1 ) //左摇杆左+右摇杆右：轮子同向转动90度，横移前准备
    {
        wheels_rotate_bfrun(90,1,0);
    }
    
    else if(mode_auto==0 && L_ganLR == -1 && R_ganLR == 1 )//左摇杆右+右摇杆左：轮子对中心，整车旋转前准备
    {
        wheels_rotate_bfturn(0);
    }
    else if(mode_auto==0 && L_anLR != 0 && an_LB == 1 ) //LB+左按键：原地旋转
    {
        car_turn(-hand_w*L_anLR);
    }
    else if(mode_auto==0 && an_A == 1 && LT<0  && RT > 0)//LT+A按键：1号轮旋转
    {
        disable_model_set(1);
        tech_turn(5,LT);
        usleep(10000);
        motec_disable_enable = 0;
    } 
    else if(mode_auto==0 && an_A == 1 && LT > 0  && RT < 0)//RT+A按键：1号轮旋转
    {
        disable_model_set(1);
        tech_turn(5,-RT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_B == 1 && LT<0  && RT > 0)//LT+B按键：2号轮旋转
    {
        disable_model_set(2);
        tech_turn(6,LT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_B == 1 && LT > 0  && RT < 0)//RT+B按键：2号轮旋转
    {
        disable_model_set(2);
        tech_turn(6,-RT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_X == 1 && LT<0  && RT > 0)//LT+X按键：3号轮旋转
    {
        disable_model_set(3);
        tech_turn(7,LT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_X == 1 && LT > 0  && RT < 0)//RT+X按键：3号轮旋转
    {
        disable_model_set(3);
        tech_turn(7,-RT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_Y == 1 && LT<0  && RT > 0)//LT+Y按键：4号轮旋转
    {
        disable_model_set(4);
        tech_turn(8,LT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_Y == 1 && LT > 0  && RT < 0)//RT+Y按键：4号轮旋转
    {
        disable_model_set(4);
        tech_turn(8,-RT);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_A == 1 && an_LB== 1 && an_RB == 0)//LB+A按键：1号轮旋转
    {
        disable_model_set(1);
        tech_turn(5,-1);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_A == 1 && an_LB== 0 && an_RB == 1)//RB+A按键：1号轮旋转 顺时针
    {
        disable_model_set(1);
        tech_turn(5,1);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(an_LB== 1 && an_RB == 1)
    {
        lift_cmd = 1;  
        ROS_INFO("Shou Dao Jia Che Min Ling");  
    }
    else if(mode_auto==0 && an_B == 1 && an_LB== 1 && an_RB == 0) //LB+B按键：2号轮旋转 逆时针
    {
        disable_model_set(2);
        tech_turn(6, -1);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_B == 1 && an_LB== 0 && an_RB == 1)  //RB+B按键：2号轮旋转 顺时针
    {
        disable_model_set(2);
        tech_turn(6, 1);
         usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_X == 1 && an_LB== 1 && an_RB == 0) //LB+X按键：3号轮旋转 逆时针
    {
        disable_model_set(3);
        tech_turn(7, -1);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_X == 1 && an_LB== 0 && an_RB == 1) //RB+X:3号轮旋转 顺时针
    {
        disable_model_set(3);
        tech_turn(7, 1);
        usleep(10000);
        motec_disable_enable = 0;
    }
    else if(mode_auto==0 && an_Y == 1 && an_LB== 1 && an_RB == 0)  //LB+Y按键：4号轮旋转 逆时针
    {
        disable_model_set(4);
        tech_turn(8, -1);
        usleep(10000);
        motec_disable_enable = 0;
    }
    
    else if(mode_auto==0 && an_Y == 1 && an_LB== 0 && an_RB == 1) //RB+Y:4号轮旋转 顺时针
    {
        disable_model_set(4);
        tech_turn(8, 1);
        usleep(10000);
        motec_disable_enable = 0;
    } 

    else if(mode_auto==0 && an_LB== 1 && an_B == 0 &&  L_anUD!= 0 && L_anLR == 0) // 整体提升:左按键上下ＬB
    {
        set_motor_position(9,-hand_up*L_anUD);
        set_motor_position(10,-hand_up*L_anUD);
        set_motor_position(11,-hand_up*L_anUD);
        set_motor_position(12,-hand_up*L_anUD);
        usleep(10000);
    }
    else if(mode_auto==0 && an_A == 1 && an_B == 0 &&  L_anUD!= 0 && L_anLR == 0)   //A+左按键上下：1号夹杆夹车 放车
    {
        set_motor_position(9,-hand_up*L_anUD);
        usleep(10000);
    } 
    else if(mode_auto==0 && an_A == 0 && an_B == 1 && L_anUD!= 0 && L_anLR == 0)   //B+左按键上下：2号夹杆夹车 放车
    {
        set_motor_position(10,-hand_up*L_anUD);
        usleep(10000);
    } 
    else if(mode_auto==0 && an_X == 1 && an_Y == 0 && L_anUD!= 0 && L_anLR == 0)   //X+左按键上下：3号夹杆夹车 放车
    {
        set_motor_position(11,-hand_up*L_anUD);
        usleep(10000);
    } 
    else if(mode_auto==0 && an_X == 0 && an_Y == 1 && L_anUD!= 0 && L_anLR == 0)  //Y+左按键上下：4号夹杆夹车 放车
    {
        set_motor_position(12,-hand_up*L_anUD);
        usleep(10000);
    } 
    else if(mode_auto==0 && an_A == 1 && an_B == 1 &&  L_anUD!= 0 && L_anLR == 0)   //AB+左按键上下：1/2号夹杆夹车 放车
    {
        set_motor_position(9,-hand_up*L_anUD);
        set_motor_position(10,-hand_up*L_anUD);
        usleep(10000);
    } 
     else if(mode_auto==0 && an_X == 1 && an_Y == 1 && L_anUD!= 0 && L_anLR == 0)  //XY+左按键上下：3/4号夹杆夹车 放车
    {
        set_motor_position(11,-hand_up*L_anUD);
        set_motor_position(12,-hand_up*L_anUD);
        usleep(10000);
    } 
    else if(an_LB == 0 && an_A == 0 && L_anLR ==1 && L_anUD ==0 && an_A ==0 && an_B ==0 && an_X ==0 && an_Y ==0 && an_BACK == 0 ) //左按键左
    {
        mode_auto=1;
        ROS_INFO("an_a;");
    }
    
    else if(an_LB == 0 && an_LB == 0 && L_anLR ==-1 && L_anUD ==0 && an_A ==0 && an_B ==0 && an_X ==0 && an_Y ==0 && an_BACK == 0 ) //左按键右
    {
        mode_auto=0;
        lift_cmd = 0;
        car_stop();
    }
    else
    {
       // ROS_INFO("mode_auto:%d  motec_enable:%d    wheel_direction:%d",mode_auto,motec_disable_enable,wheel_direction);
        if(mode_auto == 0)
        {
            if(!motec_disable_enable)
            {
                Motec_Enable();
                usleep(10000);
                motec_disable_enable=1;
            }
             car_stop();           
            // printf("stop!!!!\n");
        }
    }
    
}

//--扫玛器信息回调函数
void scannerCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	    float angleCur_tar=0;
    	bool flag_180=0;
        xPositionCur=msg->linear.x;
        yPositionCur=msg->linear.y;
        codeCur=msg->linear.z;
        angleCur=msg->angular.x;
        orderNoCur=msg->angular.y;
        pointNoCur=msg->angular.z;
        calculation_bf_autoturn(0,&angleCur_agv,&angleCur_tar,&flag_180); 
        if(angleCur_agv==0)//不同车头朝向时二维码转换
        {
            xPositionCur_agv=xPositionCur;
            yPositionCur_agv=yPositionCur;
        }
        else if(fabs(angleCur_agv)==180)
        {
            xPositionCur_agv=-xPositionCur;
            yPositionCur_agv=-yPositionCur;
        }
        else if(angleCur_agv==90 || angleCur_agv==-90)
        {
             xPositionCur_agv=-yPositionCur*(angleCur_agv/fabs(angleCur_agv));
             yPositionCur_agv=xPositionCur*(angleCur_agv/fabs(angleCur_agv));
        }    
}

//--路径信息回调函数
void traceCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    point_cur.code=msg->linear.x;
    point_cur.direction=msg->linear.y;
    point_cur.length=msg->linear.z;
    point_cur.run_mode=msg->angular.x;
    point_cur.speed=msg->angular.y/1000.0f;
    point_cur.reserve=msg->angular.z;
    if(point_cur.direction==0)//不同车头朝向时二维码转换
    {
        xPositionT_agv=xPositionCur;
        yPositionT_agv=yPositionCur;
    }
    else if(fabs(point_cur.direction)==180)
    {
        xPositionT_agv=-xPositionCur;
        yPositionT_agv=-yPositionCur;
    }
    else if(point_cur.direction==90 || point_cur.direction==-90)
    {
         xPositionT_agv=-yPositionCur*(point_cur.direction/fabs(point_cur.direction));
         yPositionT_agv=xPositionCur*(point_cur.direction/fabs(point_cur.direction));
    }
    // ROS_INFO("code:%d direction:%d length:%d run_mode:%d speed;%d",point_cur.code,point_cur.direction,point_cur.length,point_cur.run_mode,point_cur.speed);
}

/*****************************************************
name		:Bjf_angle_to0
function	:倍加福角度归0处理
input		:bjf_angle
output		:返回归0后的角度
******************************************************/
float Bjf_angle_to0(float angle)
{/*定义左偏为正，右偏为负*/
	float angle_to0 = 0.0;
	if((angle <= 30)&&(angle >=0))
		{/*车头朝y，车左偏*/
			angle_to0 = angle;
		}
	if((angle >= -30)&&(angle < 0))
		{/*车头朝y，车右偏*/
			angle_to0 = angle;
		}
	if((angle >= -90)&&(angle <= -60))
		{/*车头朝x，车左偏*/
			angle_to0 = (90 - fabs(angle)); 
		}
	if((angle >= -120)&&(angle <-90))
		{/*车头朝x，车右偏*/
			angle_to0 = (fabs(angle) - 90)*(-1);
		}
	if((angle >= 90)&&(angle <= 120))
		{/*车头朝-x，车左偏*/
			angle_to0 = (angle - 90);
		}	
	if((angle >= 60)&&(angle < 90))
		{/*车头朝-x，车右偏*/
			angle_to0 = angle - 90;
		}
	if((angle >= -180)&&(angle <= -160))
		{/*车头朝-y，车左偏*/
			angle_to0 = angle + 180;
		}
	if((angle >= 160)&&(angle <= 180))
		{/*车头朝-y，车右偏*/
			angle_to0 = angle - 180;
		}
	return angle_to0;
}

//--自动旋转前针对二维码信息计算当前车头位置与目标车头位置
void calculation_bf_autoturn(float angle,float *angleCur_cor,float *angleCur_tar,bool *flag_180)
{
    //判断当前位置
    if(angleCur>-30 && angleCur<30)
    {
        *angleCur_cor=0;
    }
    else if(angleCur>60 && angleCur<120)
    {
        *angleCur_cor=90;
    }
    else if(angleCur>-120 && angleCur<-60)
    {
        *angleCur_cor=-90;
    }
    else if(angleCur<-150 || angleCur>150)
    {
        *angleCur_cor=180;
    }
    //计算应抵达位置
    *angleCur_tar=*angleCur_cor+angle;
    if(*angleCur_tar>180)
    {
        *angleCur_tar-=360;
        *flag_180=1;
    }
    else if(*angleCur_tar<-180)
    {
        *angleCur_tar+=360;
        *flag_180=1;
    }
}

//在二维码上车身自动旋转:-90为顺时针转90度，90为逆时针转90度,输入速度为整车旋转角速度
int car_auto_turn(float angle,float speed_max) 
{
    static int i = 0;
    static float angleCur_cor=0;//当前角度
    static float angleCur_tar=0;//目标角度
    float angleCur_tol=1;//容错角度
    float rotary_speed=0;//转向速度
    static bool flag_180=0;
    static int once=0;
	float speed_tmp = 0.2; //旋转速度
    if(once==0)
    {
        if(wheel_direction!=READY_TURN)
        {
            car_stop();
            sleep(1);
            wheels_rotate_bfturn(1); 
        }
        calculation_bf_autoturn(angle,&angleCur_cor,&angleCur_tar,&flag_180);
        once=1;
    }
    //当前角度在容错区间外时，车身旋转
   if((abs(angleCur_tar)!=180 && (angleCur>angleCur_tar+angleCur_tol || angleCur<angleCur_tar-angleCur_tol))
   || (abs(angleCur_tar)==180 && (angleCur>-180+angleCur_tol && angleCur<180-angleCur_tol)) )
   {
       if(!mode_auto)//跳出自动运行模式
       {
           car_turn(0);
           wheels_rotate_bfrun(0,1,1);
           return 0;
       }
        if(abs(angleCur_tar-angleCur)<10)
        {
            rotary_speed=speed_tmp/10*abs(angleCur_tar-angleCur);
        }  
        else
        {
            rotary_speed=speed_tmp;
        }
       if(angle>0)
       {
           car_turn(-rotary_speed);
       }
       else if(angle<0)
       {
           car_turn(rotary_speed);
       }
       ROS_INFO("angleCur_cor:%f  angleCur_tar:%f  rotary_speed:%f  rotary_speed:%f",angleCur_cor,angleCur_tar,rotary_speed,rotary_speed);
       return 0;
   }
   else
   {
        car_stop();
        AGV_direction=angle;
        angleCur_cor=AGV_direction;//当前角度
        angleCur_tar=0;//目标角度
        flag_180=0;
        once=0;
        return 1;
   } 
}

//--在二维码上进行偏移校正
int adjust_x(float dis,int dir,float codeDir)//agv的x方向上矫正
{
    float runspeed=0;
    int limit_x=10;
    runspeed=fabs(dis)/300;
    runspeed=runspeed>0.1?0.1:runspeed;
    if(dir == 0)//agv横移
    {
        if(dis>limit_x)
        {
            car_direct_run(runspeed,1,1);
        }
        else if(dis<-limit_x)
        {
            car_direct_run(-runspeed,1,1);
        }
        else
        {
            car_stop();
            sleep(0.5);
            return 1;
        }
        return 0;
    }
    else//agv直行
    {
        if(dis>limit_x)
        {
            car_direct_shift(runspeed,1,1);
        }
        else if(dis<-limit_x)
        {
            car_direct_shift(-runspeed,1,1);
        } 
        else
        {
            car_stop();
            sleep(0.5);
            return 1;
        }
        return 0;
    }
}

//--在二维码上进行角度校正
int adjust_w(float angle)
{
    float rotspeed=0;
    if((fabs(fabs(angleCur)-fabs(angle)))  > 0.5)
    {
        if(fabs(angle) == 180)
        {
            rotspeed=fabs(fabs(angleCur)-fabs(angle));
            rotspeed=rotspeed*(-angleCur)/fabs(angleCur);
            rotspeed=rotspeed>36?(rotspeed-36):rotspeed;
            rotspeed=rotspeed<-36?(rotspeed+36):rotspeed;
            rotspeed=rotspeed/60.0;
            rotspeed=rotspeed<0.5?rotspeed:0.5;
            rotspeed=rotspeed>-0.5?rotspeed:-0.5;
            car_turn(rotspeed);   
        }
        else
        {
            rotspeed=(angleCur-angle)/60.0;
            rotspeed=rotspeed<0.5?rotspeed:0.5;
            rotspeed=rotspeed>-0.5?rotspeed:-0.5;
            car_turn(rotspeed);
        }
        //ROS_INFO("==Angle adjusting==:-------rotspeed = %f--------",rotspeed);
        return 0;
    }
    else
    {
        car_stop();
        usleep(10000);
        return 1;
    }
}

//--run_correction_xTheta校正函数针对两个二维码之间规划了两段圆弧路径进行校正
//v:车速（米每秒）     w：角速度(弧度每秒)    dis_x：车偏移（车相对二维码右偏为正）    dis_theta：车偏角，角度（车相对于二维码逆偏为正）   distance：二维码间距(毫米)    agv_dir：车头朝向，角度
void run_correction_xTheta(float v,float w,float dis_x,float dis_theta,float distance,float agv_dir)
{
    float theta_cor=0;
    dis_x=dis_x*1.251;
    ROS_INFO("v:%f w:%f dis_x:%f dis_theta:%f distance:%f   agv_dir:%f",v,w,dis_x,dis_theta,distance,agv_dir);
    if(w==0)
    {
        car_adjust_run(v,0);
        return;
    }
    if(fabs(agv_dir)!=180)
        theta_cor=(dis_theta-agv_dir)*v/fabs(v);//矫正角度：度
    else
        theta_cor=fabs(fabs(dis_theta)-agv_dir)*v/fabs(v)*(-dis_theta)/fabs(dis_theta);//矫正角度：度
    float radius_cor=fabs(v/w);//转弯半径:米
    float alpha_2=acos((radius_cor*(1+cos(theta_cor*pi/180))-fabs(dis_x)/1000)/2/radius_cor)*180/pi;//第二段圆弧：角度
    float alpha_1=alpha_2-theta_cor*dis_x/fabs(dis_x);//第一段圆弧：角度
    int time_1=pow(10,6)*alpha_1/180*pi/w;//第一段圆弧时间:单位us
    int time_2=pow(10,6)*alpha_2/180*pi/w;//第二段圆弧时间
    int dis_cor=(time_1+time_2)*v*0.001;//校正路径长度:mm
    ROS_INFO("theta_cor:%f radius_cor:%f alpha_2:%f alpha_1:%f time_1:%d   time_2:%d    dis_cor:%d",theta_cor,radius_cor,alpha_2,alpha_1,time_1,time_2,dis_cor);
    if(dis_cor>distance-200)
    {
        time_1=time_1*(distance-200)/dis_cor;
        time_2=time_2*(distance-200)/dis_cor;
    }
    ROS_INFO("theta_cor:%f radius_cor:%f alpha_2:%f alpha_1:%f time_1:%d   time_2:%d",theta_cor,radius_cor,alpha_2,alpha_1,time_1,time_2);
    time_1*=1.1;
    time_2*=0.9;
    if(dis_x>0)//车右偏
    {
        if(v>0)
        {
            // time_1*=1.2;
            // time_2*=0.8;//0.2---1.15
        }
        else
        {
            // time_1*=0.8;
            // time_2*=1.2;
        }
        if(time_1>300000)
        {
            car_adjust_run(v,-w);//圆心在左
            usleep(time_1);
        }
        if(time_2>300000)
        {
            car_adjust_run(v,w);//圆心在右
            usleep(time_2);
        }
    }
    else if(dis_x<0)//车左偏
    {
        if(v>0)//可
        {
            //  time_2/=1.2;
            //  time_1*=1.2;//0.2---1.
        }
        else
        {
            // time_1*=1.2;
            // time_2*=0.8;
        }
        if(time_1>300000)
        {
            car_adjust_run(v,w);//圆心在右
            usleep(time_1);
        }
        if(time_2>300000)
        {
            car_adjust_run(v,-w);//圆心在左
            usleep(time_2);
        }
    }
    car_direct_run(v,0,0);
    return;
}

//--run_correction_xTheta2校正函数针对两个二维码之间分为车身扭转和横移两段进行校正 
int run_correction_xTheta2(float v,float dis_x,float dis_theta,float distance,float agv_dir)
{
    float theta_cor=0;
    float alpha=10;//转角
    int limit_time=300000*0.6/(v+0.1);
    static int wheel_flag=0;
    int time_1=0,time_2=0,dis_plus=0;
    //ROS_INFO("v:%f dis_x:%f dis_theta:%f distance:%f   agv_dir:%f",v,dis_x,dis_theta,distance,agv_dir);
    if(distance<400)
    {
        car_direct_run(0,0,0);
        //car_direct_run(v,0,0);
		printf("设定距离已到达\n");
        return -1;
    }
	else
		{
			Speed_AGV(v);
		}
    if(fabs(agv_dir)!=180)
        theta_cor=dis_theta-agv_dir;//矫正角度：度
    else
        theta_cor=fabs(fabs(dis_theta)-agv_dir)*(-dis_theta)/fabs(dis_theta);//矫正角度：度
    float radius_cor=fabs(2*CAR_H/tan(alpha*pi/180));//车身扭转转弯半径:米
    time_1=pow(10,6)*fabs(radius_cor*theta_cor/180*pi/v);//第一段车身扭转时间
    time_1=time_1*1;
    if(theta_cor > 0)
        dis_plus=dis_x-radius_cor*(1-cos(theta_cor/180*pi));
    else
        dis_plus=dis_x+radius_cor*(1-cos(theta_cor/180*pi));
    if(dis_plus * dis_x<0)
        time_2 = 0;
    else
        time_2=pow(10,6)*fabs(dis_plus/(v*1000)/sin(alpha*pi/180));//第二段车身平移时间
    time_2=time_2*1;
    int dis_cor=(time_1+time_2)*v*0.001;//校正路径长度:mm
    if(dis_cor>distance-200)
    {
        time_1=time_1*(distance-200)/dis_cor;
        time_2=time_2*(distance-200)/dis_cor;
    }
    ROS_INFO("theta_cor:%f radius_cor:%f  time_1:%d   time_2:%d",theta_cor,radius_cor,time_1,time_2);
    if(dis_x*v>0)
    {
        if((time_2<2*limit_time)&&(time_1>limit_time))
        {
            if(theta_cor*v>0 && wheel_flag!=1)
            {
                wheels_syntropy_rotate(0,1);
                wheels_syntropy_rotate(-alpha,2);//后轮组左转
                ROS_INFO("----back turn left: %d----",time_1);
                wheel_flag=1;
            }
            else if(wheel_flag!=2)
            {
                wheels_syntropy_rotate(-alpha,1);//前轮组左转
                wheels_syntropy_rotate(0,2);
                ROS_INFO("----front turn left: %d----",time_1);
                wheel_flag=2;
            }
        }
        else if(time_2>limit_time && wheel_flag!=3)
        {
            wheels_syntropy_rotate(-alpha,0);//前后轮组左转
            ROS_INFO("----all turn left: %d----",time_2);
            wheel_flag=3;
        }
        else if(wheel_flag!=0)
        {
            wheels_syntropy_rotate(0,0);//全部回正
            ROS_INFO("----all wheels back----");
            wheel_flag=0;
        }
    }
    else if(dis_x*v<0)
    {
        if((time_2<2*limit_time)&&(time_1>limit_time))
        {
            if(theta_cor*v>0 && wheel_flag!=4)
            {
                wheels_syntropy_rotate(alpha,1);//前轮组右转
                wheels_syntropy_rotate(0,2);
                ROS_INFO("----front turn right: %d----",time_1);
                wheel_flag=4;
            }
            else if(wheel_flag!=5)
            {
                wheels_syntropy_rotate(alpha,2);//后轮组右转
                wheels_syntropy_rotate(0,1);
                ROS_INFO("----back turn right: %d----",time_1);
                wheel_flag=5;
            }
        }
        else if(time_2>limit_time && wheel_flag!=6)
        {
            wheels_syntropy_rotate(alpha,0);//前后轮组右转
            ROS_INFO("----all turn right: %d----",time_2);
            wheel_flag=6;
        }
        else if(wheel_flag!=0)
        {
            wheels_syntropy_rotate(0,0);//全部回正
            ROS_INFO("----all wheels back----");
            wheel_flag=0;
        }
    }
    Speed_AGV(v);
    return 0;
}

/*将微妙换算成秒和纳秒*/
int us_to_s_ns(int us, int* s)
{
	int ns = 0;
	*s = us/1000000;
	ns = us - (*s * 1000000);
	ns = ns*1000;
	return ns;
}

//--run_correction_xTheta2校正函数针对两个二维码之间分为车身扭转和横移两段进行校正
int run_correction_xTheta3(float v,float dis_x,float dis_theta,float distance,float agv_dir)
{
    float theta_cor=0;
    float alpha= Q_DuoLun_angle;//转角
    int limit_time=300000;
    int time_1=0,time_2=0, time_s = 0,time_ns = 0;
    float dis_plus=0;
    //ROS_INFO("v:%f dis_x:%f dis_theta:%f distance:%f   agv_dir:%f",v,dis_x,dis_theta,distance,agv_dir);
    if(fabs(agv_dir)!=180)
        theta_cor=dis_theta-agv_dir;//矫正角度：度
    else
        theta_cor=fabs(fabs(dis_theta)-agv_dir)*(-dis_theta)/fabs(dis_theta);//矫正角度：度
    float radius_cor=fabs(CAR_H/tan(alpha*pi/180));//车身扭转转弯半径:米
    time_1=pow(10,6)*fabs(radius_cor*theta_cor/180*pi/v);//第一段车身扭转时间
    if(theta_cor > 0)
        dis_plus=dis_x-radius_cor*(1-cos(theta_cor/180*pi));
    else
        dis_plus=dis_x+radius_cor*(1-cos(theta_cor/180*pi));
    time_2=pow(10,6)*fabs(dis_plus/(v*1000)/sin(alpha*pi/180));//第二段车身平移时间
    time_2=time_2*1;
    int dis_cor=(time_1+time_2)*v*0.001;//校正路径长度:mm
    if(dis_cor>distance-200)
    {
        time_1=time_1*(distance-200)/dis_cor;
        time_2=time_2*(distance-200)/dis_cor;
    }
    ROS_INFO("theta_cor:%f radius_cor:%f  time_1:%d   time_2:%d",theta_cor,radius_cor,time_1,time_2);
    //--前后轮异向，扭转车身角度
    if(theta_cor*v>0 && time_1 > limit_time)
    {
        wheels_syntropy_rotate(alpha,1);//前轮组右转
        wheels_syntropy_rotate(-alpha,2);//后轮组左转
        ROS_INFO("----front turn right and back turn left: delay %d us----",time_1);
        delay_us(time_1);
    }
    else if((theta_cor*v<0 && time_1 > limit_time))
    {
        wheels_syntropy_rotate(-alpha,1);//前轮组左转
        wheels_syntropy_rotate(alpha,2);//后轮组右转
        ROS_INFO("----front turn left and back turn left: delay %d us----",time_1);
        delay_us(time_1);
    }
    //--前后轮同向，矫正距离
    if(dis_plus*v>0 && time_2>limit_time)
    {
        wheels_syntropy_rotate(-alpha,0);
        ROS_INFO("----all turn left: delay %d us----",time_2);
        delay_us(time_2);
    }
    else if(dis_plus*v<0 && time_2>limit_time)
    {
        wheels_syntropy_rotate(alpha,0);
        ROS_INFO("----all turn right: delay %d us----",time_2);
        delay_us(time_2);
    }
    wheels_syntropy_rotate(0,0);//全部回正
    ROS_INFO("----all wheels back----");
    car_direct_run(v,0,0);
    return 0;
}

//--run_correction_xTheta2校正函数针对两个二维码之间分为车身扭转和横移两段进行校正
int run_correction_xTheta5(float v,float dis_x,float dis_theta,float distance,float agv_dir)
{
    float theta_cor=0;
    float alpha= Q_DuoLun_angle;//转角
    int limit_time=300000;
    int time_1=0,time_2=0, time_s = 0,time_ns = 0;
    float dis_plus=0,delta_x_right=0,delta_x_left=0;
    //ROS_INFO("v:%f dis_x:%f dis_theta:%f distance:%f   agv_dir:%f",v,dis_x,dis_theta,distance,agv_dir);
    if(fabs(agv_dir)!=180)
        theta_cor=dis_theta-agv_dir;//矫正角度：度
    else
        theta_cor=fabs(fabs(dis_theta)-agv_dir)*(-dis_theta)/fabs(dis_theta);//矫正角度：度
    float radius_cor = fabs(2*CAR_H/tan(du2rad(alpha)));//车身扭转转弯半径:米
    time_1=pow(10,6)*fabs(radius_cor*du2rad(theta_cor)/v);//第一段车身扭转时间
    if(theta_cor > 0){
        delta_x_right = 1.2*sin(theta_cor/180*pi)-radius_cor*(1-cos(du2rad(theta_cor)));
        delta_x_left = -(1.2*sin(theta_cor/180*pi)+radius_cor*(1-cos(du2rad(theta_cor))));
    }
    else if(theta_cor < 0){
        delta_x_left = -1.2*sin(theta_cor/180*pi)+radius_cor*(1-cos(du2rad(theta_cor)));
        delta_x_right = 1.2*sin(theta_cor/180*pi)+radius_cor*(1-cos(du2rad(theta_cor)));
    }
    if(dis_x+delta_x_right > 0){
        dis_plus=dis_x+delta_x_right;
    }
    else if(dis_x+delta_x_left < 0){
        dis_plus=dis_x+delta_x_left;
    }
    ROS_INFO("dis_x=%f theta_cor=%f delta_x_right=%f delta_x_left=%f dis_plus=%f",dis_x,theta_cor,delta_x_right,delta_x_left,dis_plus);
    time_2=pow(10,6)*fabs(dis_plus/(v*1000)/sin(alpha*pi/180));//第二段车身平移时间
    int dis_cor=(time_1+time_2)*v*0.001;//校正路径长度:mm
    if(dis_cor>distance-200)
    {
        time_1=time_1*(distance-200)/dis_cor;
        time_2=time_2*(distance-200)/dis_cor;
    }
    ROS_INFO("theta_cor:%f radius_cor:%f  time_1:%d   time_2:%d",theta_cor,radius_cor,time_1,time_2);
    if(dis_plus*v>0)
    {
        if(time_1 > limit_time)
        {
            if(theta_cor*v>0)
            {
                wheels_syntropy_rotate(-alpha,2);//后轮组左转
                wheels_syntropy_rotate(-alpha,2);//后轮组左转
                ROS_INFO("----back turn left: %d----",time_1);
				delay_us(time_1);
            }
            else
            {
                wheels_syntropy_rotate(-alpha,1);//前轮组左转
                wheels_syntropy_rotate(-alpha,1);//前轮组左转
                ROS_INFO("----front turn left: %d----",time_1);
				delay_us(time_1);
            }
        }
        if(time_2>limit_time)
        {
            wheels_syntropy_rotate(-alpha,0);
            wheels_syntropy_rotate(-alpha,0);
            ROS_INFO("----all turn left: %d----",time_2);
			delay_us(time_2);
        }
    }
    else if(dis_plus*v<0)
    {
        if(time_1 > limit_time)
        {
            if(theta_cor*v>0)
            {
                wheels_syntropy_rotate(alpha,1);//前轮组右转
                wheels_syntropy_rotate(alpha,1);//前轮组右转
                ROS_INFO("----front turn right: %d----",time_1);
				delay_us(time_1);
            }
            else
            {
                wheels_syntropy_rotate(alpha,2);//后轮组右转
                wheels_syntropy_rotate(alpha,2);//后轮组右转
                ROS_INFO("----back turn right: %d----",time_1);
				delay_us(time_1);
            }
        }
        if(time_2>limit_time)
        {
            wheels_syntropy_rotate(alpha,0);
            wheels_syntropy_rotate(alpha,0);
            ROS_INFO("----all turn right: %d----",time_2);
			delay_us(time_2);
        }
    }
    wheels_syntropy_rotate(0,0);//全部回正
    wheels_syntropy_rotate(0,0);//全部回正
    ROS_INFO("----all wheels back----");
    car_direct_run(v,0,0);
    return 0;
}


//--run_correction_codeband校正函数针对码带规划了两段圆弧路径进行校正
void run_correction_codeband(float v,float w,float dis_x,float dis_theta)
{
    ROS_INFO("v:%f w:%f dis_x:%f dis_theta:%f ",v,w,dis_x,dis_theta);
    float theta_cor=0;
    float dis_cor=0;
    static int adjust_mode=0;
    if(dis_theta<120 && dis_theta>60)//车头朝90度 
    {
        dis_cor=-dis_x;//车右偏dis_x为正
        dis_theta=dis_theta-90;
        theta_cor=dis_theta*dis_cor/fabs(dis_cor)*(-v)/fabs(v);//矫正角度：度,dis_theta逆时针偏为正
    }
    else if(dis_theta>-120 || dis_theta<-60)//车头-90度
    {
        dis_cor=dis_x;//车左偏dis_x为正 
        dis_theta=dis_theta+90;
        theta_cor=dis_theta*dis_cor/fabs(dis_cor)*v/fabs(v);//矫正角度：度,dis_theta顺时针偏为负       
    }
    ROS_INFO("dis_cor:%f theta_cor:%f ",dis_cor,theta_cor);
    float radius_cor=fabs(v/w);//转弯半径:米
    float alpha_2=acos((radius_cor*(1+cos(theta_cor*pi/180))-fabs(dis_cor)/1000)/2/radius_cor)*180/pi;//第二段圆弧：角度
    float alpha_1=alpha_2+theta_cor;//第一段圆弧：角度
    int time_1=pow(10,6)*alpha_1/180*pi/w;//第一段圆弧时间:单位us
    int time_2=pow(10,6)*alpha_2/180*pi/w;//第二段圆弧时间
    ROS_INFO("theta_cor:%f radius_cor:%f alpha_2:%f alpha_1:%f time_1:%d   time_2:%d",theta_cor,radius_cor,alpha_2,alpha_1,time_1,time_2);
    if(w==0)
    {
        car_adjust_run(v,0);
        return;
    }
    if(dis_cor>10)//车左偏
    {
        if(time_1>100000 && adjust_mode!=1)
        {
            car_adjust_run(v,w);//圆心在右边
            adjust_mode=1;
        }
        else if(time_1<=100000 && time_2>100000 && adjust_mode!=2)
        {
            car_adjust_run(v,-w);//圆心在左边
            adjust_mode=2;
        }
    }
    else if(dis_cor<-10)//车右偏
    {
        if(time_1>100000 && adjust_mode!=2)
        {
            car_adjust_run(v,-w);//圆心在左边
            adjust_mode=2;
        }
        else if(time_1<=100000 && time_2>100000 && adjust_mode!=1)
        {
            car_adjust_run(v,w);//圆心在右边
            adjust_mode=1;
        }
    }
    else
    {
            car_adjust_run(v,0);
            adjust_mode=0;
    }
}

//v:车速（米每秒）     w：角速度(弧度每秒)    dis_x：车偏移（车相对二维码右偏为正）    dis_theta：车偏角，角度（车相对于二维码逆偏为正）   distance：二维码间距(毫米)    agv_dir：车头朝向，角度
void shift_correction_codeband(float v,float w,float dis_x,float dis_theta)
{
    ROS_INFO("v:%f w:%f dis_x:%f dis_theta:%f ",v,w,dis_x,dis_theta);
    float theta_cor=0;
    float dis_cor=0;
    if(dis_theta<10 && dis_theta>-10)
    {
        dis_cor=dis_x;//车上偏dis_x为正
        theta_cor=dis_theta*dis_cor/fabs(dis_cor)*(-v)/fabs(v);//矫正角度：度,dis_theta逆时针偏为正
    }
    else if(dis_theta>170 || dis_theta<-170)
    {
        dis_cor=-dis_x;//车上偏dis_x为负
        dis_theta=fabs(fabs(dis_theta)-180)*dis_theta/fabs(dis_theta);
        theta_cor=dis_theta*dis_cor/fabs(dis_cor)*v/fabs(v);//矫正角度：度,dis_theta顺时针偏为正        
    }
    ROS_INFO("dis_cor:%f theta_cor:%f ",dis_cor,theta_cor);
    float radius_cor=fabs(v/w);//转弯半径:米
    float alpha_2=acos((radius_cor*(1+cos(theta_cor*pi/180))-fabs(dis_cor)/1000)/2/radius_cor)*180/pi;//第二段圆弧：角度
    float alpha_1=alpha_2+theta_cor;//第一段圆弧：角度
    int time_1=pow(10,6)*alpha_1/180*pi/w;//第一段圆弧时间:单位us
    int time_2=pow(10,6)*alpha_2/180*pi/w;//第二段圆弧时间
    ROS_INFO("theta_cor:%f radius_cor:%f alpha_2:%f alpha_1:%f time_1:%d   time_2:%d",theta_cor,radius_cor,alpha_2,alpha_1,time_1,time_2);
    if(w==0)
    {
        car_adjustSpeed_shift(v,0);
        return;
    }
    if(dis_cor>10)//车偏移上偏
    {
        if(time_1>100000)
        {
            car_adjustSpeed_shift(v,w);//圆心在车尾
            ROS_INFO("car_adjustSpeed_shift(v,w)");
        }
        else if(time_2>100000)
        {
            car_adjustSpeed_shift(v,-w);//圆心在车头
            ROS_INFO("car_adjustSpeed_shift(v,-w)");
        }
    }
    else if(dis_cor<-10)//车偏移下偏
    {
        if(time_1>100000)
        {
            car_adjustSpeed_shift(v,-w);//圆心在车头
            ROS_INFO("car_adjustSpeed_shift(v,-w)");
        }
        else if(time_2>100000)
        {
            car_adjustSpeed_shift(v,w);//圆心在车尾
            ROS_INFO("car_adjustSpeed_shift(v,w)");
        }
    }
    else
    {
            car_adjustSpeed_shift(v,0);
    }
}

//v:车速（米每秒）     w：角速度(弧度每秒)    dis_x：车偏移（车相对二维码右偏为正）    dis_theta：车偏角，角度（车相对于二维码逆偏为正）   distance：二维码间距(毫米)    agv_dir：车头朝向，角度
void shift_correction_yTheta(float v,float w,float dis_y,float dis_theta,float distance,float agv_dir)
{
    ROS_INFO("v:%f     w:%f   dis_y:%f    dis_theta:%f    distance:%f   agv_dir:%f",v,w,dis_y,dis_theta,distance,agv_dir);
    if(w==0)
    {
        car_direct_shift(v,0,1);
        return;
    }
    float theta_cor=(dis_theta-agv_dir)*v/fabs(v);//矫正角度：度
    float radius_cor=fabs(v/w);//转弯半径:米
    float alpha_2=acos((radius_cor*(1+cos(theta_cor*pi/180))-fabs(dis_y)/1000)/2/radius_cor)*180/pi;//第二段圆弧：角度
    float alpha_1=alpha_2+theta_cor*dis_y/fabs(dis_y);//第一段圆弧：角度
    int time_1=pow(10,6)*alpha_1/180*pi/w;//第一段圆弧时间:单位us
    int time_2=pow(10,6)*alpha_2/180*pi/w;//第二段圆弧时间
    int dis_cor=(time_1+time_2)*v*0.001;//校正路径长度:mm
    ROS_INFO("theta_cor:%f radius_cor:%f alpha_2:%f alpha_1:%f time_1:%d   time_2:%d    dis_cor:%d",theta_cor,radius_cor,alpha_2,alpha_1,time_1,time_2,dis_cor);
    if(dis_cor>distance-300)
    {
        time_1=time_1*(distance-300)/dis_cor;
        time_2=time_2*(distance-300)/dis_cor;
    }
    ROS_INFO("theta_cor:%f radius_cor:%f alpha_2:%f alpha_1:%f time_1:%d   time_2:%d",theta_cor,radius_cor,alpha_2,alpha_1,time_1,time_2);
    if(dis_y>0)//车后偏
    {
        if(time_1>300000)
        {
            ROS_INFO("YUAN XIN ZAI QIAN-------%d",time_1);
            car_adjustSpeedAngle_shift(v,-w);//圆心在前
            usleep(time_1);
        }
        if(time_2>300000)
        {
            ROS_INFO("YUAN XIN ZAI HOU-------%d",time_2);
            car_adjustSpeedAngle_shift(v,w);//圆心在后
            usleep(time_2);
        }
    }
    else if(dis_y<0)//车前偏
    {
        if(time_1>300000)
        {
            ROS_INFO("YUAN XIN ZAI HOU-------%d",time_1);
            car_adjustSpeedAngle_shift(v,w);//圆心在后
            usleep(time_1);
        }
        if(time_2>300000)
        {
            ROS_INFO("YUAN XIN ZAI QIAN-------%d",time_2);
            car_adjustSpeedAngle_shift(v,-w);//圆心在前
            usleep(time_2);
        }
    }
    car_direct_shift(v,0,0);
    return;
}

//shift_correction_yTheta2　扭转偏移两段矫正
//v:车速（米每秒）     w：角速度(弧度每秒)    dis_y：车偏移（车相对二维码右偏为正）    dis_theta：车偏角，角度（车相对于二维码逆偏为正）   distance：二维码间距(毫米)    agv_dir：车头朝向，角度
void shift_correction_yTheta2(float v,float dis_y,float dis_theta,float distance,float agv_dir)
{
    ROS_INFO("v:%f　dis_y:%f　dis_theta:%f  distance:%f  agv_dir:%f",v,dis_y,dis_theta,distance,agv_dir);
    float alpha = 10,w = 0.1,dis_plus = 0;
    int time_limit = 300000;
    float theta_cor=(dis_theta-agv_dir)*v/fabs(v);//矫正角度：度
    float radius_cor=fabs(v/w);//转弯半径:米
    int time_1=pow(10,6)*theta_cor/180*pi/w;//第一段圆弧时间:单位us
    if(theta_cor > 0 && time_1 > time_limit){
        dis_plus = dis_y + radius_cor*(1-cos(du2rad(theta_cor)));
    }
    else if(theta_cor < 0 && time_1 > time_limit){
        dis_plus = dis_y - radius_cor*(1-cos(du2rad(theta_cor)));
    }
    else{
        dis_plus = dis_y ;
    }
    int time_2=pow(10,6)*dis_plus/(v*1000)/sin(du2rad(alpha));//第二段圆弧时间
    int dis_cor=(time_1+time_2)*v*0.001;//校正路径长度:mm
    ROS_INFO("theta_cor:%f radius_cor:%f alpha:%f time_1:%d  time_2:%d  dis_cor:%d",theta_cor,radius_cor,alpha,time_1,time_2,dis_cor);
    if(dis_cor>distance-300)
    {
        time_1=time_1*(distance-300)/dis_cor;
        time_2=time_2*(distance-300)/dis_cor;
    }
    ROS_INFO("theta_cor:%f radius_cor:%f alpha:%f time_1:%d   time_2:%d",theta_cor,radius_cor,alpha,time_1,time_2);
    if(theta_cor > 0 && time_1 > time_limit)//车逆偏
    {
        ROS_INFO("YUAN XIN ZAI QIAN-------%d",time_1);
        car_adjustSpeedAngle_shift(v,-w);//圆心在前
        delay_us(time_1);
    }
    else if(theta_cor < 0 && time_1 > time_limit)
    {
        ROS_INFO("YUAN XIN ZAI HOU-------%d",time_1);
        car_adjustSpeedAngle_shift(v,w);//圆心在后
        delay_us(time_2);
    }
    Shift_AGV(v);
    if(dis_plus > 0 && time_2 > time_limit)//车后偏
    {
            ROS_INFO("shun zhi zhen zhuan alpha-------%d",time_2);
            wheels_rotate_shift90(alpha,0);//向前矫正
            usleep(time_2);
    }
    else if(dis_plus < 0 && time_2 > time_limit)//车后偏
    {
        ROS_INFO("ni zhi zhen zhuan alpha-------%d",time_2);
        wheels_rotate_shift90(-alpha,0);//向后矫正
        usleep(time_2);
    }
    car_direct_shift(v,0,0);
    return;
}

//原地校正：输入参数为当前运动模式
int car_spot_correction(int mode=1)
{
	static int corStep=0;
    static int findcode=0;
    const int time=200;
    if(codeCur == 0)
    {
        ROS_INFO("code_cur=%d findcode=%d",codeCur,findcode);
        if(findcode<time || (findcode>=time*3 && findcode<time*4))
        {
            car_direct_run(0.1,1,1);
            findcode++;
        }
        else if(findcode>=time && findcode<time*3)
        {
            car_direct_run(-0.1,1,1);
            findcode++;
        }
        else if((findcode>=time*4 && findcode<time*5) && (findcode>=time*7 && findcode<time*8))
        {
            car_direct_shift(-0.1,1,1);
            findcode++;
        }
        else if(findcode>=time*5 && findcode<time*7)
        {
            car_direct_shift(-0.1,1,1);
            findcode++;
        }
        else
        {
            car_direct_run(0,1,1);
            ROS_WARN("Can not find the code!!!");
            findcode=0;
            return -1;
        }  
        return 0;     
    }
	if(corStep==0)//角度校正
    {
        if(wheel_direction != READY_TURN  &&  wheel_direction==READY_RUN)
        {
            wheels_rotate_bfturn(1);
        }
        if((fabs(fabs(angleCur)-fabs(point_cur.direction)))  > 0.5 && pointNoCur==0)
        {
            corStep+=adjust_w(point_cur.direction);
        }
        else if(pointNoCur!=0 && (fabs(fabs(angleCur)-fabs(angleCur_agv)))  > 0.5)
        {
            corStep+=adjust_w(angleCur_agv);
        }
        else
        {
            corStep=1;
        }
    }
    else if(corStep==1)//针对前进方向的左右偏移校正
    {
        if((mode==2 || mode==3) && (fabs(yPositionT_agv) > 5 || fabs(yPositionCur_agv) > 5))
        {
            if(pointNoCur==0)
                corStep+=adjust_x(yPositionT_agv,0,point_cur.direction);
            else
                corStep+=adjust_x(yPositionCur_agv,0,angleCur_agv);
            //ROS_INFO("------- yPositionT_agv = %d yPositionCur_agv=%d Tdirection=%d angleCur_cor=%f----------",yPositionT_agv,yPositionCur_agv,point_cur.direction,angleCur_agv);
        }
        else if((mode!=2 && mode!=3) && (fabs(xPositionT_agv) > 5|| fabs(yPositionCur_agv) > 5))
        {
            if(pointNoCur==0)
                 corStep+=adjust_x(xPositionT_agv,1,point_cur.direction);
            else
                 corStep+=adjust_x(xPositionCur_agv,1,angleCur_agv);
            //ROS_INFO("------- xPositionT_agv = %d xPositionCur_agv=%d Tdirection=%d angleCur_cor=%f----------",xPositionT_agv,xPositionCur_agv,point_cur.direction,angleCur_agv);
        }
        else
        {
            corStep=2;
        }   
    }
    else if(corStep==2)//针对前进方向的前后偏移校正 
    {
    	if((mode==2 || mode==3) && (fabs(xPositionT_agv) > 5 || fabs(xPositionCur_agv) > 5))
        {
            if(pointNoCur==0)
                corStep+=adjust_x(xPositionT_agv,1,point_cur.direction);
            else
                corStep+=adjust_x(xPositionCur_agv,1,angleCur_agv);
            //ROS_INFO("------- yPositionT_agv = %d  Tdirection=%d angleCur_cor=%f----------",yPositionT_agv,point_cur.direction,angleCur_agv);
        }
        else if((mode!=2 && mode!=3) && (fabs(yPositionT_agv) > 5 || fabs(yPositionCur_agv) > 5))
        {
            if(pointNoCur==0)
                 corStep+=adjust_x(yPositionT_agv,0,point_cur.direction);
            else
                 corStep+=adjust_x(yPositionCur_agv,0,angleCur_agv);
            //ROS_INFO("------- xPositionT_agv = %d Tdirection=%d angleCur_cor=%f----------",xPositionT_agv,point_cur.direction,angleCur_agv);
        }
        else
        {
            corStep=3;
        }
    } 
    else
    {
        if(mode==2 ||mode==3)
            wheels_rotate_bfrun(90,1,1);
        else
            wheels_rotate_bfrun(0,1,1);
        car_stop();
        Motec_Enable();
        corStep=4;
    }
    if(corStep<4)//未完成校正返回0
    	return 0;
   	else
   	{
   		corStep=0;//完成校正返回1
	   	return 1;
 	}
    	
}

/*
速度改变函数
*/
void change_Tspeed(float *speed, int run_mode)
{
	if(run_mode == 1)//前进
		{
         	if((codeCur == 60)||(codeCur == 64))
				{	
					*speed = 0.2;
				}
			else
				{	
					*speed = 0.5;
				}

		}
	if(run_mode == 4)//后退
		{
         	if((codeCur == 62)||(codeCur == 53))
				{	
					*speed = 0.2;
				}
			else
				{	
					*speed = 0.5;
				}	
		}
}

int  QStart_angle=0,res=0;
//--自动行驶函数
void car_auto_run(void)
{
    int i,j,k;
    int num_point=0;//该条路径的二维码数量
    static int last_code=0;//上一个二维码
    static int last_run_mode=0;//上一个运动模式
    float theta_able = 10; //可矫正角度
    float adjust_distheta = 2; //需要矫正的最小角度跟距离
    float adjust_disx = 15;
    float adjust_disy = 15;
    static int step=0;
    float jiache_delayTime=7.5;//6.2
	int res = 0;

    if(mode_auto==0)                                                                                                                  {
		//set_acc(8);
        return;
    }
    else if(mode_auto==1)
    {
		//set_acc(60);
		set_acc(6);
		Set_TKmotor_acc(6,2000000);
		Set_TKmotor_acc(8,2000000);
		Set_TKmotor_decc(6,2000000);
		Set_TKmotor_decc(8,2000000);
        int Tcode=point_cur.code;
        int Trun_mode=point_cur.run_mode;
		//printf("run_mode = %d\n",point_cur.run_mode);
        int Tdirection=point_cur.direction;
        int Tlength=point_cur.length;
        float Tspeed= Qwalk_speed;//point_cur.speed;
        
		//ROS_INFO("auto_mode = %d,run_mode = %d ,seep = %d",mode_auto,Trun_mode,step);
        if(last_code!=Tcode || last_run_mode!=Trun_mode)//新码或者新的运动模式
        {
            AGVupdate_ready=0;
            step=0;
            if(pointNoCur == 0)
            {
               // odometer_start();
                step=100;
            }
        }  
 
        change_Tspeed(&Tspeed,Trun_mode);
        //--根据二维码校正原地校正位置：第一个码矫正位置/测量夹车前校正位置
        if(step == 100)
        {
            //ROS_INFO("first code adjust!");
			#if 0
            	step=step+car_spot_correction(Trun_mode);
			#else
				step += 1;
				delay_sec(1);
			#endif
            last_code=Tcode;
            last_run_mode=Trun_mode;
            if(step>100)
            {
                car_stop();
                ROS_INFO("=========wait 1 s=======");
                //sleep(1);
				if(Trun_mode <= 4)
					{
						//odometer_stop();
						odometer_start();
						//CarMileage_str.offset_x += xPositionCur_agv/1000.0;
						//printf("初始X偏移量====%f\n",CarMileage_str.offset_x);
						//printf("初始角度 ===%f\n",angleCur_agv);
						QStart_angle = angleCur_agv;
					}
               // if(pointNoCur == 0)
               //    step=0;
               // else
                    step=1;
            }
            return; 
        } 
            
        //--根据运动指令来进行运动   
        switch(Trun_mode)
        {
            case 0:         //停止    ***
                ROS_INFO("This is stop code!,code num:%d",Tcode);
                car_stop();
                AGVupdate_ready=1;
                if(wheel_direction == SHIFT_ADJUST)
                    wheels_rotate_bfrun(90,1,1);
                else if(wheel_direction == RUN_ADJUST)
                    wheels_rotate_bfrun(0,1,1);
            break;
            case 1:         //直线前进  ***
                //ROS_INFO("--------------case 1:step=%d------------",step);
                if(step==0)
                {
                    if(wheel_direction!=RUN_ADJUST && wheel_direction!=READY_RUN)
                        step=100;
                    else 
                        step=1;
                }
                else if(step==1)
                {
					printf("===========theta_able:%f\n",theta_able);
					//printf("===========fabs((fabs(angleCur) - fabs(Tdirection)) = %n",fabs(fabs(angleCur) - fabs(Tdirection)));
                    if(codeCur!=998 && (fabs(fabs(angleCur)-fabs(Tdirection)))  > theta_able)
                    {
						printf("car_stop(): angleCur:%f,Tdirection:%f,theta_able:%f\n",angleCur,Tdirection,(float)theta_able);
                        car_stop();
                        return;
                    }
                    if(wheel_direction!=RUN_ADJUST && wheel_direction!=READY_RUN)
                    {
						printf("car_stop(): wheel_direction%d,RUN_ADJUST%d,READY_RUN%d",wheel_direction,RUN_ADJUST,READY_RUN);
                        car_stop();
                        sleep(0.5);
                        wheels_rotate_bfrun(0,1,1);
                    }
					printf("codeCur = %d\n",codeCur);   
                    if(codeCur==4)
                        step=3;
                    else
                        step=2;
                }
                else if(step==2)
                {
                    //if(fabs(xPositionCur_agv) > adjust_disx || (fabs(fabs(angleCur)-fabs(Tdirection))) > adjust_distheta) 
                    {
						 //run_correction_xTheta2((float)Tspeed,(float)xPositionCur_agv,angleCur,(float)Tlength,(float)Tdirection);
						angle_dir = 1;
						Speed_AGV(Qwalk_speed);
						//res = car_walk_correction(0,(float)Tdirection);
						if(res == -3)
							{
								car_stop();
								step = 4;
								return ;
							}
                        //run_correction_xTheta((float)Tspeed,adjust_turn,(float)xPositionCur_agv,angleCur,(float)Tlength,(float)Tdirection);
						//car_direct_run(Tspeed,0,0);
                    }
                    //step=3;
                    AGVupdate_ready=1;
                } 
               else if(step==3)
               {
                   if(codeCur==998)
                            run_correction_codeband((float)Tspeed,0.03,(float)xPositionCur,angleCur);
                    else
                            car_direct_run(Tspeed,0,0);
                    AGVupdate_ready=1;
               }
               else
                {
                    car_direct_run(Tspeed,0,0);
                }
            break;
            case 2:   //左横移
                if(step==0)
                {
                    if(wheel_direction!=RUN_ADJUST && wheel_direction!=READY_SHIFT)
                        step=100;
                    else 
                        step=1;
                }
                else if(step==1)
                {
                    if(codeCur!=998 && (fabs(fabs(angleCur)-fabs(Tdirection)))  > (float)theta_able)
                    {
                        car_stop();
                        return;
                    }
                    if(wheel_direction!=SHIFT_ADJUST && wheel_direction!=READY_SHIFT)
                    {
                        car_stop();
                        sleep(0.5);
                        wheels_rotate_bfrun(90,1,1);
                    }   
                    step=2;
                }
                else if(step==2)
                {
                    if(codeCur!=998 && (fabs(fabs(angleCur)-fabs(Tdirection)))  > (float)theta_able)
                    {
                        car_stop();
                        return;
                    }
                    if(codeCur==998) 
                    {
                        ROS_INFO("xPositionCur:%d angleCur:%f Tdirection:%d",xPositionCur,angleCur,Tdirection);
                        shift_correction_codeband((float)Tspeed,0.02,(float)xPositionCur,angleCur);
                    }
                    else
                    {
                        car_direct_shift(Tspeed,0,0);
                    }
                    AGVupdate_ready=1;
                } 
            break;
            case 3:    //右横移
                if(step==0)
                {
                    if(wheel_direction!=RUN_ADJUST && wheel_direction!=READY_SHIFT)
                        step=100;
                    else 
                        step=1;
                }
                else if(step==1)
                {
                    if(codeCur!=998 && (fabs(fabs(angleCur)-fabs(Tdirection)))  > (float)theta_able)
                    {
                        car_stop();
                        return;
                    }
                    if(wheel_direction!=SHIFT_ADJUST && wheel_direction!=READY_SHIFT)
                    {
                        car_stop();
                        sleep(0.5);                   
                        wheels_rotate_bfrun(90,1,1);
                    } 
                    step=2;
                }
                else if(step==2)
                {
                        if(codeCur==998) 
                        {
                            ROS_INFO("xPositionCur:%d angleCur:%f Tdirection:%d",xPositionCur,angleCur,Tdirection);
                            // shift_correction_yTheta((float)-Tspeed,0.02,(float)yPositionT_agv,angleCur,(float)Tlength,(float)Tdirection);
                            shift_correction_codeband((float)-Tspeed,0.02,(float)xPositionCur,angleCur);
                        }
                        else
                        {
                            car_direct_shift(-Tspeed,0,0);
                        }
                        AGVupdate_ready=1;
                } 
            break;
            case 4:   //直线后退    ***
               if(step==0)
               {
                    if(wheel_direction!=RUN_ADJUST && wheel_direction!=READY_RUN)
                        step=100;
                    else 
                        step=1;
                }
                else if(step==1)
                {
                    if(codeCur!=998 && (fabs(fabs(angleCur)-fabs(Tdirection)))  > (float)theta_able)
                    {
                        car_stop();
                        return;
                    }
                    if(wheel_direction!=RUN_ADJUST && wheel_direction!=READY_RUN)
                    {
                        car_stop();
                        sleep(0.5);
                        wheels_rotate_bfrun(0,1,1);
                    }
                    if(codeCur==4)
                        step=3;
                    else
                        step=2;
                }
                else if(step==2)
                {
                //  ROS_INFO("F_xPa%f",fabs(xPositionCur_agv)) ;
				//	ROS_INFO("ad%f",adjust_disx);
				//	ROS_INFO("F_aC%f",fabs(angleCur));
				//	ROS_INFO("F_T%f",fabs(Tdirection));
				//	ROS_INFO("F f-f%f",fabs(fabs(angleCur)-fabs(Tdirection)));
				//	ROS_INFO("F_ad%f",adjust_distheta); 
                //    if(fabs(xPositionCur_agv) > adjust_disx || (fabs(fabs(angleCur)-fabs(Tdirection))) > adjust_distheta) 
                    {
				//	   printf("+++++++++真实速度====%f",Tspeed);
                       //run_correction_xTheta2((float)-Tspeed,(float)xPositionCur_agv,angleCur,(float)Tlength,(float)Tdirection);
				        angle_dir = -1;
						Speed_AGV(-Qwalk_speed);
						//car_walk_correction(0,(float)Tdirection);
						if(res == -3)
							{
								car_stop();
								step = 4;
								return ;
							}
                       // run_correction_xTheta((float)-Tspeed,adjust_turn,(float)xPositionCur_agv,angleCur,(float)Tlength,(float)Tdirection);
						//car_direct_run(Tspeed,0,0);
                    }
                    //step=3;
                    AGVupdate_ready=1;
                } 
               else if(step==3)
               {
                   if(codeCur==998)
                            run_correction_codeband((float)-Tspeed,0.03,(float)xPositionCur,angleCur);
                    else
                            car_direct_run(-Tspeed,0,0);
                    AGVupdate_ready=1;
               }
               else
                {
                    car_direct_run(-Tspeed,0,0);
                }   
            break;
            case 5:   //停止 逆时针转 直线前进    ***
                if(step==0)
                {
                    car_stop();
                    step=100;                   
                } 
                else if(step==1)
                    step+=car_auto_turn(90,0.2);
                else if(step==2)
                {
                    wheels_rotate_bfrun(0,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_run(0.2,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 6:   //停止 逆时针转 左横移
                if(step==0)
                {
                    car_stop();
                    step=100;
                } 
                else if(step==1)
                    step+=car_auto_turn(90,Tspeed);
                else if(step==2)
                {
                    wheels_rotate_bfrun(90,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_shift(Tspeed,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 7:   //停止 逆时针转 右横移
                if(step==0)
                {
                    car_stop();
                    step=100;
                } 
                else if(step==1)
                    step+=car_auto_turn(90,Tspeed);
                else if(step==2)
                {
                    wheels_rotate_bfrun(90,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_shift(-Tspeed,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 8:   //停止 逆时针转 直线后退    ***
                if(step==0)
                {
                    car_stop();
                    step=100;
                } 
                else if(step==1)
                    step+=car_auto_turn(90,0.2);
                else if(step==2)
                {
                    wheels_rotate_bfrun(0,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_run(-0.2,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 9:   //停止 顺时针转 直线前进
                if(step==0)
                {
                    car_stop();
                    step=100;
                } 
                else if(step==1)
                    step+=car_auto_turn(-90,0.2);
                else if(step==2)
                {
                    wheels_rotate_bfrun(0,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_run(0.2,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 10:   //停止 顺时针转 左横移
                if(step==0)
                {
                    car_stop();
                    step=100;
                } 
                else if(step==1)
                    step+=car_auto_turn(-90,Tspeed);
                else if(step==2)
                {
                    wheels_rotate_bfrun(90,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_shift(Tspeed,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 11:   //停止 顺时针转 右横移
                if(step==0)
                {
                    car_stop();
                    step=100;
                } 
                else if(step==1)
                    step+=car_auto_turn(-90,Tspeed);
                else if(step==2)
                {
                    wheels_rotate_bfrun(90,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_shift(-Tspeed,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 12:   //停止 顺时针转 直线后退   ***
                if(step==0)
                {
                    car_stop();
                    step=100;
                }   
                else if(step==1)
                    step+=car_auto_turn(-90,0.2);
                else if(step==2)
                {
                    wheels_rotate_bfrun(0,1,1);
                    step=3;
                }
                else if(step==3)
                {
                    car_direct_run(-0.2,0,0);
                    AGVupdate_ready=1;
                }
            break;
            case 13:   //停止 举升或夹车 直线前进
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lift();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                    car_direct_run(Tspeed,0,1);
                    AGVupdate_ready=1;
                 }
            break;
            case 14:   //停止 举升或夹车 左横移
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lift();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                    car_direct_shift(Tspeed,0,1);
                    AGVupdate_ready=1;
                 }
            break;
            case 15:   //停止 举升或夹车 右横移
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lift();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                    car_direct_shift(-Tspeed,0,1);
                    AGVupdate_ready=1;
                 }
            break;
            case 16:   //停止 举升或夹车 直线后退
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lift();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                    car_direct_run(-Tspeed,0,1);
                    AGVupdate_ready=1;
                 }
            break;
            case 17:   //停止 举升或夹车
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lift();
                     sleep(jiache_delayTime);
                     step = 2;
                     AGVupdate_ready=1;
                 }
            break;
            case 18:   //停止 卸载 直线前进
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lay_down();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                    if(codeCur==998)
                            run_correction_codeband((float)Tspeed,0.03,(float)xPositionCur,angleCur);
                    else
                            car_direct_run(Tspeed,0,0);
                    AGVupdate_ready=1;
                 }
            break;
            case 19:   //停止 卸载 左横移
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lay_down();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                     car_direct_shift(Tspeed,0,1);
                     AGVupdate_ready=1;
                 }
            break;
            case 20:   //停止 卸载 右横移
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lay_down();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                     car_direct_shift(-Tspeed,0,1);
                     AGVupdate_ready=1;
                 }
            break;
            case 21:   //停止 卸载 直线后退
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lay_down();
                     sleep(jiache_delayTime);
                     step = 2;
                 }
                 else if(step == 2)
                 {
                     if(codeCur == 998)
                            run_correction_codeband((float)-Tspeed,0.03,(float)xPositionCur,angleCur);
                      else
                            car_direct_run(-Tspeed,0,0);
                     AGVupdate_ready=1;
                 }
            break;
            case 22:   //停止 卸载
                if(step == 0)
                {
                    car_stop();
                    step = 1;
                }
                else if(step == 1)
                 {
                     car_lay_down();
                     sleep(jiache_delayTime);
                     step = 2;
                     AGVupdate_ready=1;
                 }
            break;
            case 23:   //测量 提升 直线前进
                if(step==0)
                {
                     car_stop();
                     step = 100;
                     return;
                }
                else if(step==1)
                {
                    if(!wheel_aim())
                        step = 2;
                    else
                        return;
                }
                else if(step==2)
                {
                    car_lift();
                    sleep(jiache_delayTime);
                    step = 3;
                }
                else if(step==3)
                {
                    car_direct_run(Tspeed,0,1);
                    AGVupdate_ready=1;
                }
            break;
            case 24:   //测量 提升 左横移
                if(step==0)
                {
                     car_stop();
                     step = 100;
                     return;
                }
                else if(step==1)
                {
                    if(!wheel_aim())
                        step = 2;
                    else
                        return;
                }
                else if(step==2)
                {
                    car_lift();
                    sleep(jiache_delayTime);
                    step = 3;
                }
                else if(step==3)
                {
                    car_direct_shift(-Tspeed,0,1);
                    AGVupdate_ready=1;
                }
            break;
            case 25:   //测量 提升 右横移
                if(step==0)
                {
                     car_stop();
                     step = 100;
                     return;
                }
                else if(step==1)
                {
                    if(!wheel_aim())
                        step = 2;
                    else
                        return;
                }
                else if(step==2)
                {
                    car_lift();
                    sleep(jiache_delayTime);
                    step = 3;
                }
                else if(step==3)
                {
                    car_direct_shift(-Tspeed,0,1);
                    AGVupdate_ready=1;
                }
            break;
            case 26:   //测量 提升 直线后退
                if(step==0)
                {
                     car_stop();
                     mode_auto=0;
                     step = 1;
                     
                }
                else if(step==1)
                {
                    if(!wheel_aim())
                        step = 2;
                    else
                    {
                        mode_auto=0;
                        return;
                    }    
                }
                else if(step==2)
                {
                    car_lift();
                    sleep(jiache_delayTime);
                    step = 3;
                }
                else if(step==3)
                {
                	if(codeCur == 998) 
                    {
                    	run_correction_codeband((float)-Tspeed,0.02,(float)xPositionCur,angleCur);
                    }
                    else
                    {
                    	car_direct_run(-Tspeed,0,1);
                    }
                    AGVupdate_ready=1;
                }
            break;
            case 27:   //充电
                if(step==0)
                {
                    car_stop();
                    step = 1;
                    AGVupdate_ready=1;
                } 
            break;

        }
        last_code=Tcode;
        last_run_mode=Trun_mode;
        //ROS_INFO("codeCur:%d TCode:%d Run-mode:%d Direction:%d Length:%d Speed:%f step:%d",codeCur,Tcode,Trun_mode,Tdirection,Tlength,Tspeed,step);
    }
}
