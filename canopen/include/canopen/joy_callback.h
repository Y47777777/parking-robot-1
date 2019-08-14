#ifndef ___JOY_CALLBACK___
#define ___JOY_CALLBACK___

#include <sensor_msgs/Joy.h>       //手柄头文件
#include <geometry_msgs/Twist.h>

#define L_ganLR float(msg->axes[0])   //左遥感　左１　右－１
#define L_ganUD float(msg->axes[1])  //左遥感　上１　下－１
#define LT float(msg->axes[2])       //连续按键ＬＴ　１～～-１
#define R_ganLR float(msg->axes[3]) //右遥感　左１　右－１
#define R_ganUD float(msg->axes[4]) //右遥感　上１　下－１
#define RT float(msg->axes[5])        //连续按键RＴ　１～～-１
#define L_anLR int(msg->axes[6])   //左按键　左１　右－１
#define L_anUD int(msg->axes[7]) //左按键　上１　下－１

#define an_A int(msg->buttons[0])  //按键Ａ
#define an_B int(msg->buttons[1])   //按键B
#define an_X int(msg->buttons[2]) //按键X
#define an_Y int(msg->buttons[3]) //按键Y
#define an_LB int(msg->buttons[4])  //按键LB
#define an_RB int(msg->buttons[5])  //按键RB
#define an_BACK int(msg->buttons[6])  //按键BACK

#define CAR_H   1.200f  //前后轮距离　一半  无伸长时前后轮距2.4米
#define CAR_L   0.382f   //左右轮距离　一半
#define WHEEL_L 0.132f   //轮间距长　一半
#define CAR_W   1.26f //对角线长

#define PCAN_DEVICE     PCAN_USBBUS1
#define READY_RUN      0
#define READY_SHIFT      90
#define READY_TURN      78
#define RUN_ADJUST      999
#define SHIFT_ADJUST      998
#define ROTARY_TIME      3.5
#define SHIFT_TIME      5


struct POINT_MSG
{
        int code;
        int direction;
        int length;
        int run_mode;
        float speed;
        int reserve;
};


extern int motec_disable_enable ;   //驱动电机失能使能标志:0为失能，1为使能
extern int wheel_direction;   //轮组方向
extern int mode_auto;
extern int AGVupdate_ready;
extern int lift_cmd;
extern float angleCur; //倍加福角度
extern int angle_dir;
extern float Qwalk_speed;

void wheels_rotate_bfrun(float angle,int disable,int time);
void wheels_syntropy_rotate(float angle,int mode);
void wheels_rotate_shift90(float angle,int disable);
void wheels_rotate_adjust(float angle,int disable);
void wheels_rotate_bfturn(int time);
void car_direct_run(float v,int disable,int time);
void car_direct_shift(float v,int disable,int time);
void car_stop(void);
void car_turn(float w);
void scannerCallback(const geometry_msgs::Twist::ConstPtr& msg);
void traceCallback(const geometry_msgs::Twist::ConstPtr& msg);
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
void calculation_bf_autoturn(float angle,float *angleCur_cor,float *angleCur_tar,bool *flag_180);
int car_auto_turn(float angle,float speed_max);
void car_adjust_run(float v,float w);
void car_adjustSpeedAngle_shift(float v,float w);
void car_adjustSpeed_shift(float v,float w);
int adjust_x(float dis,int dir,float codeDir);
int adjust_w(float angle);
void run_correction_xTheta(float v,float w,float dis_x,float dis_theta,float distance,float agv_dir);
extern int run_correction_xTheta2(float v,float dis_x,float dis_theta,float distance,float agv_dir);
void run_correction_codeband(float v,float w,float dis_x,float dis_theta);
void shift_correction_yTheta(float v,float w,float dis_y,float dis_theta,float distance,float agv_dir);
void shift_correction_codeband(float v,float w,float dis_x,float dis_theta);
void shift_correction_codeband_1(float v,float w,float dis_x,float dis_theta);
void car_auto_run(void);


#endif
