#ifndef __TECHMOTORCTL_H__
#define __TECHMOTORCTL_H__

#include  "canopen/type.h"


int tech_start_node(int slaveID);

int tech_ready_switch_on(int slaveID);//改变为ready to switch on 状态

int tech_switch_on(int slaveID);//改变为 Switch on状态

int tech_enable_operation(int slaveID);//改变为Enable operation 状态

int tech_set_control_mode(int slaveID, u32 mode);//选择控制模式

int tech_set_position(int slaveID, int plusNum);//发送目标位置

int tech_set_speed(int slaveID, int speed);//发送目标速度

int tech_set_acc(int slaveID,int acceleration);
void TK_position_Enquire(void); 

int tech_set_arr(int slaveID);

int tech_set_arr_fast(int slaveID);

int tech_start(int slaveID);//启动指令

void tech_check_turn(int slaveID);

void tect_motor_init(void);

void tect_motor_reinit(int slaveID);

void tech_turn(int slaveID, float angle);   //每次叠加跑一个角度

void tech_turn_ex(int slaveID, float angle);   //只接跑到指定点

void tech_turn_ex_slow(int slaveID, float angle);

void tech_turn_right(int slaveID, float angle);   //每次叠加跑一个角度

void tech_turn_left(int slaveID, float angle);

void tech_turn_right_ex(int slaveID, float angle);   //只接跑到指定点

void tech_turn_left_ex(int slaveID,  float angle);

#endif
