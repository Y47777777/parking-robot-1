#ifndef __SCANNER__
#define __SCANNER__

#include "canopen/type.h"

extern int code_cur,xPosition_cur,yPosition_cur;//当前二维码信息
extern float angle_cur;
extern int orderNo_cur,pointNo_cur;//当前任务号，当前路径点号
extern int updateCode_ready;

extern int test1[][6];
extern int test_1;
extern int test2[13][6];
extern int test_2;
extern int test;
extern int test_num;


extern u8 scanner_initial(void);
extern void scanner_data_process(void);




#endif

