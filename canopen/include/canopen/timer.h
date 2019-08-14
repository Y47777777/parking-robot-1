#ifndef ___TIMER___
#define ___TIMER___

#include <stdio.h>
#include <termios.h>



extern int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
extern void stop_timer(void);
extern void start_timer(void);
extern void timer_deal(int timer);
extern void timer_init(void);
extern int timer_open(void);
extern void timer_close(int fd);
void delay_sec_nsec(int sec,int nsec);
void delay_sec(int sec);
void delay_us(int time);
#endif