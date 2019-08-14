#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <ros/ros.h>
#include "canopen/sport_control.h"
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <ros/ros.h>
// #include <boost/asio.hpp>                  //包含boost库函数
// #include <boost/bind.hpp>

// using namespace std;
// using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

struct timeval tpstart,tpend;
void stop_timer(void);
void start_timer(void);
void timer_deal(int timer)
{
    static unsigned int count=0;
    long unsigned int timeused;
	
	if(timer != SIGALRM)
		return;
    if(control_cycle && (count>=15))
    {
        count = 0;
        send_flag=1;
        gettimeofday(&tpend,0);
        timeused=(tpend.tv_sec-tpstart.tv_sec)*1000000+tpend.tv_usec-tpstart.tv_usec;
        ROS_INFO("used time:%lu\n",timeused);
    }
    count++;
	tpstart = tpend;
	signal(SIGALRM,timer_deal);
	return;
}
 
void stop_timer(void) 
{ 
	struct itimerval value; 
	value.it_value.tv_sec = 0; 
	value.it_value.tv_usec = 0; 
	value.it_interval = value.it_value; 
	setitimer(ITIMER_REAL, &value, NULL); 
}
 
void start_timer(void) 
{ 
	struct itimerval value; 
	value.it_value.tv_sec = 0; 
	value.it_value.tv_usec = 10; 
	value.it_interval.tv_sec = 0;
	value.it_interval.tv_usec = 1000;
	setitimer(ITIMER_REAL, &value, NULL); 
}


void timer_init(void)
{
    signal(SIGALRM,timer_deal);
	start_timer();//定时35ms
	gettimeofday(&tpstart,0);
}


int timer_open(void)
{
	int fd = open ("/dev/rtc", O_RDONLY);
	if(fd < 0)
	{
			ROS_INFO("open");
			exit(errno);
	}

	// /*Set the freq as 4Hz*/
	if(ioctl(fd, RTC_IRQP_SET,67) < 0)
	{
			ROS_INFO("ioctl(RTC_IRQP_SET)");
			close(fd);
			exit(errno);
	}
	// /* Enable periodic interrupts */
	if(ioctl(fd, RTC_PIE_ON, 0) < 0)
	{
			ROS_INFO("ioctl(RTC_PIE_ON)");
			close(fd);
			exit(errno);
	}
	ROS_INFO("ioctl successful!!!");
	return fd;
}


void timer_close(int fd)
{
	/* Disable periodic interrupts */
	ioctl(fd, RTC_PIE_OFF, 0);
	close(fd);
}


int timer_open_nonblock(void)
{
	int fd = open ("/dev/rtc", O_RDONLY|O_NONBLOCK);
	if(fd < 0)
	{
			ROS_INFO("open");
			exit(errno);
	}

	// /*Set the freq as 4Hz*/
	if(ioctl(fd, RTC_IRQP_SET,67) < 0)
	{
			ROS_INFO("ioctl(RTC_IRQP_SET)");
			close(fd);
			exit(errno);
	}
	// /* Enable periodic interrupts */
	if(ioctl(fd, RTC_PIE_ON, 0) < 0)
	{
			ROS_INFO("ioctl(RTC_PIE_ON)");
			close(fd);
			exit(errno);
	}
	ROS_INFO("ioctl successful!!!");
	return fd;
}


void delay_us(int time)
{
	double time_start,time_end,time_dur;
	// ros::Time time_start,time_end;
	time_start=ros::Time::now().toSec();
	time_dur=(double)time/1000000.0;
	do{
		time_end=ros::Time::now().toSec();
		// ROS_INFO("%f -- %f -- %f --%f",time_start,time_end,time_dur);
	}while(fabs(time_start-time_end) < time_dur);
	return;
}

void delay_ms(int time)
{
	delay_us(time*1000);
	return;	
}

void delay_sec(int time)
{
	delay_us(time*1000000);
	return;
}