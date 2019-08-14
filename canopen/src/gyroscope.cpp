#include "ros/ros.h"
#include "canopen/gyroscope.h"
#include <stdio.h>      
#include <stdlib.h>         
#include <unistd.h>         
#include <sys/types.h>     
#include <sys/stat.h>       
#include <fcntl.h>          
#include <termios.h>    
#include <errno.h>         
#include <string.h>
#include <time.h>
#include "canopen/timer.h"
#include "canopen/sensor_cmc.h"
#include <math.h>
#include "canopen/car_driver.h"
#include "canopen/MOTEC_motorCtl.h"
#include "canopen/techMotorCtl.h"
#include "canopen/type.h"
#include "canopen/joy_callback.h"
#include <sys/ioctl.h>

#define OK 1
#define ERROR 0
#define code_spacing 6 //二维码间距 6M
#define angle_xishu 1.0

float h = 0.016; //积分周期
float AGV_len = 2.4;
static unsigned char angle_buff[256] = {0};
static int angle_buff_size = 0;
int gyroscope_fd = -1;  //陀螺仪
int odometer_time_fd = -1;  //里程计定时器
float gyroscope_angle = 0.0;
int ask_speed_flag = 0; //速度查询允许标志 
int begin_angle = 1; //开始读取速度
car_mileage_struct CarMileage_str; //小车里程
car_angle_struct CarAngle_str; //小车角度

LinkQueue flq_speed,frq_speed,rlq_speed,rrq_speed ,center_speed,offset_vx;
wheel_data_struct wheel_data; //车轮数据
float test_err_val; //测试误差值
int odometer_flag = 0; //开启数据缓冲器
float test_err_flag = 0; //测试误差修正值
SqList angle_sq; //角度数据缓冲区
static float last_a = 999.9;
static float delta_a = 0.0;
static float Qcar_angle = 0.0; //陀螺角度全局变量
int walk_data_fp = -1;
int gyr_angle_fp;
float Qcode_distence = 8.1; //码间距
static float test_speed_buff[] = 
{
0.000000, 0.000052, 0.000206, 0.000461, 0.000817, 0.001272, 0.001826, 0.002478, 0.003225, 0.004069, 0.005007, 0.006038, 0.007162,
0.008378, 0.009685, 0.011081, 0.012566, 0.014139, 0.015798, 0.017544, 0.019374, 0.021289, 0.023287, 0.025366, 0.027527, 0.029769,
0.032090, 0.034489, 0.036966, 0.039519, 0.042148, 0.044852, 0.047630, 0.050481, 0.053404, 0.056398, 0.059462, 0.062596, 0.065799,
0.069069, 0.072406, 0.075809, 0.079276, 0.082808, 0.086404, 0.090062, 0.093781, 0.097561, 0.101401, 0.105300, 0.109257, 0.113272,
0.117343, 0.121470, 0.125651, 0.129887, 0.134175, 0.138516, 0.142909, 0.147352, 0.151846, 0.156388, 0.160979, 0.165617, 0.170301,
0.175032, 0.179808, 0.184628, 0.189491, 0.194397, 0.199346, 0.204335, 0.209365, 0.214434, 0.219542, 0.224689, 0.229872, 0.235092,
0.240348, 0.245639, 0.250965, 0.256324, 0.261716, 0.267139, 0.272595, 0.278080, 0.283596, 0.289141, 0.294714, 0.300315, 0.305943,
0.311598, 0.317277, 0.322982, 0.328711, 0.334463, 0.340239, 0.346036, 0.351854, 0.357694, 0.363553, 0.369432, 0.375329, 0.381244,
0.387177, 0.393126, 0.399092, 0.405072, 0.411067, 0.417077, 0.423099, 0.429134, 0.435182, 0.441241, 0.447310, 0.453390, 0.459479,
0.465577, 0.471684, 0.477798, 0.483919, 0.490046, 0.496180, 0.502318, 0.508461, 0.514608, 0.520758, 0.526912, 0.533067, 0.539224,
0.545382, 0.551540, 0.557698, 0.563856, 0.570012, 0.576166, 0.582318, 0.588467, 0.594612, 0.600753, 0.606890, 0.613021, 0.619147,
0.625266, 0.631378, 0.637483, 0.643580, 0.649668, 0.655748, 0.661818, 0.667878, 0.673927, 0.679966, 0.685992, 0.692007, 0.698009,
0.703998, 0.709973, 0.715935, 0.721881, 0.727813, 0.733729, 0.739629, 0.745512, 0.751379, 0.757228, 0.763059, 0.768871, 0.774665,
0.780439, 0.786194, 0.791928, 0.797642, 0.803335, 0.809005, 0.814654, 0.820281, 0.825884, 0.831465, 0.837021, 0.842553, 0.848061,
0.853544, 0.859001, 0.864432, 0.869837, 0.875216, 0.880567, 0.885891, 0.891187, 0.896455, 0.901694, 0.906904, 0.912085, 0.917236,
0.922357, 0.927447, 0.932507, 0.937535, 0.942532, 0.947496, 0.952428, 0.957328, 0.962194, 0.967027, 0.971827, 0.976592, 0.981323,
0.986019, 0.990680, 0.995305, 0.999895, 1.004449, 1.008967, 1.013447, 1.017891, 1.022298, 1.026667, 1.030998, 1.035290, 1.039545,
1.043760, 1.047937, 1.052074, 1.056171, 1.060229, 1.064246, 1.068223, 1.072159, 1.076054, 1.079908, 1.083720, 1.087491, 1.091219,
1.094906, 1.098549, 1.102150, 1.105708, 1.109223, 1.112694, 1.116122, 1.119505, 1.122845, 1.126140, 1.129390, 1.132596, 1.135757,
1.138872, 1.141942, 1.144966, 1.147945, 1.150877, 1.153764, 1.156604, 1.159397, 1.162144, 1.164843, 1.167496, 1.170101, 1.172659,
1.175169, 1.177631, 1.180046, 1.182412, 1.184730, 1.187000, 1.189221, 1.191394, 1.193517, 1.195592, 1.197617, 1.199594, 1.201521,
1.203399, 1.205227, 1.207005, 1.208733, 1.210412, 1.212040, 1.213619, 1.215147, 1.216625, 1.218052, 1.219429, 1.220755, 1.222031,
1.223256, 1.224430, 1.225553, 1.226625, 1.227646, 1.228616, 1.229534, 1.230402, 1.231218, 1.231983, 1.232696, 1.233358, 1.233969,
1.234528, 1.235035, 1.235491, 1.235895, 1.236248, 1.236549, 1.236798, 1.236996, 1.237142, 1.237236, 1.237278, 1.237269, 1.237208,
1.237095, 1.236931, 1.236715, 1.236447, 1.236127, 1.235756, 1.235333, 1.234859, 1.234333, 1.233755, 1.233126, 1.232446, 1.231714,
1.230931, 1.230096, 1.229210, 1.228273, 1.227285, 1.226245, 1.225155, 1.224013, 1.222821, 1.221578, 1.220284, 1.218940, 1.217545,
1.216099, 1.214603, 1.213057, 1.211461, 1.209814, 1.208118, 1.206371, 1.204575, 1.202729, 1.200834, 1.198889, 1.196895, 1.194852,
1.192759, 1.190618, 1.188428, 1.186189, 1.183902, 1.181567, 1.179183, 1.176752, 1.174272, 1.171745, 1.169170, 1.166548, 1.163878,
1.161162, 1.158398, 1.155588, 1.152731, 1.149828, 1.146879, 1.143884, 1.140844, 1.137757, 1.134625, 1.131449, 1.128227, 1.124960,
1.121649, 1.118294, 1.114894, 1.111451, 1.107964, 1.104434, 1.100861, 1.097244, 1.093585, 1.089884, 1.086140, 1.082354, 1.078527,
1.074658, 1.070749, 1.066798, 1.062806, 1.058774, 1.054703, 1.050591, 1.046440, 1.042249, 1.038020, 1.033751, 1.029445, 1.025100,
1.020718, 1.016298, 1.011841, 1.007347, 1.002816, 0.998249, 0.993646, 0.989008, 0.984334, 0.979626, 0.974883, 0.970105, 0.965294,
0.960448, 0.955570, 0.950659, 0.945715, 0.940739, 0.935731, 0.930691, 0.925621, 0.920520, 0.915388, 0.910226, 0.905035, 0.899814,
0.894565, 0.889286, 0.883980, 0.878646, 0.873285, 0.867897, 0.862483, 0.857042, 0.851575, 0.846084, 0.840567, 0.835026, 0.829461,
0.823872, 0.818261, 0.812626, 0.806969, 0.801290, 0.795590, 0.789869, 0.784127, 0.778366, 0.772584, 0.766784, 0.760964, 0.755127,
0.749272, 0.743399, 0.737510, 0.731604, 0.725682, 0.719745, 0.713793, 0.707827, 0.701846, 0.695853, 0.689846, 0.683827, 0.677796,
0.671754, 0.665700, 0.659637, 0.653563, 0.647481, 0.641389, 0.635289, 0.629181, 0.623067, 0.616945, 0.610817, 0.604684, 0.598546,
0.592403, 0.586257, 0.580107, 0.573954, 0.567799, 0.561642, 0.555484, 0.549326, 0.543168, 0.537010, 0.530854, 0.524699, 0.518547,
0.512398, 0.506252, 0.500111, 0.493974, 0.487843, 0.481717, 0.475599, 0.469487, 0.463384, 0.457289, 0.451203, 0.445127, 0.439061,
0.433006, 0.426963, 0.420932, 0.414915, 0.408910, 0.402920, 0.396945, 0.390986, 0.385042, 0.379116, 0.373207, 0.367316, 0.361444,
0.355592, 0.349760, 0.343949, 0.338160, 0.332393, 0.326649, 0.320928, 0.315232, 0.309562, 0.303917, 0.298298, 0.292707, 0.287144,
0.281610, 0.276105, 0.270630, 0.265186, 0.259773, 0.254393, 0.249046, 0.243733, 0.238455, 0.233212, 0.228004, 0.222834, 0.217701,
0.212607, 0.207552, 0.202537, 0.197562, 0.192629, 0.187738, 0.182890, 0.178086, 0.173326, 0.168612, 0.163944, 0.159323, 0.154749,
0.150225, 0.145749, 0.141324, 0.136950, 0.132627, 0.128358, 0.124142, 0.119980, 0.115873, 0.111822, 0.107828, 0.103892, 0.100014,
0.096195, 0.092437, 0.088739, 0.085104, 0.081531, 0.078022, 0.074578, 0.071198, 0.067885, 0.064639, 0.061462, 0.058352, 0.055313,
0.052345, 0.049447, 0.046623, 0.043871, 0.041194, 0.038592, 0.036066, 0.033617, 0.031246, 0.028954, 0.026741, 0.024609, 0.022559,
0.020591, 0.018707, 0.016906, 0.015192, 0.013563, 0.012022, 0.010569, 0.009205, 0.007931, 0.006748, 0.005657, 0.004659, 0.003755,0.002946, 0.002232, 0.001616, 0.001097, 0.000677, 0.000358, 0.000138
};



/* 交换L中数组r的下标为i和j的值 */
void swap(SqList *L,int i,int j) 
{ 
	float temp=L->r[i]; 
	L->r[i]=L->r[j]; 
	L->r[j]=temp; 
}

void print(SqList L)
{
	int i;
	for(i=1;i<L.length;i++)
		printf("as1%f,",L.r[i]);
	printf("as2%f",L.r[i]);
	printf("\n");
}

/* 对顺序表L作直接插入排序 */
void InsertSort(SqList *L)
{ 
	int i,j;
	for(i=2;i<=L->length;i++)
	{
		if (L->r[i]<L->r[i-1]) /* 需将L->r[i]插入有序子表 */
		{
			L->r[0]=L->r[i]; /* 设置哨兵 */
			for(j=i-1;L->r[j]>L->r[0];j--)
				L->r[j+1]=L->r[j]; /* 记录后移 */
			L->r[j+1]=L->r[0]; /* 插入到正确位置 */
		}
	}
}

/* 快速排序优化算法 */
int Partition1(SqList *L,int low,int high)
{ 
	float pivotkey;

	int m = low + (high - low) / 2;  
	if (L->r[low]>L->r[high])			
		swap(L,low,high);
	if (L->r[m]>L->r[high])
		swap(L,high,m);		
	if (L->r[m]>L->r[low])
		swap(L,m,low);		
	
	pivotkey=L->r[low]; /* 用子表的第一个记录作枢轴记录 */
	L->r[0]=pivotkey;  /* 将枢轴关键字备份到L->r[0] */
	while(low<high) /*  从表的两端交替地向中间扫描 */
	{ 
		 while(low<high&&L->r[high]>=pivotkey)
			high--;
		 L->r[low]=L->r[high];
		 while(low<high&&L->r[low]<=pivotkey)
			low++;
		 L->r[high]=L->r[low];
	}
	L->r[low]=L->r[0];
	return low; /* 返回枢轴所在位置 */
}

void QSort1(SqList *L,float low,float high)
{ 
	float pivot;
	if((high-low)>MAX_LENGTH_INSERT_SORT)
	{
		while(low<high)
		{
			pivot=Partition1(L,low,high); /*  将L->r[low..high]一分为二，算出枢轴值pivot */
			QSort1(L,low,pivot-1);		/*  对低子表递归排序 */
			/* QSort(L,pivot+1,high);		 对高子表递归排序 */
			low=pivot+1;	/* 尾递归 */
		}
	}
	else
		InsertSort(L);
}

/* 对顺序表L作快速排序 */
void QuickSort(SqList *L)
{ 
	QSort1(L,1,L->length);
}


/****************************************************
name		:T_integral
function	:复化T型积分
input		:none
output		:none
*****************************************************/
float T_integral(float* buff, int size){
     float T = 0; 
	 int k = 0;
     
	for (k = 1; k < size; k++) {
      T += buff[k];
     }
     T= (h / 2) * (buff[0]+ 2*T + buff[k]);
     return T;
}

/****************************************************
name		:test_integral
function	:积分测试函数
input		:none
output		:none
****************************************************/
int test_integral(void) 
{
     //double result = 0;
     float result = 0;
     
	//复化T型积分
     result = T_integral(test_speed_buff,sizeof(test_speed_buff)/sizeof(test_speed_buff[0]));
     printf("复化梯形求解答案 : %lf\n", result);
     return 0;
}

int visit(float c)
{
	printf("as%f ",c);
	return OK;
}

/* 构造一个空队列Q */
int InitQueue(LinkQueue *Q)
{ 
	Q->front=Q->rear=(QueuePtr)malloc(sizeof(QNode));
	if(!Q->front)
		exit(EOVERFLOW);
	Q->front->next=NULL;
	return OK;
}

/* 销毁队列Q */
int DestroyQueue(LinkQueue *Q)
{
	while(Q->front)
	{
		 Q->rear=Q->front->next;
		 free(Q->front);
		 Q->front=Q->rear;
	}
	return OK;
}

/* 将Q清为空队列 */
int ClearQueue(LinkQueue *Q)
{
	QueuePtr p,q;
	Q->rear=Q->front;
	p=Q->front->next;
	Q->front->next=NULL;
	while(p)
	{
		 q=p;
		 p=p->next;
		 free(q);
	}
	return OK;
}

/* 若Q为空队列,则返回OK,否则返回ERROR */
int QueueEmpty(LinkQueue Q)
{ 
	if(Q.front==Q.rear)
		return OK;
	else
		return ERROR;
}

/* 求队列的长度 */
int QueueLength(LinkQueue Q)
{ 
	int i=0;
	QueuePtr p;
	p=Q.front;
	while(Q.rear!=p)
	{
		 i++;
		 p=p->next;
	}
	return i;
}

/* 若队列不空,则用e返回Q的队头元素,并返回OK,否则返回ERROR */
int GetHead(LinkQueue Q,float *e)
{ 
	QueuePtr p;
	if(Q.front==Q.rear)
		return ERROR;
	p=Q.front->next;
	*e=p->data;
	return OK;
}


/* 插入元素e为Q的新的队尾元素 */
int EnQueue(LinkQueue *Q,float e)
{ 
	QueuePtr s=(QueuePtr)malloc(sizeof(QNode));
	if(!s) /* 存储分配失败 */
		exit(EOVERFLOW);
	s->data=e;
	s->next=NULL;
	Q->rear->next=s;	/* 把拥有元素e的新结点s赋值给原队尾结点的后继*/
	Q->rear=s;		/* 把当前的s设置为队尾结点，rear指向s*/
	return OK;
}

/* 若队列不空,删除Q的队头元素,用e返回其值,并返回OK,否则返回ERROR */
int DeQueue(LinkQueue *Q,float *e)
{
	QueuePtr p;
	if(Q->front==Q->rear)
		return ERROR;
	p=Q->front->next;		/* 将欲删除的队头结点暂存给p*/
	*e=p->data;				/* 将欲删除的队头结点的值赋值给e */
	Q->front->next=p->next;/* 将原队头结点的后继p->next赋值给头结点后继*/
	if(Q->rear==p)		/* 若队头就是队尾，则删除后将rear指向头结点*/
		Q->rear=Q->front;
	free(p);
	return OK;
}

/* 从队头到队尾依次对队列Q中每个元素输出 */
int QueueTraverse(LinkQueue Q)
{
	QueuePtr p;
	p=Q.front->next;
	while(p)
	{
		 visit(p->data);
		 p=p->next;
	}
	printf("\n");
	return OK;
}


/****************************************************
name		:test_Lqueue
function	:链式队列测试函数
input		:none
output		:none
****************************************************/
int test_Lqueue(void)
{
	int i;
	float d;
	LinkQueue q;
	i=InitQueue(&q);
	if(i)
		printf("成功地构造了一个空队列!\n");
	printf("是否空队列？%d(1:空 0:否)",QueueEmpty(q));
	printf("队列的长度为%d\n",QueueLength(q));
	EnQueue(&q,-5);
	EnQueue(&q,5);
	EnQueue(&q,10);
	printf("插入3个元素(-5,5,10)后,队列的长度为%d\n",QueueLength(q));
	printf("是否空队列？%d(1:空 0:否)  ",QueueEmpty(q));
	printf("队列的元素依次为：");
	QueueTraverse(q);
	i=GetHead(q,&d);
	if(i==OK)
	 printf("队头元素是：%f\n",d);
	DeQueue(&q,&d);
	printf("删除了队头元素%f\n",d);
	i=GetHead(q,&d);
	if(i==OK)
		printf("新的队头元素是：%f\n",d);
	ClearQueue(&q);
	//printf("清空队列后,q.front=%u q.rear=%u q.front->next=%u\n",q.front,q.rear,q.front->next);
	printf("清空队列后,q.front=%p q.rear=%p q.front->next=%p\n",q.front,q.rear,q.front->next);
	DestroyQueue(&q);
	//printf("销毁队列后,q.front=%u q.rear=%u\n",q.front, q.rear);
	printf("销毁队列后,q.front=%p q.rear=%p\n",q.front, q.rear);	
	printf("队列的长度为%d\n",QueueLength(q));
	return 0;
}


/*******************************************************************  
name：        :UART0_Open  
function      :打开串口并返回串口设备文件描述  
ino           :port    串口号(ttyS0,ttyS1,ttyS2)  
*出口参数：正确返回为1，错误返回为0  
*******************************************************************/    
int UART0_Open(int fd,const char* port)
{    
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);    
    if (fd<0)    
    {    
        perror("Can't Open Serial Port");    
        return(FALSE);    
    }    
    //恢复串口为阻塞状态                                   
    if(fcntl(fd, F_SETFL, 0) < 0)    
    {    
        printf("fcntl failed!\n");    
        return(FALSE);    
    }         
    else    
    {    
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));    
    }    
    //测试是否为终端设备        
    if(0 == isatty(STDIN_FILENO))    
    {    
        printf("standard input is not a terminal device\n");    
        return(FALSE);    
    }    
    else    
    {    
        printf("isatty success!\n");    
    }                  
    printf("fd->open=%d\n",fd);    
    return fd;    
}    
/*******************************************************************  
name：           UART0_Close  
function：       关闭串口并返回串口设备文件描述  
input：  fd      文件描述符   
         port    串口号(ttyS0,ttyS1,ttyS2)  
output：void  
*******************************************************************/    
     
void UART0_Close(int fd)    
{    
    close(fd);    
}    
     
/*******************************************************************  
name		:UART0_Set  
function	:设置串口数据位，停止位和效验位  
input:
	speed       串口速度  
	flow_ctrl   数据流控制  
	databits    数据位   取值为 7 或者8  
	stopbits    停止位   取值为 1 或者2  
	parity      效验类型 取值为N,E,O,,S  
output	:正确返回为1，错误返回为0  
*******************************************************************/    
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)    
{    
       
    int   i;    
    int   status;    
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};    
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};    
             
    struct termios options;    
       
    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */    
    if( tcgetattr( fd,&options)  !=  0)    
    {    
        perror("SetupSerial 1");        
        return(FALSE);     
    }    
      
    //设置串口输入波特率和输出波特率    
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)    
    {    
        if  (speed == name_arr[i])    
        {                 
            cfsetispeed(&options, speed_arr[i]);     
            cfsetospeed(&options, speed_arr[i]);      
        }    
    }         
       
    //修改控制模式，保证程序不会占用串口    
    options.c_cflag |= CLOCAL;    
    //修改控制模式，使得能够从串口中读取输入数据    
    options.c_cflag |= CREAD;    
      
    //设置数据流控制    
    switch(flow_ctrl)    
    {    
          
        case 0 ://不使用流控制    
              options.c_cflag &= ~CRTSCTS;    
              break;       
          
        case 1 ://使用硬件流控制    
              options.c_cflag |= CRTSCTS;    
              break;    
        case 2 ://使用软件流控制    
              options.c_cflag |= IXON | IXOFF | IXANY;    
              break;    
    }    
    //设置数据位    
    //屏蔽其他标志位    
    options.c_cflag &= ~CSIZE;    
    switch (databits)    
    {      
        case 5    :    
                     options.c_cflag |= CS5;    
                     break;    
        case 6    :    
                     options.c_cflag |= CS6;    
                     break;    
        case 7    :        
                 options.c_cflag |= CS7;    
                 break;    
        case 8:        
                 options.c_cflag |= CS8;    
                 break;      
        default:       
                 fprintf(stderr,"Unsupported data size\n");    
                 return (FALSE);     
    }    
    //设置校验位    
    switch (parity)    
    {      
        case 'n':    
        case 'N': //无奇偶校验位。    
                 options.c_cflag &= ~PARENB;     
                 options.c_iflag &= ~INPCK;        
                 break;     
        case 'o':      
        case 'O'://设置为奇校验        
                 options.c_cflag |= (PARODD | PARENB);     
                 options.c_iflag |= INPCK;                 
                 break;     
        case 'e':     
        case 'E'://设置为偶校验      
                 options.c_cflag |= PARENB;           
                 options.c_cflag &= ~PARODD;           
                 options.c_iflag |= INPCK;          
                 break;    
        case 's':    
        case 'S': //设置为空格     
                 options.c_cflag &= ~PARENB;    
                 options.c_cflag &= ~CSTOPB;    
                 break;     
        default:      
                 fprintf(stderr,"Unsupported parity\n");        
                 return (FALSE);     
    }     
    // 设置停止位     
    switch (stopbits)    
    {      
        case 1:       
                 options.c_cflag &= ~CSTOPB; break;     
        case 2:       
                 options.c_cflag |= CSTOPB; break;    
        default:       
                       fprintf(stderr,"Unsupported stop bits\n");     
                       return (FALSE);    
    }    
       
    //修改输出模式，原始数据输出    
    options.c_oflag &= ~OPOST;    
      
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    //options.c_lflag &= ~(ISIG | ICANON);    
       
    //设置等待时间和最小接收字符    
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */      
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */    
       
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读    
    tcflush(fd,TCIFLUSH);    
       
    //激活配置 (将修改后的termios数据设置到串口中）    
    if (tcsetattr(fd,TCSANOW,&options) != 0)      
    {    
        perror("com set error!\n");      
        return (FALSE);     
    }    
    return (TRUE);     
}    
/*******************************************************************  
name	:UART0_Init()  
function:串口初始化  
input	:
	speed      串口速度  
	flow_ctrl  数据流控制  
	databits   数据位   取值为 7 或者8  
	stopbits   停止位   取值为 1 或者2  
	parity     效验类型 取值为N,E,O,,S  
output:出口参数：正确返回为1，错误返回为0  
*******************************************************************/    
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)    
{    
    int err;    
    //设置串口数据帧格式    
    if (UART0_Set(fd,speed,0,8,1,'N') == FALSE)    
    {                                                             
        return FALSE;    
    }    
    else    
    {    
        return  TRUE;    
    }    
}    
     
/*******************************************************************  
* 名称：            UART0_Recv  
* 功能：            接收串口数据  
* 入口参数：        fd         文件描述符      
*                   rcv_buf    接收串口中数据存入rcv_buf缓冲区中  
*                   data_len   一帧数据的长度  
* 出口参数：        正确返回为1，错误返回为0  
*******************************************************************/    
int UART0_Recv(int fd, char *rcv_buf,int data_len)    
{    
    int len,fs_sel;    
    fd_set fs_read;    
       
    struct timeval time;    
       
    FD_ZERO(&fs_read);    
    FD_SET(fd,&fs_read);    
       
    time.tv_sec = 2;    
    time.tv_usec =0;    
       
    //使用select实现串口的多路通信    
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);    
    //printf("fs_sel = %d\n",fs_sel);    
    if(fs_sel)    
    {    
        len = read(fd,rcv_buf,data_len);    
        return len;    
    }    
    else    
    {    
        return FALSE;    
    }         
}    
/********************************************************************  
name	: UART0_Send  
function: 发送数据 
input:
	 fd           文件描述符      
 	send_buf     存放串口发送数据 
 	data_len     一帧数据的个数  
output:	 正确返回为1，错误返回为0  
*******************************************************************/    
int UART0_Send(int fd, char *send_buf,int data_len)    
{    
    int len = 0;    
       
    len = write(fd,send_buf,data_len);    
    if (len == data_len )    
    {    
        //printf("send data is %s\n",send_buf);  
        return len;    
    }         
    else       
    {    
                   
        tcflush(fd,TCOFLUSH);    
        return FALSE;    
    }    
       
}    


/***********************************************************
name	:gyroscope_init
function:实现对陀螺仪的初始化
intpu	:none
output	:none
************************************************************/
void gyroscope_init(void)
{
    int err;               //返回调用函数的状态    
	gyroscope_fd = UART0_Open(gyroscope_fd,"/dev/angle_serial"); //打开串口，返回文件描述符   
     do  
		{    
			err = UART0_Init(gyroscope_fd,115200,0,8,1,'N');    
			printf("Set Port Exactly!\n"); 
			sleep(1);   
		}while(FALSE == err || FALSE == gyroscope_fd);    
}

/*******************************************************
name	:gyroscope_read_write
function:实现对陀螺仪数据读取和设置函数
mod_cmd: 模式选择：0为发送数据读取命令，1为读取数据
****************************************************/ 
int gyroscope_read_write(char mode_cmd, char*buff)    
{
    int err;               //返回调用函数的状态    
    int len;                            
    int i;    
    char rcv_flag = 0;
	char rcv_buf[256];             
    char send_buf[] = {0x68,0x04,0x00,0x04,0x08};
     
    if(0 == mode_cmd)    
    {   
		len = UART0_Send(gyroscope_fd,send_buf, sizeof(send_buf));    
		if(len > 0)    
			;//printf(" %d time send %d data successful\n",i,len);    
		else    
			printf("send data failed!\n");                          
		return 0;                
    }    
    else                                       
    {                                          
        while (1) //循环读取数据    
		{   
			memset(rcv_buf,0x00,sizeof(rcv_buf));
            len = UART0_Recv(gyroscope_fd, rcv_buf,sizeof(rcv_buf));    
            if(len > 0)    
            {   
			//	printf("len = %d\n",len); 
                if((rcv_buf[0] == 0x68)&&((rcv_buf[1] == 0x0a)||(rcv_buf[1])==0x0d))
					{  
				//		angle_buff_size = 0;
						memset(angle_buff,0x00,sizeof(angle_buff));
						memcpy(angle_buff,rcv_buf,len);
						angle_buff_size = len;
						rcv_flag = 1;
			//			printf("=====rcv_buf[1] = 0x0d??%X\n",rcv_buf[1]);
					}
				else
					{
						memcpy(angle_buff + angle_buff_size,rcv_buf,len);
						angle_buff_size += len;
					//	printf("buff size = %d\n",angle_buff_size);
					//	printf("gyroscpe_rcv_len = %d\n",len);
						rcv_flag = 2;
					}
				if(angle_buff_size >= 14)
					{
						if(angle_buff[0]== 0x68)// && (angle_buff[1] == 0x0d))
							{
								angle_buff_size = 0;
								#if 0
								printf("recive data:\n");
								for(int i = 0; i < angle_buff_size; i++)
									{
										printf("%X ",angle_buff[i]);
									}
								printf("\n");
								#endif
								return 0;
							}	
						else	
							{
								memset(angle_buff,0x00,sizeof(angle_buff));
								angle_buff_size = 0;
								return -1;
							}
					}
				else
					{
						if(rcv_flag == 2)
							return -1;
					}
            }    
            else    
            {    
                printf("cannot receive data\n");    
            }    
        }                
        UART0_Close(gyroscope_fd);     
    }    
}    

/**********************************************************
name	:gyroscope_analyze
function:实现陀螺仪的输出数据解析
input	:buff,size
output	:输出陀螺仪角度
***********************************************************/
float gyroscope_analyze(unsigned char* buff,int buff_size)
{
	float angle = 0.0;

	angle = (buff[10]&0x0f)*100;
	angle += (buff[11]>>4)*10;
	angle += buff[11]&0x0f;
	angle += (buff[12]>>4)*0.1;
	angle += (buff[12]&0x0f)*0.01;
	
	if((buff[10]>>4)==1)
		angle *= (-1);
	return angle;
}
/*
int main(char* argc, char** agrv)
{
	float gyroscope_angle = 0.0;

	gyroscope_init();
	while(1)
		{	
			usleep(200000);
			gyroscope_read_write(0,NULL);
			gyroscope_read_write(1,NULL);
			gyroscope_angle = gyroscope_analyze(angle_buff,14);
			printf("陀螺仪角度:%f\n",gyroscope_angle);
        	//UART0_Close(gyroscope_fd);     
		}
	return 0;
}
*/

/*************************************************
name	:get_gyroscope_angle
functiom:获取陀螺仪角度
input	:none
output	:角度
**************************************************/
float get_gyroscope_angle(void)
{
	float angle = 0.0;
	int res = 0;

	do{
		gyroscope_read_write(0,NULL);
		res = gyroscope_read_write(1,NULL);
		if(res >= 0)
			angle = gyroscope_analyze(angle_buff,14);
		}while(res < 0);
	return angle;
}

/*度到弧度*/
float du_to_rad(float du)
{
	float res = 3.14159/180*du;
	return res;
}
/*获取中心点角度*/
float get_center_angle(float a1,float a2,float a3,float a4)
{
	float center_angle = 0.0;
	float theta_f = (a1 + a2)/2;
	float theta_b = (a3 + a4)/2;
	float h = 0.0;
	if(fabs(theta_f) == fabs(theta_b))
		{
			center_angle = (theta_f + theta_b)/2;
		}
	else
		{
			h = AGV_len/fabs(tan(du_to_rad(theta_f)) - tan(du_to_rad(theta_b)));
			if(fabs(theta_f)<fabs(theta_b))
			{
				h = fabs(h)*theta_b/fabs(theta_b);
				h *= (-1);
			float l1 = fabs(h*sin(du_to_rad(theta_f))/cos(du_to_rad(theta_f)));
			float l2 = fabs(h*sin(du_to_rad(theta_b))/cos(du_to_rad(theta_b)));
			center_angle = atan((-l2 + AGV_len/2)/h);
			}
			else
			{
				
				h = fabs(h)*theta_f/fabs(theta_f);
			float l1 = fabs(h*sin(du_to_rad(theta_f))/cos(du_to_rad(theta_f)));
			float l2 = fabs(h*sin(du_to_rad(theta_b))/cos(du_to_rad(theta_b)));
			center_angle = atan((l1 - AGV_len/2)/h);
			}
			center_angle = center_angle/(3.14159/180);
		}
	//printf("center_angle :%f\n",center_angle);
	return center_angle;	
}


/*获取中心点速度*/
float get_center_speed(float v1,float v2, float v3, float v4, float a1, float a2, float a3, float a4)
{
	float theta_f = (a1 + a2)/2;
	float theta_b = (a3 + a4)/2;
	float vf = (v1 + v2)/2;
	float vb = (v3 + v4)/2;
	float v_center = 0.0;
	float rf,rb,w = 0.0;

	float h = AGV_len/(tan(du_to_rad(theta_f)) + tan(du_to_rad(theta_b)));
	float l1 = fabs(h*sin(du_to_rad(theta_f))/cos(du_to_rad(theta_f)));
	float l2 = fabs(h*sin(du_to_rad(theta_b))/cos(du_to_rad(theta_b)));
	//printf("speed: %f,%f,%f,%f\n",v1,v2,v3,v4);
	//printf("角度: %f,%f,%f,%f\n",a1,a2,a3,a4);
	if(fabs(theta_f) == fabs(theta_b))
		{ 
			v_center = (vf + vb)/2;
		}
	else
		{
			rf = fabs(h/cos(du_to_rad(theta_f)));
			rb = fabs(h/cos(du_to_rad(theta_b)));
			w = (vf/rf + vb/rb)/2;
			v_center = w*sqrt((l1 - AGV_len/2)*(l1 - AGV_len/2) + h*h);
		}
	//printf("Center speed :%f\n",v_center);	
	return v_center;
}

void* gyroscope_pending(void *)
{
	char tmp[]="motorCurrent[0]\n";
	while(1)
		{	
			//usleep(75000);
			usleep(260000);
			gyroscope_angle = get_gyroscope_angle();
			//printf("gyroscope_angle++++++++++++++++++++++++++++++++++: %f\n",gyroscope_angle);
#if 0	
							if(gyr_angle_fp=open("./gyr_angle_data.txt",O_RDWR|O_CREAT|O_APPEND)<0)
								{
									ROS_INFO("ERROR:Can not open walk data.txt!");
								}
								int num=sprintf(tmp,"%6f\n",gyroscope_angle);
				//				ROS_INFO("%s",tmp);
								write(gyr_angle_fp,tmp,num);
								close(gyr_angle_fp);
#endif
			#if 0
			angle_sq.r[angle_sq.length++] = get_gyroscope_angle();;
			if(angle_sq.length >= 6)
				{
					angle_sq.length -= 1;
				//	print(angle_sq);
					QuickSort(&angle_sq);
				//	print(angle_sq);
					gyroscope_angle = angle_sq.r[3];
					//printf("gyroscope_angle++++++++++++++++++++++++++++++++++: %f\n",gyroscope_angle);
					memset(&angle_sq,0.0,sizeof(angle_sq));
				}
			#endif
		}
}

/**********************************************************
name		:send_ask_speed
function	:定时询问驱动电机速度
**********************************************************/
void* send_ask_speed(void* A)
{
	unsigned long data;
	float tmp_center_speed = 0.0;
	float tmp_center_angle = 0.0;
	float tmp_offset_vx = 0.0;
	char tmp[]="motorCurrent[0]\tmotorCurrent[1]\tmotorCurrent[2]\tmotorCurrent[3]\tmotorCurrent[4]\tmotorCurrent[5]\tmotorCurrent[6]\tmotorCurrent[7]\tmotorCurrent[8]\tmotorCurrent[9]\tmotorCurrent[10]\tmotorCurrent[11]\n";
	while(1)
		{
			if(ask_speed_flag)
				{
				//	printf("进入查询速度函数：\n");
					AGV_SPEED_Enquire(1);
					AGV_SPEED_Enquire(2);
					AGV_SPEED_Enquire(3);
					AGV_SPEED_Enquire(4);
					TK_position_Enquire();
					if(read(odometer_time_fd, &data, sizeof(long unsigned int)) <= 0)
					//if(read(odometer_time_fd,&data,sizeof(long unsigned int)) <= 0)
						{
							ROS_INFO("read_timer error\n");
						}
		//			ROS_INFO("wheel_speed:%f,%f,%f,%f",Get_Speed1,Get_Speed2,Get_Speed3,Get_Speed4);
					//ROS_INFO("wheel_angle:%f,%f,%f,%f",wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4);
					if((flq_speed.front != NULL)||(flq_speed.rear != NULL))
						EnQueue(&flq_speed,Get_Speed1);
					if((frq_speed.front != NULL)||(frq_speed.rear != NULL))
						EnQueue(&frq_speed,Get_Speed2);
					if((rlq_speed.front != NULL)||(rlq_speed.rear != NULL))
						EnQueue(&rlq_speed,Get_Speed3);
					if((rrq_speed.front != NULL)||(rrq_speed.rear != NULL))
						EnQueue(&rrq_speed,Get_Speed4);
					if((center_speed.front != NULL)||(center_speed.rear != NULL))
						{
							tmp_center_speed = get_center_speed(fabs(Get_Speed1),fabs(Get_Speed2),fabs(Get_Speed3),fabs(Get_Speed4),wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4);
				//			ROS_INFO("Center_speed:%f",tmp_center_speed);
							EnQueue(&center_speed,tmp_center_speed);
						}
					if((offset_vx.front != NULL)||(offset_vx.rear != NULL))
						{
							tmp_center_angle = get_center_angle(wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4);
				//			ROS_INFO("off_center_angle:%f",tmp_center_angle);
							tmp_offset_vx = sin(du_to_rad(tmp_center_angle))*tmp_center_speed;
				//			ROS_INFO("wheel a: %f,%f,%f,%f",wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4);
				//			ROS_INFO("offset_x_speed:%f",tmp_offset_vx);
							EnQueue(&offset_vx,tmp_offset_vx);

								if((walk_data_fp=open("./walk_data.txt",O_RDWR|O_CREAT|O_APPEND))<0)
								{
									ROS_INFO("ERROR:Can not open walk data.txt!");
								}
								int num=sprintf(tmp,"%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\n",tmp_center_speed,tmp_center_angle,Qcar_angle,wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4,fabs(Get_Speed1),fabs(Get_Speed2),fabs(Get_Speed3),fabs(Get_Speed4));
				//				ROS_INFO("%s",tmp);
								write(walk_data_fp,tmp,num);
								close(walk_data_fp);
						}
				}
		}
}
/*************************************************************
name		:odometer_start
function	:开启里程计
input		:none
output		:none
**************************************************************/
int odometer_start(void)
{
	int res = -1;
	odometer_time_fd = timer_open();
	if(odometer_time_fd < 0)
		{
			ROS_INFO("open timer err");
		}
	else
		{
			ROS_INFO("open timer ok!! fd = %d",odometer_time_fd);
		}
	res = InitQueue(&flq_speed);
	res = InitQueue(&frq_speed);
	res = InitQueue(&rlq_speed);
	res = InitQueue(&rrq_speed);
	res = InitQueue(&center_speed);
	res = InitQueue(&offset_vx);
	Get_Speed1 = 0;
	Get_Speed2 = 0;
	Get_Speed3 = 0;
	Get_Speed4 = 0;
	memset(&CarMileage_str,0x00,sizeof(car_mileage_struct));
	ask_speed_flag = 1;
	begin_angle = 1;
	test_err_val = gyroscope_angle;
	printf("angle_err_val:%f\n",test_err_val);
	sleep(0.5);
	test_err_val += gyroscope_angle;
	printf("angle_err_val:%f\n",test_err_val);
	test_err_val = test_err_val/2; 
	printf("angle_err_val:%f\n",test_err_val);
	test_err_flag = 1;
	odometer_flag = 1;
	return res;
}


/*************************************************************
name		:odometer_stop
function	:关闭里程计
input		:none
output		:none
**************************************************************/
void odometer_stop(void)
{	
	timer_close(odometer_time_fd);
	DestroyQueue(&flq_speed);
	DestroyQueue(&frq_speed);
	DestroyQueue(&rlq_speed);
	DestroyQueue(&rrq_speed);
	DestroyQueue(&center_speed);
	DestroyQueue(&offset_vx);
	ask_speed_flag = 0;
	test_err_flag = 0;
	odometer_flag = 0;
}


/************************************************************
name		:get_car_mileage
function	:获取小车里程
input		:cmd_mode :=1时：最后一次，清空队列 
output		:当前里程,出错返回-1
*************************************************************/
float get_car_mileage(int cmd_mode)
{
	int FlqSpeed_len = 0;
	int FrqSpeed_len = 0;
	int RlqSpeed_len = 0;
	int RrqSpeed_len = 0;
	int run_flag = 0;
	int offset_vx_len = 0;
	int CenterSpeed_len = 0;

	int i = 0;
	float temp_mileage = 0.0;
	int res = 0;
	float err_max = 0.1 ;//误差最大值
	float tmp_s1 = 0.0;
	float tmp_s2 = 0.0;
	float tmp_s3 = 0.0;
	float tmp_s4 = 0.0;
	float tmp_s0 = 0.0;
	float center_s = 0.0;
	float offset_sx = 0.0;

	FlqSpeed_len = QueueLength(flq_speed);
	FrqSpeed_len = QueueLength(frq_speed);
	RlqSpeed_len = QueueLength(rlq_speed);
	RrqSpeed_len = QueueLength(rrq_speed);
	CenterSpeed_len = QueueLength(center_speed);
	offset_vx_len = QueueLength(offset_vx);

	//printf("Qspeed长度为:%d\n",CenterSpeed_len);
	float speed_buff1[FlqSpeed_len + 2] = {0};
	float speed_buff2[FrqSpeed_len + 2] = {0};
	float speed_buff3[RlqSpeed_len + 2] = {0};
	float speed_buff4[RrqSpeed_len + 2] = {0};
	float speed_buff_center[CenterSpeed_len +2] = {0}; 
	float offset_vx_buff[offset_vx_len +2] = {0}; 
		
	if((((FlqSpeed_len + FrqSpeed_len + RlqSpeed_len + RrqSpeed_len) > 100)||(cmd_mode == 1))&&(run_flag)) 
		{
			printf("Qspeed长度为:%d,%d,%d,%d\n",FlqSpeed_len,FrqSpeed_len,RlqSpeed_len,RrqSpeed_len);
			if(cmd_mode == 1)
				{
					printf("Qspeed长度为:%d,%d,%d,%d\n",FlqSpeed_len,FrqSpeed_len,RlqSpeed_len,RrqSpeed_len);
				}
			for(i = 0; i < FlqSpeed_len; i++)
				{
					DeQueue(&flq_speed,&speed_buff1[i]);
				}
			tmp_s1 = T_integral(speed_buff1,FlqSpeed_len);	
			CarMileage_str.FlWheel_mileage += tmp_s1;	
			for(i = 0; i < FrqSpeed_len; i++)
				{
					DeQueue(&frq_speed,&speed_buff2[i]);
				}
			tmp_s2 = T_integral(speed_buff2,FrqSpeed_len);	
			CarMileage_str.FrWheel_mileage += tmp_s2;	
			for(i = 0; i < RlqSpeed_len; i++)
				{
					DeQueue(&rlq_speed,&speed_buff3[i]);
				}
			tmp_s3 = T_integral(speed_buff3,RlqSpeed_len);	
			CarMileage_str.RlWheel_mileage += tmp_s3;	
			for(i = 0; i < RrqSpeed_len; i++)
				{
					DeQueue(&rrq_speed,&speed_buff4[i]);
				}
			tmp_s4 = T_integral(speed_buff4,RrqSpeed_len);	
			CarMileage_str.RrWheel_mileage += tmp_s4;	
		
			/*计算均值，单个轮子偏离均值太大说明轮子打滑或有轮子没转*/	
			temp_mileage = (fabs(CarMileage_str.FlWheel_mileage) + fabs(CarMileage_str.FrWheel_mileage));
			temp_mileage += fabs(CarMileage_str.RlWheel_mileage) + fabs(CarMileage_str.RrWheel_mileage);
			CarMileage_str.car_mileage = (temp_mileage/4);
			tmp_s0 = (fabs(tmp_s1) + fabs(tmp_s2) + fabs(tmp_s3) + fabs(tmp_s4))/4;
			CarMileage_str.car_tmp_mileage = tmp_s0;
			//ROS_INFO("car_mileage  before :%f",CarMileage_str.car_mileage);
			if(fabsf(CarMileage_str.car_mileage - fabs(CarMileage_str.FlWheel_mileage)) > err_max)
				{
					res = -1;
				}
			if(fabsf(CarMileage_str.car_mileage - fabs(CarMileage_str.FrWheel_mileage)) > err_max)
				{
					res = -1;
				}
			if(fabsf(CarMileage_str.car_mileage - fabs(CarMileage_str.RlWheel_mileage)) > err_max)
				{
					res = -1;
				}
			if(fabsf(CarMileage_str.car_mileage - fabs(CarMileage_str.RrWheel_mileage)) > err_max)
				{
					res = -1;
				}
			if(res == -1)
				{
					for(i = 0; i < 20; i++)
						printf("有轮子打滑或没转\r");
					printf("平均里程:%f\n",temp_mileage/4);
					printf("单轮里程:FL:%f, FR:%f, RL:%f, RR:%f\n",CarMileage_str.FlWheel_mileage,
							CarMileage_str.FrWheel_mileage,CarMileage_str.RlWheel_mileage,CarMileage_str.RrWheel_mileage);
					return -1;
				}	
			else
				{
					return CarMileage_str.car_mileage;
				}
		}
	if((offset_vx_len >=25)||(cmd_mode == 1))
	{
		//printf("Qspeed offset_vx 长度为:%d\n",offset_vx_len);
		if(cmd_mode == 1)
			{
				printf("Qspeed offset_vx 长度为:%d\n",offset_vx_len);
			}
		for(i = 0; i < offset_vx_len; i++)
			{
				DeQueue(&offset_vx,&offset_vx_buff[i]);
			}
		offset_sx = T_integral(offset_vx_buff,CenterSpeed_len);	
		CarMileage_str.car_offset_x += offset_sx;	
		CarMileage_str.car_tmp_offset_x = offset_sx;
		printf("本次小车X偏移量:%f\n",CarMileage_str.car_tmp_offset_x);
		printf("累计小车X偏移量:%f\n",CarMileage_str.car_offset_x);
	}
	if((CenterSpeed_len >=25)||(cmd_mode == 1))
	{
		//printf("Qspeed长度为:%d\n",CenterSpeed_len);
		if(cmd_mode == 1)
			{
				printf("Qspeed长度为:%d\n",CenterSpeed_len);
			}
		for(i = 0; i < CenterSpeed_len; i++)
			{
				DeQueue(&center_speed,&speed_buff_center[i]);
			}
		center_s = T_integral(speed_buff_center,CenterSpeed_len);	
		CarMileage_str.car_mileage += center_s;	
		CarMileage_str.car_tmp_mileage = center_s;
		printf("中心移动里程:%f\n",CarMileage_str.car_mileage);
		return CarMileage_str.car_mileage;	
	}
//	return CarMileage_str.car_mileage;
}

/**********************************************************
name		:angle_to_0
function	:实现陀螺仪角度归０化处理
input		:angle 输入的角度
output		:输出归0后的角度, 若出错，输出1000
***********************************************************/
float GyAngle_to_0(float angle)
{
	float angle_0 = CarAngle_str.theta2_GY;
	if((fabs(0 - angle) < 30))
		{
			return angle;
		}
	if((fabs(90 - angle) < 30))
		{
			return angle -90;
		}
	if((fabs(180 - angle) < 30))
		{
			return angle - 180;
		}
	if((fabs(270 - angle) < 30))
		{
			return angle - 270;
		}
	if((fabs(360 - angle) < 30))
		{
			return angle - 360;
		}
	return angle_0;
}

/**********************************************************
name		:angle_to_0
function	:实现倍加福角度归０化处理
input		:angle 输入的角度
output		:输出归0后的角度,出错输出1000
***********************************************************/
float BjfAngle_to_0(float angle)
{
	int car_dir = 0;
		
    if(angle>-30 && angle<30)
    {
        car_dir = 0;
		return angle;
    }
    else if(angle>60 && angle<120)
    {
        car_dir = 90;
		return angle - 90;
    }
    else if(angle > -120 && angle <-60)
    {
        car_dir =-90;
		return angle + 90;
    }
    else if(angle <-150 || angle >150)
    {
        car_dir = 180;
		if(angle < 0)
			{
				return 180 + angle;
			}
		else
			{
				return angle -180;
			}
    }
	return 1000.0;
}

/***********************************************************
name		:get_car_angle
function	:获取小车偏移角度
input		:none
output		:小车与二维码之间的偏移角
***********************************************************/
float get_car_angle(void)
{
	float temp_angle1 = 0.0;
	float temp_angle2 = 0.0;
	float center_speed_angle = 0.0;
	float now_a;

	CarAngle_str.GyrNow_angle = GyAngle_to_0(gyroscope_angle);
	printf("Now_angle :%f\n",CarAngle_str.GyrNow_angle);
	#if 0
	printf("原始角度:%f\n",gyroscope_angle);
	printf("归0角度:%f\n",CarAngle_str.GyrNow_angle);
	printf("last angle :%f\n",last_a);
	printf("delta :%f\n",delta_a);
	printf("有效角度theta2:%f\n",CarAngle_str.theta2_GY);
	if(delta_a == 0.0){delta_a = 0.01;}
	if((fabs(CarAngle_str.GyrNow_angle - last_a) > fabs(delta_a)*4)&&(begin_angle == 0)) /*变化率太大*/
		{
			printf("++++++++++++++++++变化率太大\n");	
			CarAngle_str.theta2_GY = last_a + delta_a;
			last_a = CarAngle_str.theta2_GY;
		}
	else
		{
			begin_angle = 0;
			delta_a = CarAngle_str.GyrNow_angle - last_a;
			last_a = CarAngle_str.GyrNow_angle;
			CarAngle_str.theta2_GY = CarAngle_str.GyrNow_angle;//读取陀螺偏移角度
		}
	#else
		if(fabs(CarAngle_str.GyrNow_angle) > 20)
			{
				CarAngle_str.theta2_GY = last_a;
			}
		else
			{
				CarAngle_str.theta2_GY = CarAngle_str.GyrNow_angle;//读取陀螺偏移角度
				last_a = CarAngle_str.GyrNow_angle;
			}
	#endif
	if(CarAngle_str.now_CodeNum != CarAngle_str.last_CodeNum)
		{/*扫到新码,并且有效,记录数据*/
			temp_angle1 = angleCur;
			CarAngle_str.theta1_bjf_QR = BjfAngle_to_0(temp_angle1);
			/*记录误差*/
			CarAngle_str.theta3_GY_deviation = CarAngle_str.theta1_bjf_QR - CarAngle_str.theta2_GY;
			CarAngle_str.last_CodeNum = CarAngle_str.now_CodeNum;
			printf("+++++++++++++++++++++++++++++++++++误差修正中......\n");
			ROS_INFO("NowCode :%f, LastCode :%f",CarAngle_str.now_CodeNum,CarAngle_str.last_CodeNum);
			ROS_INFO("BJF_angle :%f, GY_angle :%f",CarAngle_str.theta1_bjf_QR,CarAngle_str.theta2_GY);
			printf("+++++++++++++++++++++++++++++++++误差修正结束......\n");
			printf("\n");
		}
	CarAngle_str.theta4_real_angle = CarAngle_str.theta3_GY_deviation + CarAngle_str.theta2_GY;
	center_speed_angle = get_center_angle(wheel_data.a1,wheel_data.a2,wheel_data.a3,wheel_data.a4);
	printf("陀螺角度:%f\n",CarAngle_str.theta4_real_angle);
	printf("舵轮中心偏向角:%f\n",center_speed_angle);
	return (CarAngle_str.theta4_real_angle - center_speed_angle);
}

/**************************************************************
name		:car_walk_correction
function	:实现小车行走矫正算法
input		:
output		:none
***************************************************************/
int car_walk_correction(int cmd_mode)
{
	float car_mileage = 0.0;
	float car_angle = 0.0;
	float car_dis_x = 0.0;
	float car_dis_y = 0.0;
	int status = 0;
	float rad_angle = 0.0;
//	static int DBG_step_count = 0;
	float last_angle = 0.0;
	float totle_gy_x = 0.0;
	float totle_center_x = 0.0;	
	int res = 0;
	/*获取小车里程*/
	car_mileage = get_car_mileage(cmd_mode);	
	if(car_mileage < 0)
		{
			printf("小车里程计算有误,car_mileage = %f\n",car_mileage);
			return -1;
		}
	else if(car_mileage <= 0.00201)
		{
			//printf("里程速度数据不足100,当前里程为：%f\n",car_mileage);
			return -1;
		}
	else //里程获取结束
		{
			status = 1; 
		}
	/*获取小车偏移角度*/
	car_angle = get_car_angle();
//	ROS_INFO(" Walk function >> CAR_angle:+++++++++++++++++++++++%f",car_angle);	
	car_angle = CarAngle_str.theta4_real_angle;
//	ROS_INFO(" Walk function >> CAR_angle:+++++++++++++++++++++++%f",car_angle);
	
	#if 1 
	if(fabs(fabs(car_angle) - fabs(last_angle)) > 5)
		{
			car_angle = last_angle;
		}
	else
		{
			last_angle = car_angle;
		}
	#endif
	if(test_err_flag)
		{
		//	test_err_flag = 0;
			car_angle = car_angle - test_err_val;
			//printf("程序运行到这里，减去误差\n");
		}
	rad_angle = (3.14159/180)*(car_angle*angle_xishu);
	/*获取小车偏移距离*/
	//ROS_INFO("tmp_mileage:%f" ,CarMileage_str.car_tmp_mileage);
	//ROS_INFO("offset_x before:%f",CarMileage_str.offset_x);
	//ROS_INFO("offset_x now:%f",sin(rad_angle)*fabs(CarMileage_str.car_tmp_mileage));
	#if 1
	CarMileage_str.offset_x += CarMileage_str.car_tmp_offset_x*(1);
	totle_center_x += CarMileage_str.car_tmp_offset_x*(1);
	
	CarMileage_str.offset_x += sin(rad_angle)*CarMileage_str.car_tmp_mileage*(-1);
	totle_gy_x += sin(rad_angle)*CarMileage_str.car_tmp_mileage*(-1);
		
	CarMileage_str.offset_y += cos(rad_angle)*CarMileage_str.car_tmp_mileage;
	printf("小车陀螺累计偏移量：%f\n",totle_gy_x);
	printf("小车行走累计偏移量：%f\n",totle_center_x);
	#else
	CarMileage_str.offset_x += sin(fabs(rad_angle))*CarMileage_str.car_tmp_mileage;
	CarMileage_str.offset_y += cos(fabs(rad_angle))*CarMileage_str.car_tmp_mileage;
	#endif
	//ROS_INFO("offset_x after:%f",CarMileage_str.offset_x);
	ROS_INFO("Now_Mileage:%f,  angle:%f,  offset_X:%f,  offset_Y:%f\n",car_mileage,car_angle*angle_xishu,CarMileage_str.offset_x,CarMileage_str.offset_y);
	/*快到码的时候调用_xTheat2*/
//void run_correction_xTheta2(float v,float dis_x,float dis_theta,float distance,float agv_dir)
	res = run_correction_xTheta2(Qwalk_speed,CarMileage_str.offset_x*(1000),car_angle*(angle_dir),Qcode_distence*1000+2000-(CarMileage_str.offset_y*1000),0);
	if(CarMileage_str.offset_y >= Qcode_distence)
		{
			Qwalk_speed = 0.2;
		}
	Qcar_angle = car_angle;
	if(res == -1)
		return -3;
	/*扫到码的时候清除陀螺误差*/
	return 0;	
}

/*误差计算*/

/*误差修正*/

/*陀螺仪测试函数*/
void gyroscope_cor_test(void)
{
	time_t t_start, t_now;
	
	t_start = time(NULL);
	wheel_data.a1 = -5;
	wheel_data.a2 = -5;
	odometer_start();	
	test_err_val = gyroscope_angle;
	printf("angle_err_val:%f\n",test_err_val);
	sleep(0.5);
	test_err_val += gyroscope_angle;
	printf("angle_err_val:%f\n",test_err_val);
	test_err_val = test_err_val/2; 
	printf("angle_err_val:%f\n",test_err_val);
	test_err_flag = 1;
	while(1)
		{
			t_now = time(NULL);
			if(t_now - t_start > 10)
				{
					printf("超时退出\n");
					Speed_AGV(0);
					Stop_AGV();
					Speed_AGV(0.002);
					sleep(50);
					car_walk_correction(1);
					odometer_stop();
					break ;
				}
			car_walk_correction(0);
		}	
}




