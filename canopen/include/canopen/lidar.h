#ifndef __LASER_H__
#define __LASER_H__

#define LOCATION_BY_BACK_WHEEL 1
#define LOCATION_BY_SIDE_WHEEL 2

typedef struct{
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    int data_num;
    float ranges[811];
    float intensities[811];
}laser_scan_data;

typedef struct laser_data{
    float dis;
    float angle;
    float x;
    float y;
    float variance;
    laser_data *prev,*next;
}LASER_DATA;

void lms1xxCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

void sickCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

void sick2Callback(const sensor_msgs::LaserScan::ConstPtr& msg);

void laser_location(float *center_x,float *center_theta,float *center_dis,float *side_dis,float *side_theta);

int linearity_calculation(float x[][100],float y[][100],int *num,int *line_num);

void wheel_find_by_dbscan(float *angle,float *dis,float wheel_angle[][100],float wheel_dis[][100],int num[][2]);

void lidar_calibration(float *dis,float *theta);

void plane_find_by_dbscan(float *angle,float *dis,int num[2]);

void update_to_server(float dis,float theta);

#endif