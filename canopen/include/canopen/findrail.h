#ifndef __FINDRAIL_H__
#define __FINDRAIL_H__

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

void sickCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void findrail(float *offset,float *height);

#endif