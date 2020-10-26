#ifndef CLEANER_TOF_H
#define CLEANER_TOF_H

#include <iostream>
#include <thread>
#include <sys/time.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include "tof_dev_sdk.h"
#include <unistd.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SoSleep(a) sleep(a) 

class CleanerTof
{
public:
//默认构造与析构
CleanerTof();
~CleanerTof(){};

//初始化函数，完成读取全局参数，(初始化相机等)
int Init();

//线程管理，启动所有线程工作
void StartAllThread();

//操作相机
void StartCleaner();

public:
    TofDeviceDescriptor* mDevsDescList;

private:
/*ros订阅与发布部分*/
ros::NodeHandle nh;                      //句柄  

public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    //当有eigen成员时，需要加上这一个声明，避免对齐问题
};

#endif
