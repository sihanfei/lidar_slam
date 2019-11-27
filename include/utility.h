#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv/cv.h>

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

// TW-16
typedef pcl::PointXYZI PointType;

typedef struct
{
  PointType point;
  int8_t label;
  size_t id;
} rangePoint;

const string pointCloudTopic = "/tensorpro_cloud";

const int N_SCAN = 16;
const int Horizon_SCAN = 566;
const float offset_ang_x = 0.1 - 120.0;                 // 实际水平角度是从120度开始的(120~240)
const float ang_res_x = 120 / float(Horizon_SCAN - 1);  // 扫描角度是120度
const float ang_res_y = 11 / float(N_SCAN - 1);
const float offset_ang_y = 5.5 + 0.1;
const int groundScanInd = N_SCAN / 2 - 1;

const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

// 可配置参数
float mount_height = 1.3;                  // 安装高度
float mount_angle = 0;                     // 安装角度,下倾为正
float ground_angle_threshold = 5;          // 地面倾角阈值
float ground_height_threshold = 0.3;       // 地面高度阈值
float segmentTheta = 40.0 / 180.0 * M_PI;  // 障碍物分割角度阈值 decrese this value may improve accuracy
int segmentValidPointNum = 5;              // 聚类大小
int segmentValidLineNum = 3;               // 聚类判断的垂直方向数量
float segmentDistance = 0.5;               // 聚类分割距离阈值

#endif
