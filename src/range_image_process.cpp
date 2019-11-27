#include "../include/utility.h"

class RangeImageProcess
{
private:
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud;

  ros::Publisher pubFullCloud;
  ros::Publisher pubGroundCloud;
  ros::Publisher pubSegmentedCloud;
  ros::Publisher pubSegmentedCloudPure;

  pcl::PointCloud<PointType>::Ptr laserCloudIn;        // 原始点云
  pcl::PointCloud<PointType>::Ptr fullCloud;           // 将全部点云有序排列成 N_SCAN * Horizon_SCAN
  pcl::PointCloud<PointType>::Ptr groundCloud;         // 地面点云
  pcl::PointCloud<PointType>::Ptr segmentedCloud;      // 无地面点云,准备用于聚类
  pcl::PointCloud<PointType>::Ptr segmentedCloudPure;  // 聚类完成点云

  PointType nanPoint;  // fill in fullCloud at each iteration

  cv::Mat rangeMat;   // range matrix for range image
  cv::Mat groundMat;  // 用于标记地面点 -1 无效 0 非地面点 1 地面点
  cv::Mat objectMat;  // 目标物标记
  int objectCount;

  std::vector<pcl::PointCloud<PointType>> segMsg;  // 各个目标点云序列

  std::vector<std::pair<uint8_t, uint8_t>> neighborIterator;  // neighbor iterator for segmentaiton process

  uint16_t *allPushedIndX;  // array for tracking points of a segmented object
  uint16_t *allPushedIndY;

  uint16_t *queueIndX;  // array for breadth-first search process of segmentation
  uint16_t *queueIndY;

  std_msgs::Header cloudHeader;

public:
  RangeImageProcess() : nh("~")
  {
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &RangeImageProcess::cloudHandler, this);

    pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
    pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
    pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    allocateMemory();
    resetParameters();
  }

  void allocateMemory()
  {
    laserCloudIn.reset(new pcl::PointCloud<PointType>());

    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullCloud->points.resize(N_SCAN * Horizon_SCAN);

    groundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());

    allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
  }

  void resetParameters()
  {
    laserCloudIn->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));  // 清空数据 -1:无效 0:初始值 1:地面点
    objectMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));  // 0:非地面 n:目标id

    objectCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
  }

  ~RangeImageProcess()
  {
  }

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    cloudHeader = laserCloudMsg->header;
    // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
    pcl::PointCloud<pcl::PointXYZRGB> tmp_pc_;
    pcl::fromROSMsg(*laserCloudMsg, tmp_pc_);
    pcl::PointCloudXYZRGBtoXYZI(tmp_pc_, *laserCloudIn);
    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
    // // ROS_INFO("\033[1;32m---->\033[0m copy pointcloud, size = %ld.", laserCloudIn->points.size());
  }

  /**
   * 转换为rangeImage,并按照顺序存入fullCloud中
   * */
  void projectPointCloud()
  {
    fullCloud->points.resize(N_SCAN * Horizon_SCAN);

    size_t rowIDn_, columnIDn_;
    float verti_angle_, horizon_angle_;
    float range_;
    PointType this_point_;

    for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
    {
      this_point_.x = laserCloudIn->points[i].x;
      this_point_.y = laserCloudIn->points[i].y;
      this_point_.z = laserCloudIn->points[i].z;

      // 计算行号
      verti_angle_ =
          atan2f(this_point_.z, sqrt(this_point_.x * this_point_.x + this_point_.y * this_point_.y)) * 180 / M_PI;
      verti_angle_ += offset_ang_y;  // 转换为0~11度
      rowIDn_ = verti_angle_ / ang_res_y;
      if (rowIDn_ < 0 || rowIDn_ >= N_SCAN)
      {
        continue;
      }

      // 计算列号
      horizon_angle_ = atan2f(this_point_.y, this_point_.x) * 180 / M_PI;
      if (horizon_angle_ < 0)
      {
        horizon_angle_ += 360;  // 转换到0~360度
      }
      horizon_angle_ += offset_ang_x;  // 从120~240 转换到 0~120
      columnIDn_ = horizon_angle_ / ang_res_x;
      if (columnIDn_ < 0 || columnIDn_ >= Horizon_SCAN)
      {
        continue;
      }

      range_ = sqrt(this_point_.x * this_point_.x + this_point_.y * this_point_.y + this_point_.z * this_point_.z);

      // range image
      // pointMat[rowIDn_][columnIDn_] = this_point_;
      // pointMat[rowIDn_][columnIDn_].intensity = range_;

      // 按行存储
      fullCloud->points[columnIDn_ + rowIDn_ * Horizon_SCAN] = this_point_;  // 按照行排布
      rangeMat.at<float>(rowIDn_, columnIDn_) = range_;
    }
  }

  /**
   * 直接根据离地面高度来判断是否地面
   *
   * 需要注意: 计算z轴高度时,需要加上传感器的安装高度
   * */
  void removeGroundbyHeight(float height)
  {
    PointType this_point_;
    for (size_t i = 0; i < N_SCAN; ++i)
    {
      for (size_t j = 0; j < Horizon_SCAN; ++j)
      {
        this_point_ = fullCloud->points[j + i * Horizon_SCAN];
        if (this_point_.intensity != -1)
        {
          if (this_point_.z + mount_height < height)
          {
            // this_point_.intensity = 0;
            // groundCloud->push_back(this_point_);
            groundMat.at<int8_t>(i, j) = 1;
          }
        }
        else
        {
          groundMat.at<int8_t>(i, j) = -1;  // 无效点
        }
      }
    }
    for (size_t i = 0; i < N_SCAN; ++i)
    {
      for (size_t j = 0; j < Horizon_SCAN; ++j)
      {
        if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX)  // 地面点或者无效点
        {
          objectMat.at<int>(i, j) = -1;
        }
      }
    }
    if (pubGroundCloud.getNumSubscribers() != 0)
    {
      for (size_t i = 0; i <= groundScanInd; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (groundMat.at<int8_t>(i, j) == 1)
            groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
        }
      }
    }
  }

  /**
   * 根据韩国人的文章寻找地面点:A Fast Ground Segmentation Method for 3D Point Cloud
   *    1.前后点角度
   *    2.缺点时高度
   *    3.
   * parameter:None
   * input:
   *    fullcloud
   * output:
   *    groundMat : -1表示无效点/1表示地面点/0表示非地面点
   *    groundCloud: 全部地面点
   * */
  void removeGroundbyRay()
  {
    size_t current_index_, seg_index_, seg_rowID_;
    float angle_;
    PointType current_point_, seg_point_;
    int8_t ground_seg_rowID_mat_[Horizon_SCAN];  // 用于保存每列中最后一个地面点的行序号
    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
      ground_seg_rowID_mat_[j] = -1;
    }

    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
      for (int8_t i = 0; i < groundScanInd; ++i)  // 按列计算
      {
        current_index_ = j + i * Horizon_SCAN;
        current_point_ = fullCloud->points[current_index_];
        if (current_point_.intensity == -1)  // 无效点
        {
          groundMat.at<int8_t>(i, j) = -1;
          ground_seg_rowID_mat_[j] = -1;
        }
        else  // 当前点有效
        {
          if (ground_seg_rowID_mat_[j] == -1)  // 无地面点
          {
            if (i == 0)  // 当前是第一点
            {
              angle_ = atan2f(current_point_.z - seg_point_.z, sqrtf(powf(current_point_.y - seg_point_.y, 2) +
                                                                     powf(current_point_.x - seg_point_.x, 2))) *
                       180 / M_PI;
              if (angle_ - mount_angle > ground_angle_threshold)
              {
                groundMat.at<int8_t>(i, j) = 0;
              }
              else
              {
                groundMat.at<int8_t>(i, j) = 1;
                ground_seg_rowID_mat_[j] = i;
                groundCloud->push_back(current_point_);
              }
            }
            else
            {
              if (current_point_.z - (-mount_height) < ground_height_threshold)  // 高度在阈值范围内
              {
                groundMat.at<int8_t>(i, j) = 1;
                ground_seg_rowID_mat_[j] = i;
                groundCloud->push_back(current_point_);
              }
              else
              {
                groundMat.at<int8_t>(i, j) = 0;
              }
            }
          }
          else  // 有地面点
          {
            seg_rowID_ = ground_seg_rowID_mat_[j];
            seg_index_ = j + seg_rowID_ * Horizon_SCAN;
            seg_point_ = fullCloud->points[seg_index_];
            if (rangeMat.at<float>(i, j) <= rangeMat.at<float>(seg_rowID_, j) ||
                current_point_.z - seg_point_.z > ground_height_threshold)  // 当前点range更短,或者z高度差超过阈值
            {
              groundMat.at<int8_t>(i, j) = 0;
            }
            else  // z高度差在阈值内,且range更长
            {
              if (i - seg_rowID_ > 1)  // 不是连续点,则只需要考虑高度差
              {
                groundMat.at<int8_t>(i, j) = 1;
                ground_seg_rowID_mat_[j] = i;
                groundCloud->push_back(current_point_);
              }
              else  // 连续点则判断角度
              {
                angle_ = atan2f(current_point_.z - seg_point_.z, sqrtf(powf(current_point_.y - seg_point_.y, 2) +
                                                                       powf(current_point_.x - seg_point_.x, 2))) *
                         180 / M_PI;
                if (angle_ - mount_angle > ground_angle_threshold)
                {
                  groundMat.at<int8_t>(i, j) = 0;
                }
                else
                {
                  groundMat.at<int8_t>(i, j) = 1;
                  ground_seg_rowID_mat_[j] = i;
                  groundCloud->push_back(current_point_);
                }
              }
            }
          }
        }
      }
    }
    ROS_INFO("\033[1;32m---->\033[0m groundCloud size = %ld.", groundCloud->points.size());
  }

  /**
   * 根据RANSAC方法拟合地平面方程
   *
   * */
  void getGroundDescript()
  {
    ;
  }

  void publishCloud()
  {
    // 2. Publish clouds
    sensor_msgs::PointCloud2 laserCloudTemp;

    // projected full cloud
    if (pubFullCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*fullCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "TanwayTP";
      pubFullCloud.publish(laserCloudTemp);
    }

    // ground cloud
    if (pubGroundCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*groundCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "TanwayTP";
      pubGroundCloud.publish(laserCloudTemp);
    }

    // segmented cloud
    if (pubSegmentedCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "TanwayTP";
      pubSegmentedCloud.publish(laserCloudTemp);
    }

    // segmented pure cloud
    if (pubSegmentedCloudPure.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "TanwayTP";
      pubSegmentedCloudPure.publish(laserCloudTemp);
    }
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    // 1. Convert ros message to pcl point cloud
    copyPointCloud(laserCloudMsg);  // done
    // 2. Start and end angle of a scan
    // // 3. Range image projection
    projectPointCloud();
    // // 4. Mark ground points
    // // groundRemoval();
    removeGroundbyHeight(ground_height_threshold);
    // // 5. Point cloud segmentation
    cloudSegmentation();
    // // 6. Publish all clouds
    publishCloud();
    // // 7. Reset parameters for next iteration
    resetParameters();
  }

  /**
   * 根据论文<Fast  Range  Image-Based  Segmentationof  Sparse  3D  Laser  Scans  for  Online  Operation>
   * 通过计算两点之间的角度来判断是否是同一目标
   * */
  void cloudSegmentation()
  {
    // segmentation process
    for (size_t i = 0; i < N_SCAN; ++i)
      for (size_t j = 0; j < Horizon_SCAN; ++j)
        if (objectMat.at<int>(i, j) == 0)  // 有效点
        {
          labelComponents(i, j);  // ToDo 仔细检查,不行重写
        }

    int sizeOfSegCloud = 0;  // 目标物个数
    // // extract segmented cloud for lidar odometry
    // for (size_t i = 0; i < N_SCAN; ++i)
    // {
    //   segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

    //   for (size_t j = 0; j < Horizon_SCAN; ++j)
    //   {
    //     if (objectMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1)
    //     {
    //       // outliers that will not be used for optimization (always continue)
    //       if (objectMat.at<int>(i, j) == 999999)
    //       {
    //         if (i > groundScanInd && j % 5 == 0)
    //         {
    //           outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
    //           continue;
    //         }
    //         else
    //         {
    //           continue;
    //         }
    //       }
    //       // majority of ground points are skipped
    //       if (groundMat.at<int8_t>(i, j) == 1)
    //       {
    //         if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
    //           continue;
    //       }
    //       // mark ground points so they will not be considered as edge features later
    //       segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1);
    //       // mark the points' column index for marking occlusion later
    //       segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
    //       // save range info
    //       segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
    //       // save seg cloud
    //       segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
    //       // size of seg cloud
    //       ++sizeOfSegCloud;
    //     }
    //   }

    //   segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
    // }

    // extract segmented cloud for visualization
    size_t object_count_ = 0;
    if (pubSegmentedCloudPure.getNumSubscribers() != 0)
    {
      for (size_t i = 0; i < N_SCAN; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (objectMat.at<int>(i, j) > 0 && objectMat.at<int>(i, j) != 999999)
          {
            segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
            segmentedCloudPure->points.back().intensity = objectMat.at<int>(i, j);
            if (object_count_ < objectMat.at<int>(i, j))
            {
              object_count_ = objectMat.at<int>(i, j);
            }
          }
        }
      }
      ROS_INFO("\033[1;32m---->\033[0m ojbects = %ld, total size = %ld", object_count_,
               segmentedCloudPure->points.size());
    }
  }

  void labelComponents(int row, int col)
  {
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle, distance;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    bool lineCountFlag[N_SCAN] = { false };

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;

    while (queueSize > 0)
    {
      // Pop point
      fromIndX = queueIndX[queueStartInd];
      fromIndY = queueIndY[queueStartInd];
      --queueSize;
      ++queueStartInd;
      // Mark popped point
      objectMat.at<int>(fromIndX, fromIndY) = objectCount;
      // Loop through all the neighboring grids of popped grid
      for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
      {
        // new index
        thisIndX = fromIndX + (*iter).first;
        thisIndY = fromIndY + (*iter).second;
        // index should be within the boundary
        if (thisIndX < 0 || thisIndX >= N_SCAN)
          continue;
        // at range image margin (left or right side) // tanway 是非循环的
        if (thisIndY < 0)
          // thisIndY = Horizon_SCAN - 1;
          continue;
        if (thisIndY >= Horizon_SCAN)
          // thisIndY = 0;
          continue;
        // prevent infinite loop (caused by put already examined point back)
        if (objectMat.at<int>(thisIndX, thisIndY) != 0)
          continue;

        d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));
        d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));

        if ((*iter).first == 0)
          alpha = segmentAlphaX;
        else
          alpha = segmentAlphaY;

        angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));
        distance = sqrt(pow(d2 * sin(alpha), 2) + pow(d1 - d2 * cos(alpha), 2));

        if (angle > segmentTheta || distance < segmentDistance)  // 角度够大,或者距离够近
        {
          queueIndX[queueEndInd] = thisIndX;
          queueIndY[queueEndInd] = thisIndY;
          ++queueSize;
          ++queueEndInd;

          objectMat.at<int>(thisIndX, thisIndY) = objectCount;
          lineCountFlag[thisIndX] = true;

          allPushedIndX[allPushedIndSize] = thisIndX;
          allPushedIndY[allPushedIndSize] = thisIndY;
          ++allPushedIndSize;
        }
      }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
      feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum)
    {
      int lineCount = 0;
      for (size_t i = 0; i < N_SCAN; ++i)
        if (lineCountFlag[i] == true)
          ++lineCount;
      if (lineCount >= segmentValidLineNum)
        feasibleSegment = true;
    }
    // segment is valid, mark these points
    if (feasibleSegment == true)
    {
      ++objectCount;
      // ROS_INFO("\033[1;32m Get a Object.\033[0m");
    }
    else
    {  // segment is invalid, mark these points
      for (size_t i = 0; i < allPushedIndSize; ++i)
      {
        objectMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
      }
    }
  }
};

int main(int argc, char *argv[])
{
  if (argc > 1)
  {
    if (argc % 2 != 0)
    {
      for (int8_t i = 1; i < argc; i += 2)
      {
        if (std::strcmp(argv[i], "-ma"))
        {
          mount_angle = atof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "-mh"))
        {
          mount_height = atof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "-ga"))
        {
          ground_angle_threshold = atof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "-gh"))
        {
          ground_height_threshold = atof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "-sd"))
        {
          segmentDistance = atof(argv[i + 1]);
        }
        if (std::strcmp(argv[i], "-sa"))
        {
          segmentTheta = atof(argv[i + 1]) * M_PI / 180;
        }
      }
    }
    else
    {
      ROS_INFO("\033[1;31m<-the number of parameters is wrong!->\033[0m");
    }
  }
  // todo 判断输入参数是否合理
  ROS_INFO("\033[1;32m<-Parameters: mount_angle=%f, mount_height=%f->\033[0m", mount_angle, mount_height);
  ROS_INFO("\033[1;32m<-Parameters: ground_angle_threshold=%f, ground_height_threshold=%f->\033[0m",
           ground_angle_threshold, ground_height_threshold);

  ros::init(argc, argv, "tanway_lidar_slam");

  RangeImageProcess range_image_process_;

  ROS_INFO("\033[1;32m---->\033[0m range_image_process_ Started.");

  ros::spin();
  return 0;
}
