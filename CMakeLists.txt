cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(lidar_slam)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${PCL_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
	${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS})

add_executable(rangeImageProcess
  src/range_image_process.cpp
  # src/lidar_preproc.cpp
  # src/lidar_obstacle.cpp
  # src/lidar_tracking.cpp
)

add_dependencies(rangeImageProcess 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rangeImageProcess
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
