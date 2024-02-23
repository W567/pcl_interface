cmake_minimum_required(VERSION 3.0.2)
project(pcl_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
  eigen_conversions
  tf
  tf2_ros
  tf_conversions
  )

add_service_files(
  FILES
  num.srv
  poseTF.srv
  curvFilter.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs
                 pcl_ros pcl_conversions eigen_conversions 
                 tf tf2_ros tf_conversions
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

