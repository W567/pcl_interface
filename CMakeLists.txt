cmake_minimum_required(VERSION 3.0.2)
project(pcl_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  pcl_ros
  pcl_conversions
  eigen_conversions
  tf
  tf_conversions
  tf2_ros
  jsk_recognition_utils
  )

add_service_files(
  FILES
  num.srv
  poseTF.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs cv_bridge pcl_ros pcl_conversions
                 eigen_conversions tf tf_conversions tf2_ros jsk_recognition_utils
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

