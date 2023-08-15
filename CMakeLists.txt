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
  PreProPCPub.srv
  TipConPub.srv
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

aux_source_directory(./src/src lib_src)

add_executable(obj_pc_pub src/obj_pc_publisher.cpp ${lib_src})
target_link_libraries(obj_pc_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_executable(ws_pc_pub src/ws_pc_publisher.cpp ${lib_src})
target_link_libraries(ws_pc_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )
  
add_executable(ws_prepro_pub_server src/ws_prepro_pub_server.cpp ${lib_src})
target_link_libraries(ws_prepro_pub_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_executable(tip_con_pub_server src/tip_con_pub_server.cpp ${lib_src})
target_link_libraries(tip_con_pub_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_executable(pc_pub src/pc_publisher.cpp ${lib_src})
target_link_libraries(pc_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_executable(obj_pc_pub_save src/obj_pc_publisher_save.cpp ${lib_src})
target_link_libraries(obj_pc_pub_save
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  )
