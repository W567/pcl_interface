#ifndef __WS_PC_PUB_H__
#define __WS_PC_PUB_H__

#include <vector>

#include "cloud_type.h"  // #define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class WsPCPublisher {
    int R[5] = {255,   0,   0, 255, 100};
    int G[5] = {  0, 255,   0, 255, 100};
    int B[5] = {  0,   0, 255,   0, 100};

  private:
    ros::NodeHandle nh_;
    std::vector<ros::Publisher> pubs_;
    
    int loop_rate_;
    
    std::string base_frame_;

    std::vector<std::string> fin_pc_names;
    
    std::vector<sensor_msgs::PointCloud2> clouds_;

  public:
    WsPCPublisher();
    
    void
    exe();
};

#endif 
