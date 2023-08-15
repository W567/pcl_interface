#ifndef __PC_PUB_H__
#define __PC_PUB_H__

#include <vector>

#include "cloud_type.h"  // #define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class PCPublisher {
  private:
    ros::NodeHandle nh_;

    int loop_rate_;

    std::vector<std::string> pc_names_;
    std::vector<std::string> pc_topics_;
    std::vector<ros::Publisher> pc_pubs_;
    std::vector<std::string> tf_frames_;
    std::vector<int> pc_colors_;
    std::vector<sensor_msgs::PointCloud2> clouds_;

  public:
    PCPublisher();
    
    void
    exe();
};

#endif 
