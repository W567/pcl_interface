#ifndef __WS_PREPRO_PUB_SRV_H__
#define __WS_PREPRO_PUB_SRV_H__

#include <vector>

#include "cloud_type.h"  // #define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_interface/PreProPCPub.h"

class WsPreProPubServer {
    int R[5] = {255,   0,   0, 255, 100};
    int G[5] = {  0, 255,   0, 255, 100};
    int B[5] = {  0,   0, 255,   0, 100};

  private:
    ros::NodeHandle nh_;
    std::vector<ros::Publisher> pubs_;
    ros::ServiceServer server_;
    
    int loop_rate_;
    
    std::string base_frame_;

    std::string fin_prepro_name_prefix;
    
    std::vector<sensor_msgs::PointCloud2> clouds_;

    bool
    pubCallback(pcl_interface::PreProPCPub::Request  &req,
                pcl_interface::PreProPCPub::Response &res);

  public:
    WsPreProPubServer();

    void
    exe();
};

#endif 
