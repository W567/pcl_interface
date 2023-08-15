#ifndef __TIP_CON_PUB_SRV_H__
#define __TIP_CON_PUB_SRV_H__

#include <vector>

#include "cloud_type.h"  // #define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_interface/TipConPub.h"

#include <tf2_ros/transform_broadcaster.h>

class TipConPubServer {
    int R[5] = {255, 255};
    int G[5] = {122, 180};
    int B[5] = {  0, 230};

  private:
    ros::NodeHandle nh_;
    ros::Publisher tip_pubs_;
    ros::Publisher con_pubs_;
    ros::ServiceServer server_;
    
    int loop_rate_;
    
    std::string tip_prefix_;
    std::string con_prefix_;
    std::string base_frame_;

    sensor_msgs::PointCloud2 tip_cloud_;
    sensor_msgs::PointCloud2 con_cloud_;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped palm_tf;

    bool received;

    bool
    pubCallback(pcl_interface::TipConPub::Request  &req,
                pcl_interface::TipConPub::Response &res);

  public:
    TipConPubServer();
    
    void
    exe();
};

#endif 
