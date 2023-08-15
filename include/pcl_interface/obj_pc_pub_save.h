#ifndef __OBJ_PC_PUB_SAVE_H__
#define __OBJ_PC_PUB_SAVE_H__

#include "pcl_interface/cloud_type.h"  // #define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include "pcl_interface/pcl_func.h"
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/package.h>

class ObjPCPublisher_save {
  private:
    ros::NodeHandle nh_;
    ros::Publisher  rviz_pub_;
    ros::Publisher  palm_pub_;
    ros::Publisher  pose_pub_;
    ros::Subscriber sub_;
    ros::Subscriber name_sub_;

    bool pose_initialized;
    bool get_name;
    std::string previous_name;
    
    int loop_rate_;
    std::string base_frame_;
    std::string palm_frame_;
    std::string obj_pc_name;
    sensor_msgs::PointCloud2 rviz_cloud_;
    sensor_msgs::PointCloud2 palm_cloud_;

    pnPtr pc_input_;

    Eigen::Affine3d world_trans_;
    Eigen::Affine3d palm_trans_, palm_rot_;
    Eigen::Affine3d obj_bot_pose_;
    geometry_msgs::Pose obj_bot_pose_msg_;
    float centroid[3];
    float h;
    
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    void
    callback(const geometry_msgs::Pose &pose);

    void
    name_callback(const std_msgs::String &str);

  public:
    ObjPCPublisher_save();
    
    void
    exe();
};

#endif 
