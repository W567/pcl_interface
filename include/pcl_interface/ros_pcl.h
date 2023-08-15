#pragma once

#include "cloud_type.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer_client.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>

template <typename T>
void
convert(
    const sensor_msgs::PointCloud2ConstPtr &input,
    const typename pcl::PointCloud<T>::Ptr &output);

template <typename T>
void
transform(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const std::string &target_frame_id_,
    const tf::TransformListener &tf_listener_);

void
poseMsg2Eigen(
    const geometry_msgs::Pose &m,
    Eigen::Affine3d &e)
{ 
  // Eigen::Affine3d tmp;
  tf::poseMsgToEigen(m, e);
  // e = tmp.cast<float>();
}

void
poseEigen2Msg(
   Eigen::Affine3d &e,
   geometry_msgs::Pose &m)
{ tf::poseEigenToMsg(e, m); }

#include "impl/ros_pcl.hpp"
