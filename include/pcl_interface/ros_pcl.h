#pragma once

#include "cloud_type.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>


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
    tf::poseMsgToEigen(m, e);
}

void
poseEigen2Msg(
   Eigen::Affine3d &e,
   geometry_msgs::Pose &m)
{
    tf::poseEigenToMsg(e, m);
}

#include "impl/ros_pcl.hpp"
