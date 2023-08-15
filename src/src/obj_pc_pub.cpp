#include "pcl_interface/obj_pc_pub.h"

ObjPCPublisher::ObjPCPublisher()
{
  if (!nh_.getParam("obj_pc_name", obj_pc_name))
  {
    ROS_ERROR("obj_pc_name invalid");
  }

  pnPtr input_tmp (new pn);
  pcl::io::loadPCDFile<pcl::PointNormal>(obj_pc_name, *input_tmp);
  pc_input_ = input_tmp;

  if (!nh_.getParam("obj_base_frame", base_frame_))
  {
    ROS_WARN("~obj_base_frame is not specified, using %s", "/base_link");
    base_frame_ = "base_link";
  }
  if (!nh_.getParam("ws_base_frame", palm_frame_))
  {
    ROS_WARN("~palm_frame is not specified, using %s", "/palm");
    palm_frame_ = "palm";
  }

  palm_rot_ = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

  nh_.param("loop_rate", loop_rate_, 15);
  sub_ = nh_.subscribe("/obj_pose", 5, &ObjPCPublisher::callback, this);
  rviz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("rviz_obj_pc", 10);
  palm_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("palm_obj_pc", 10);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("obj_bot_pose", 10);

  initialized = false;
  ROS_INFO("ObjPCPublisher on");
}

void
ObjPCPublisher::callback(const geometry_msgs::Pose &pose)
{
  // geometry_msgs::pose  position/orientation  ->  transformation matrix
  // transform pc_input_ to pc_trans_
  tf::poseMsgToEigen(pose, world_trans_);
  initialized = true;
}

void
ObjPCPublisher::exe()
{
  ros::Rate loop_rate(loop_rate_);
  pnPtr pc_trans_ (new pn);

  while (ros::ok())
  {
    if (initialized)
    {
      pcl::transformPointCloudWithNormals<pcl::PointNormal>(*pc_input_, *pc_trans_, world_trans_);
      pcl::toROSMsg<pcl::PointNormal>(*pc_trans_, rviz_cloud_);
      rviz_cloud_.header.frame_id = base_frame_;
      rviz_pub_.publish(rviz_cloud_);

      computeCentroid<pcl::PointNormal>(pc_trans_, centroid);
      h = minZ<pcl::PointNormal>(pc_trans_);
      palm_trans_ = Eigen::Affine3d::Identity();
      palm_trans_.translation() << -centroid[0], -centroid[1], -h;
      palm_trans_.matrix() = palm_rot_.matrix() * palm_trans_.matrix();
      pcl::transformPointCloudWithNormals<pcl::PointNormal>(*pc_trans_, *pc_trans_, palm_trans_);
      pcl::toROSMsg<pcl::PointNormal>(*pc_trans_, palm_cloud_);
      palm_cloud_.header.frame_id = palm_frame_;
      palm_pub_.publish(palm_cloud_);

      obj_bot_pose_.matrix() = palm_trans_.matrix().inverse();
      tf::poseEigenToMsg(obj_bot_pose_, obj_bot_pose_msg_);    // unit: meter
      pose_pub_.publish(obj_bot_pose_msg_);

      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = base_frame_;
      transformStamped.child_frame_id = "obj_bot";
      transformStamped.transform.translation.x = obj_bot_pose_.translation()[0];
      transformStamped.transform.translation.y = obj_bot_pose_.translation()[1];
      transformStamped.transform.translation.z = obj_bot_pose_.translation()[2];
      Eigen::Quaterniond q = (Eigen::Quaterniond)obj_bot_pose_.linear();
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      tfb.sendTransform(transformStamped);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return;
}
