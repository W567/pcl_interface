#include "pcl_interface/obj_pc_pub_save.h"

ObjPCPublisher_save::ObjPCPublisher_save()
{
  // palm_rot_ = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

  // nh_.param("loop_rate", loop_rate_, 15);
  // sub_ = nh_.subscribe("/obj_pose", 1, &ObjPCPublisher_save::callback, this);
  // rviz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("rviz_obj_pc", 10);
  // palm_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("palm_obj_pc", 10);
  // pose_pub_ = nh_.advertise<geometry_msgs::Pose>("obj_bot_pose", 10);

  name_sub_ = nh_.subscribe("/obj_name", 1, &ObjPCPublisher_save::name_callback, this);

  // pose_initialized = false;
  get_name = false;
  previous_name = "";
  ROS_INFO("ObjPCPublisher_save on");
}

void
ObjPCPublisher_save::name_callback(const std_msgs::String &str)
{
  if (str.data == previous_name)
    return;
  get_name = false;
  previous_name = str.data;
  std::string path = ros::package::getPath("pcd_lib");
  path += "/obj/pcd/" + str.data + ".pcd";
  ROS_WARN_STREAM("=================================================path: " << path);
  pnPtr input_tmp (new pn);
  pcl::io::loadPCDFile<pcl::PointNormal>(path, *input_tmp);
  pc_input_ = input_tmp;
  get_name = true;
  return;
}

// void
// ObjPCPublisher_save::callback(const geometry_msgs::Pose &pose)
// {
//   // geometry_msgs::pose  position/orientation  ->  transformation matrix
//   // transform pc_input_ to pc_trans_
//   tf::poseMsgToEigen(pose, world_trans_);
//   pose_initialized = true;
// }

// void
// ObjPCPublisher_save::exe()
// {
//   ros::Rate loop_rate(loop_rate_);
//   pnPtr pc_trans_ (new pn);

//   while (ros::ok())
//   {
    // if (pose_initialized && get_name)
    // {
    //   pcl::transformPointCloudWithNormals<pcl::PointNormal>(*pc_input_, *pc_trans_, world_trans_);
    //   pcl::toROSMsg<pcl::PointNormal>(*pc_trans_, rviz_cloud_);
    //   rviz_cloud_.header.frame_id = base_frame_;
    //   rviz_pub_.publish(rviz_cloud_);

      // computeCentroid<pcl::PointNormal>(pc_trans_, centroid);
      // h = minZ<pcl::PointNormal>(pc_trans_);
      // palm_trans_ = Eigen::Affine3d::Identity();
      // palm_trans_.translation() << -centroid[0], -centroid[1], -h;
      // palm_trans_.matrix() = palm_rot_.matrix() * palm_trans_.matrix();
      // pcl::transformPointCloudWithNormals<pcl::PointNormal>(*pc_trans_, *pc_trans_, palm_trans_);
      // pcl::toROSMsg<pcl::PointNormal>(*pc_trans_, palm_cloud_);
      // palm_cloud_.header.frame_id = palm_frame_;
      // palm_pub_.publish(palm_cloud_);

      // obj_bot_pose_.matrix() = palm_trans_.matrix().inverse();
      // tf::poseEigenToMsg(obj_bot_pose_, obj_bot_pose_msg_);    // unit: meter
      // pose_pub_.publish(obj_bot_pose_msg_);

    //   transformStamped.header.stamp = ros::Time::now();
    //   transformStamped.header.frame_id = base_frame_;
    //   transformStamped.child_frame_id = "obj_bot";
    //   transformStamped.transform.translation.x = obj_bot_pose_.translation()[0];
    //   transformStamped.transform.translation.y = obj_bot_pose_.translation()[1];
    //   transformStamped.transform.translation.z = obj_bot_pose_.translation()[2];
    //   Eigen::Quaterniond q = (Eigen::Quaterniond)obj_bot_pose_.linear();
    //   transformStamped.transform.rotation.x = q.x();
    //   transformStamped.transform.rotation.y = q.y();
    //   transformStamped.transform.rotation.z = q.z();
    //   transformStamped.transform.rotation.w = q.w();
    //   tfb.sendTransform(transformStamped);
    // }
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return;
// }
