#include "pcl_interface/tip_con_pub_srv.h"

TipConPubServer::TipConPubServer()
{
  if (!nh_.getParam("tip_prefix", tip_prefix_))
  {
    ROS_WARN("tip_prefix invalid, using %s", "tip_");
    tip_prefix_ = "tip_";
  }
  
  if (!nh_.getParam("con_prefix", con_prefix_))
  {
    ROS_WARN("con_prefix invalid, using %s", "con_");
    con_prefix_ = "con_";
  }

  if (!nh_.getParam("obj_base_frame", base_frame_))
  {
    ROS_WARN("~obj_base_frame is not specified, using %s", "/base_link");
    base_frame_ = "base_link";
  }

  nh_.param("loop_rate", loop_rate_, 10);

  tip_pubs_ = nh_.advertise<sensor_msgs::PointCloud2>("tip_pc", 10);
  con_pubs_ = nh_.advertise<sensor_msgs::PointCloud2>("con_pc", 10);

  server_ = nh_.advertiseService("tip_con_pub", &TipConPubServer::pubCallback, this);

  received = false;

  ROS_INFO("TipConPubServer on");
}

bool
TipConPubServer::pubCallback(
    pcl_interface::TipConPub::Request  &req,
    pcl_interface::TipConPub::Response &res)
{
  palm_tf.header.stamp = ros::Time::now();
  palm_tf.header.frame_id = base_frame_;
  palm_tf.child_frame_id = "palm_candidate";
  palm_tf.transform.translation.x = req.rela.position.x;
  palm_tf.transform.translation.y = req.rela.position.y;
  palm_tf.transform.translation.z = req.rela.position.z;

  palm_tf.transform.rotation.x = req.rela.orientation.x;
  palm_tf.transform.rotation.y = req.rela.orientation.y;
  palm_tf.transform.rotation.z = req.rela.orientation.z;
  palm_tf.transform.rotation.w = req.rela.orientation.w;

  xyzPtr tip_input (new xyz);
  pcl::io::loadPCDFile<pcl::PointXYZ>(tip_prefix_ + std::to_string(req.idx) + ".pcd", *tip_input);
  rgbPtr tip_rgb (new rgb);

  pcl::PointXYZRGB element;
  for (auto& point : tip_input->points)
  {
    element.x = point.x;
    element.y = point.y;
    element.z = point.z;
    element.r = R[0];
    element.g = G[0];
    element.b = B[0];
    tip_rgb->points.push_back(element);
  }
  pcl::toROSMsg<pcl::PointXYZRGB>(*tip_rgb, tip_cloud_);
  tip_cloud_.header.frame_id = "palm_candidate";

  xyzPtr con_input (new xyz);
  pcl::io::loadPCDFile<pcl::PointXYZ>(con_prefix_ + std::to_string(req.idx) + ".pcd", *con_input);
  rgbPtr con_rgb (new rgb);

  for (auto& point : con_input->points)
  {
    element.x = point.x;
    element.y = point.y;
    element.z = point.z;
    element.r = R[1];
    element.g = G[1];
    element.b = B[1];
    con_rgb->points.push_back(element);
  }
  pcl::toROSMsg<pcl::PointXYZRGB>(*con_rgb, con_cloud_);
  con_cloud_.header.frame_id = "palm_candidate";

  received = true;

  return true;
}

void
TipConPubServer::exe()
{
  ros::Rate loop_rate(loop_rate_);
  
  while (ros::ok())
  {
    if (received)
    {
      tfb.sendTransform(palm_tf);
      tip_pubs_.publish(tip_cloud_);
      con_pubs_.publish(con_cloud_);
      received = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return;
}
