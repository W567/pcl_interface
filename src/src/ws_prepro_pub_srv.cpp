#include "pcl_interface/ws_prepro_pub_srv.h"

WsPreProPubServer::WsPreProPubServer()
{
  if (!nh_.getParam("fin_prepro_name_prefix", fin_prepro_name_prefix))
  {
    ROS_WARN("fin_prepro_name_prefix invalid, using %s", "finPrePro_");
    fin_prepro_name_prefix = "finPrePro_";
  }
  
  if (!nh_.getParam("ws_base_frame", base_frame_))
  {
    ROS_WARN("~ws_base_frame is not specified, using %s", "/palm");
    base_frame_ = "palm";
  }

  nh_.param("loop_rate", loop_rate_, 10);

  server_ = nh_.advertiseService("ws_prepro_pub", &WsPreProPubServer::pubCallback, this);

  ROS_INFO("WsPreProPubServer on");
}

bool
WsPreProPubServer::pubCallback(
    pcl_interface::PreProPCPub::Request  &req,
    pcl_interface::PreProPCPub::Response &res)
{
  for (int i = 0; i < req.num; i++)
  {
    ros::Publisher pub_tmp = nh_.advertise<sensor_msgs::PointCloud2>("pc_wk_prepro_" + std::to_string(i), 10);
    pubs_.push_back(pub_tmp);

    xyzPtr input (new xyz);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fin_prepro_name_prefix + std::to_string(i) + ".pcd", *input);
    rgbPtr input_rgb (new rgb);
    pcl::PointXYZRGB element;
    for (auto& point : input->points)
    {
      element.x = point.x;
      element.y = point.y;
      element.z = point.z;
      element.r = R[i];
      element.g = G[i];
      element.b = B[i];
      input_rgb->points.push_back(element);
    }
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg<pcl::PointXYZRGB>(*input_rgb, cloud);
    cloud.header.frame_id = base_frame_;
    clouds_.push_back(cloud);
  }
  return true;
}

void
WsPreProPubServer::exe()
{
  ros::Rate loop_rate(loop_rate_);
  
  while (ros::ok())
  {
    for (int i = 0; i < clouds_.size(); i++)
    {
      pubs_[i].publish(clouds_[i]);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return;
}
