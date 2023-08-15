#include "pcl_interface/ws_pc_pub.h"

WsPCPublisher::WsPCPublisher()
{
  if (!nh_.getParam("fin_pc_names", fin_pc_names))
  {
    ROS_ERROR("fin_pc_names invalid");
  }
  
  if (!nh_.getParam("ws_base_frame", base_frame_))
  {
    ROS_WARN("~ws_base_frame is not specified, using %s", "/palm");
    base_frame_ = "palm";
  }

  nh_.param("loop_rate", loop_rate_, 10);

  for (int i = 0; i < fin_pc_names.size(); i++)
  {
    ros::Publisher pub_tmp = nh_.advertise<sensor_msgs::PointCloud2>("pc_wk" + std::to_string(i), 10);
    pubs_.push_back(pub_tmp);
  }

  ROS_INFO("WsPCPublisher on");
}

void
WsPCPublisher::exe()
{
  ros::Rate loop_rate(loop_rate_);

  for (int i = 0; i < fin_pc_names.size(); i++)
  {
    xyzPtr input (new xyz);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fin_pc_names[i], *input);
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
  
  while (ros::ok())
  {
    for (int i = 0; i < fin_pc_names.size(); i++)
    {
      pubs_[i].publish(clouds_[i]);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return;
}
