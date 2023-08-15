#include "pcl_interface/pc_pub.h"

PCPublisher::PCPublisher()
{
  if (!nh_.getParam("pc_names", pc_names_))
  { ROS_ERROR("[pc_pub] pc_names invalid"); }

  if (!nh_.getParam("pc_topics", pc_topics_))
  { ROS_ERROR("[pc_pub] pc_topics invalid"); }

  if (!nh_.getParam("tf_frames", tf_frames_))
  { ROS_ERROR("[pc_pub] tf_frames is invalid"); }

  if (!nh_.getParam("pc_colors", pc_colors_))
  {
    ROS_WARN("[pc_pub] pc_colors invalid");
    for (int i = 0; i < 3 * pc_names_.size(); i++)
    {
      pc_colors_.push_back(0);
    }
  }

  nh_.param("loop_rate", loop_rate_, 10);

  for (int i = 0; i < pc_names_.size(); i++)
  {
    ros::Publisher pub_tmp = nh_.advertise<sensor_msgs::PointCloud2>(pc_topics_[i], 10);
    pc_pubs_.push_back(pub_tmp);
  }

  ROS_INFO("[pc_pub] PCPublisher on");
}

void
PCPublisher::exe()
{
  ros::Rate loop_rate(loop_rate_);

  for (int i = 0; i < pc_names_.size(); i++)
  {
    xyzPtr input (new xyz);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pc_names_[i], *input);
    rgbPtr input_rgb (new rgb);
    pcl::PointXYZRGB element;
    for (auto& point : input->points)
    {
      element.x = point.x;
      element.y = point.y;
      element.z = point.z;
      element.r = pc_colors_[3 * i];
      element.g = pc_colors_[3 * i + 1];
      element.b = pc_colors_[3 * i + 2];
      input_rgb->points.push_back(element);
    }
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg<pcl::PointXYZRGB>(*input_rgb, cloud);
    cloud.header.frame_id = tf_frames_[i];
    clouds_.push_back(cloud);
  }
  
  while (ros::ok())
  {
    for (int i = 0; i < pc_names_.size(); i++)
    {
      pc_pubs_[i].publish(clouds_[i]);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return;
}
