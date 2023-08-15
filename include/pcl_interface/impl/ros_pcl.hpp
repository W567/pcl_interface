template <typename T>
void
convert(
    const sensor_msgs::PointCloud2ConstPtr &input,
    const typename pcl::PointCloud<T>::Ptr &output)
{
  fromROSMsg(*input, *output);
}

template <typename T>
void
transform(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const std::string &target_frame_id_,
    const tf::TransformListener &tf_listener_)
{
  try
  {
    if (pcl_ros::transformPointCloud(target_frame_id_, *input, *output,
                                     tf_listener_))
    { return; }
  }
  catch (tf2::ConnectivityException &e)
  {
    ROS_ERROR("Transform error: %s", e.what());
  }
  catch (tf2::InvalidArgumentException &e)
  {
    ROS_ERROR("Transform error: %s", e.what());
  }
  catch (...)
  {
    ROS_ERROR("Unknown transform error");
  }
}


