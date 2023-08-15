#include "pcl_interface/pc_pub_save.h"

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "pc_publisher", ros::init_options::AnonymousName);
  PCPublisher pub;
  pub.exe();
  return 0;
}
