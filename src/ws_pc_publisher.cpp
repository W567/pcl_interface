#include "pcl_interface/ws_pc_pub.h"

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "ws_pc_publisher");
  WsPCPublisher pub;
  pub.exe();
  return 0;
}
