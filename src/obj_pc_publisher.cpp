#include "pcl_interface/obj_pc_pub.h"

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_pc_publisher");
  ObjPCPublisher pub;
  pub.exe();
  return 0;
}
