#include "pcl_interface/obj_pc_pub_save.h"

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_pc_publisher_save");
  ObjPCPublisher_save pub;
  pub.exe();
  return 0;
}
