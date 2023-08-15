#include "pcl_interface/tip_con_pub_srv.h"

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "tip_con_pub_server");
  TipConPubServer pub;
  pub.exe();
  return 0;
}
