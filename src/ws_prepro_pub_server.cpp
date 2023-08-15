#include "pcl_interface/ws_prepro_pub_srv.h"

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "ws_prepro_pub_server");
  WsPreProPubServer pub;
  pub.exe();
  return 0;
}
