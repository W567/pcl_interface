#!/usr/bin/env python
import rospy
import open3d as o3d
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from pcl_interface.srv import PreProPCPub, PreProPCPubResponse

from pcPubBase import pcPubBase

class pcPubServer(pcPubBase):

    def __init__(self):
        super().__init__()

        self.server = rospy.Service('ws_prepro_pub', PreProPCPub, self.pubCB)

    def pubCB(self, req):
        for i in range(req.num):
            pcd = o3d.io.read_point_cloud(self.pkg_path + self.pc_filename_prefix + str(i) + ".pcd")
            if min(self.pc_colors[i]) >= 0 and max(self.pc_colors[i]) <= 1:
                pcd.paint_uniform_color(self.pc_colors[i])
            self.pcd_list.append(pcd)
            pc_pub = rospy.Publisher(self.pc_topic_prefix + str(i), PointCloud2, queue_size=1, latch=True)
            self.pc_pub_list.append(pc_pub)
            pc_header = Header()
            pc_header.frame_id = self.pc_frame
            self.pc_head_list.append(pc_header)
        return PreProPCPubResponse(True)

    def execute(self):
        while not rospy.is_shutdown():
            for pcd, head, pub in zip(self.pcd_list, self.pc_head_list, self.pc_pub_list):
                pcd = self.pcd_trans(pcd)
                pc_msg = self.o3d2pc2(pcd, head)
                pub.publish(pc_msg)

            self.rate.sleep()

def main():
    pub = pcPubServer()
    pub.execute()

if __name__ == "__main__":
    main()