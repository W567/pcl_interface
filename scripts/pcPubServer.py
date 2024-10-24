#!/usr/bin/env python
import rospy
from pcBase import *
import open3d as o3d
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from pcl_interface.srv import num, numResponse

from pcPubBase import pcPubBase

class pcPubServer(pcPubBase):

    def __init__(self):
        super().__init__()
        srv_name = rospy.get_param("~srv_name")
        self.is_single = rospy.get_param("~is_single", False)
        self.server = rospy.Service(srv_name, num, self.pubCB)

        self.pc_colors = rospy.get_param("~pc_colors", self.pc_colors)

    def init_file_topic_frame(self):
        if rospy.has_param("~pc_filename_prefix"):
            self.pc_filename_prefix = rospy.get_param("~pc_filename_prefix")
        else:
            rospy.logerr("[pcPubBase] No pc_filename_prefix")

        self.pc_frame = rospy.get_param('pc_frame', 'palm_candidate')

        self.pc_topics = rospy.get_param("~pc_topics", [])
        self.pc_topic_prefix = rospy.get_param("~pc_topic_prefix", None)

    def pubCB(self, req):
        if self.is_single:
            idxs = [req.num]
        else:
            idxs = range(req.num)
        
        if self.pc_topics == [] and self.pc_topic_prefix != None:
            pc_topics = [self.pc_topic_prefix + str(i) for i in idxs]
        elif self.pc_topic_prefix == None and self.pc_topics != []:
            pc_topics = self.pc_topics
        elif self.pc_topic_prefix != None and self.pc_topics != []:
            pc_topics = self.pc_topics
            rospy.logwarn("[pcPubServer] Both pc_topics and pc_topic_prefix given. Using pc_topics")
        else:
            rospy.logerr("[pcPubServer] No pc_topics or pc_topic_prefix given")
            return numResponse(False)
        
        assert len(pc_topics) == len(idxs), "[pcPubServer] pc_topics and idxs have different lengths"

        for i, id in enumerate(idxs):
            pcd = o3d.io.read_point_cloud(f"{self.pc_filename_prefix}{id}.pcd")
            if min(self.pc_colors[i]) >= 0 and max(self.pc_colors[i]) <= 1:
                pcd.paint_uniform_color(self.pc_colors[i])
            self.pcd_list.append(pcd)
            pc_pub = rospy.Publisher(pc_topics[i], PointCloud2, queue_size=1, latch=True)
            self.pc_pub_list.append(pc_pub)
            pc_header = Header()
            pc_header.frame_id = self.pc_frame
            self.pc_head_list.append(pc_header)
        return numResponse(True)

    def execute(self):
        while not rospy.is_shutdown():
            for i, (pcd, head, pub) in enumerate(zip(self.pcd_list, self.pc_head_list, self.pc_pub_list)):
                pcd = self.pcd_process(pcd, i)
                pc_msg = o3d2pc2(pcd, head)
                pub.publish(pc_msg)

            self.rate.sleep()

def main():
    pub = pcPubServer()
    pub.execute()

if __name__ == "__main__":
    main()