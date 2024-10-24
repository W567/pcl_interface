#!/usr/bin/env python
import rospy
import open3d as o3d
from pcBase import *
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

import rospkg
rospack = rospkg.RosPack()

class pcPubBase():

    def __init__(self):
        rospy.init_node('pcPubBase', anonymous=True)

        rate = rospy.get_param("~rate", 30)
        self.rate = rospy.Rate(rate)

        self.init_file_topic_frame()

        self.color_id = 0
        self.pc_colors = [[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0],
                          [1.0, 1.0, 0.0],
                          [1.0, 0.0, 1.0],
                          [0.0, 1.0, 1.0],
                          [1.0, 1.0, 1.0]]

        self.pcd_list = []
        self.pc_pub_list = []
        self.pc_head_list = []
        rospy.loginfo("[pcPubBase] Initialized")

    def init_file_topic_frame(self):
        raise NotImplementedError("[pcPubBase] init_file_topic_frame not implemented")

    def init_pc_read(self):
        for i in range(len(self.pc_file_paths)):
            pcd = o3d.io.read_point_cloud(self.pc_file_paths[i])
            if len(np.asarray(pcd.colors)) == 0:
                pcd.paint_uniform_color(self.pc_colors[self.color_id])
                self.color_id += 1
            self.pcd_list.append(pcd)

    def init_pc_pub(self):
        if self.pc_frames == None:
            raise ValueError("[pcPubBase] pc_frame not set")
        for pc_topic, pc_frame in zip(self.pc_topics, self.pc_frames):
            pc_pub = rospy.Publisher(pc_topic, PointCloud2, queue_size=1, latch=True)
            self.pc_pub_list.append(pc_pub)
            pc_header = Header()
            pc_header.frame_id = pc_frame
            self.pc_head_list.append(pc_header)

    def pcd_process(self, pcd, i):
        return pcd

    def execute(self):
        self.init_pc_read()
        self.init_pc_pub()
        while not rospy.is_shutdown():
            for i, (pcd, head, pub) in enumerate(zip(self.pcd_list, self.pc_head_list, self.pc_pub_list)):
                if pcd == None:
                    continue
                pcd = self.pcd_process(pcd, i)
                if pcd == None:
                    continue
                pc_msg = o3d2pc2(pcd, head)
                pub.publish(pc_msg)

            self.rate.sleep()