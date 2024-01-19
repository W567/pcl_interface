#!/usr/bin/env python
import tf
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

        if rospy.has_param("~pc_filenames"):
            self.pc_filenames = rospy.get_param("~pc_filenames")
            self.pc_filename_prefix = None
            self.sub_pc_topics = None
        elif rospy.has_param("~pc_filename_prefix"):
            self.pc_filename_prefix = rospy.get_param("~pc_filename_prefix")
            self.pc_filenames = []
            self.sub_pc_topics = None
        elif rospy.has_param("~sub_pc_topics"):
            self.pc_filenames = []
            self.pc_filename_prefix = None
            self.sub_pc_topics = rospy.get_param("~sub_pc_topics")
        else:
            rospy.logerr("[pcPubBase] No pc_filenames or pc_filename_prefix")

        if rospy.has_param("~pc_topics"):
            self.pc_topics = rospy.get_param("~pc_topics")
            self.pc_topic_prefix = None
        elif rospy.has_param("~pc_topic_prefix"):
            self.pc_topic_prefix = rospy.get_param("~pc_topic_prefix")
            self.pc_topics = [self.pc_topic_prefix + str(i) for i in range(len(self.pc_filenames))]
        else:
            rospy.logerr("[pcPubBase] No pc_topics or pc_topic_prefix")

        if rospy.has_param("~pc_frame"):
            self.pc_frame = rospy.get_param("~pc_frame")
        else:
            rospy.logerr("[pcPubBase] No pc_frame")
        
        self.pc_colors = rospy.get_param("~pc_colors", [[1.0, 0.0, 0.0],
                                                        [0.0, 1.0, 0.0],
                                                        [0.0, 0.0, 1.0],
                                                        [1.0, 1.0, 0.0],
                                                        [1.0, 0.0, 1.0],
                                                        [0.0, 1.0, 1.0],
                                                        [1.0, 1.0, 1.0]])

        assert len(self.pc_filenames) <= len(self.pc_colors) and \
               len(self.pc_topics) <= len(self.pc_colors), \
            "[pcPubBase] Color length imcompatible"
        
        self.tf_listener = tf.TransformListener()

        self.pcd_list = []
        self.pc_sub_list = []
        self.pc_pub_list = []
        self.pc_head_list = []
        rospy.loginfo("[pcPubBase] Initialized")


    def init_pc_sub(self):
        for i, pc_topic in enumerate(self.sub_pc_topics):
            pc_sub = rospy.Subscriber(pc_topic, PointCloud2, self.pc_sub_cb, callback_args=i)
            self.pc_sub_list.append(pc_sub)
            self.pcd_list.append(None)


    def pc_sub_cb(self, msg, i):
        np_cloud_dict = pc22np(msg)
        if np_cloud_dict == None:
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_cloud_dict['xyz'][0])
        if 'rgb' in np_cloud_dict.keys():
            pcd.colors = o3d.utility.Vector3dVector(np_cloud_dict['rgb'][0])
        if 'nor' in np_cloud_dict.keys():
            pcd.normals = o3d.utility.Vector3dVector(np_cloud_dict['nor'][0])
        self.pcd_list[i] = pcd


    def init_pc_read(self):
        for i in range(len(self.pc_filenames)):
            pcd = o3d.io.read_point_cloud(self.pc_filenames[i])
            if min(self.pc_colors[i]) >= 0 and max(self.pc_colors[i]) <= 1:
                pcd.paint_uniform_color(self.pc_colors[i])
            self.pcd_list.append(pcd)


    def init_pc_pub(self):
        for pc_topic in self.pc_topics:
            pc_pub = rospy.Publisher(pc_topic, PointCloud2, queue_size=1, latch=True)
            self.pc_pub_list.append(pc_pub)
            pc_header = Header()
            pc_header.frame_id = self.pc_frame
            self.pc_head_list.append(pc_header)


    def init_pc(self):
        if self.sub_pc_topics != None:
            self.init_pc_sub()
        elif self.pc_filenames != []:
            self.init_pc_read()
        else:
            rospy.logerr("[pcPubBase] No pc_filenames/sub_pc_topics(pc_filename_prefix not supported)")
            return
        self.init_pc_pub()


    def pcd_process(self, pcd):
        return pcd


    def execute(self):
        self.init_pc()
        while not rospy.is_shutdown():
            for pcd, head, pub in zip(self.pcd_list, self.pc_head_list, self.pc_pub_list):
                if pcd == None:
                    continue
                pcd = self.pcd_process(pcd)
                pc_msg = o3d2pc2(pcd, head)
                pub.publish(pc_msg)

            self.rate.sleep()