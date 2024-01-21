#!/usr/bin/env python
import rospy
import open3d as o3d
from pcBase import *
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

class pcSrvBase():

    def __init__(self):
        rospy.init_node('pcSrvBase', anonymous=True)
        rate = rospy.get_param("~rate", 30)
        self.rate = rospy.Rate(rate)

        self.with_pub = rospy.get_param("~with_pub", True)
        if self.with_pub:
            pc_topic = rospy.get_param("~pc_topic")
            pc_frame = rospy.get_param("~pc_frame")
            self.pc_color = rospy.get_param("~pc_color", [1, 1, 1])

            self.pc_pub = rospy.Publisher(pc_topic, PointCloud2, queue_size=1, latch=True)
            self.pc_header = Header()
            self.pc_header.frame_id = pc_frame

            self.pcd = o3d.geometry.PointCloud()


    def init_server(self, srv_type):
        if isinstance(srv_type, type):
            srv_name = rospy.get_param("~srv_name")
            self.server = rospy.Service(srv_name, srv_type, self.callback)
            rospy.loginfo("[pcSrvBase] Server initialized")
        else:
            raise TypeError("[pcSrvBase] srv_type must be a service class type")


    def callback(self, req):
        raise NotImplementedError("[pcSrvBase] callback not implemented")


    def execute(self):
        if self.with_pub:
            while not rospy.is_shutdown():
                if self.pcd != None:
                    pc_msg = o3d2pc2(self.pcd, self.pc_header)
                    self.pc_pub.publish(pc_msg)
    
                self.rate.sleep()
        else:
            rospy.spin()