#!/usr/bin/env python
import rospy
import open3d as o3d
from pcBase import *
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from pcl_interface.srv import curvFilter, curvFilterResponse
import curvature_computation as cc

class pcCurvServer():

    def __init__(self):
        rospy.init_node('pcCurvServer', anonymous=True)
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

        self.init_server(curvFilter)
        self.response = curvFilterResponse()
        self.response.output_pcd = None


    def init_server(self, srv_type):
        if isinstance(srv_type, type):
            srv_name = rospy.get_param("~srv_name")
            self.server = rospy.Service(srv_name, srv_type, self.callback)
            rospy.loginfo("[pcCurvServer] Server initialized")
        else:
            raise TypeError("[pcCurvServer] srv_type must be a service class type")


    def callback(self, req):
        np_cloud_dict = pc22np(req.input_pcd)
        if np_cloud_dict == None:
            rospy.logerr("[pcCurvServer] Invalid PointCloud2 message")
            return self.response
        pos = np_cloud_dict["xyz"][0]
        nor = np_cloud_dict["nor"][0]
        idxs = cc.curvFilter(pos, nor, req.threshold)
        self.pcd.points = o3d.utility.Vector3dVector(pos[idxs])
        self.pcd.normals = o3d.utility.Vector3dVector(nor[idxs])
        self.pcd.paint_uniform_color([0, 0, 1])
        self.response.output_pcd = o3d2pc2(self.pcd, self.pc_header)
        return self.response


    def execute(self):
        if self.with_pub:
            while not rospy.is_shutdown():
                if self.response.output_pcd != None:
                    self.response.output_pcd.header.stamp = rospy.Time.now()
                    self.pc_pub.publish(self.response.output_pcd)
                self.rate.sleep()
        else:
            rospy.spin()


def main():
    handle = pcCurvServer()
    handle.execute()

if __name__ == "__main__":
    main()
