#!/usr/bin/env python
import rospy
import open3d as o3d
from pcBase import *
from pcSrvBase import pcSrvBase
from pcl_interface.srv import curvFilter, curvFilterResponse
import curvature_computation as cc

class pcCurvServer(pcSrvBase):

    def __init__(self):
        super().__init__()
        self.init_server(curvFilter)
        self.response = curvFilterResponse()
        self.response.output_pcd = None


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
