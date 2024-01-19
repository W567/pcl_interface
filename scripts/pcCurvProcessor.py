#!/usr/bin/env python
import rospy
import numpy as np
from pcPubBase import pcPubBase
import curvature_computation as cc

class pcCurvProcessor(pcPubBase):

    def __init__(self):
        super().__init__()
        self.curv_threshold = rospy.get_param("~curv_threshold", 0.6)

    def pcd_process(self, pcd):
        pos = np.asarray(pcd.points)
        nor = np.asarray(pcd.normals)
        idxs = cc.curvFilter(pos, nor, self.curv_threshold)
        pcd = pcd.select_by_index(idxs)
        return pcd

def main():
    pub = pcCurvProcessor()
    pub.execute()

if __name__ == "__main__":
    main()