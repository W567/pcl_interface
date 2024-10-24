#!/usr/bin/env python
import os
import rospy
from pcPubBase import pcPubBase

import rospkg
rospack = rospkg.RosPack()

class objPcPublisher(pcPubBase):
    def __init__(self):
        super().__init__()

    def init_file_topic_frame(self):
        while True:
            if rospy.has_param("free_body_names"):
                self.pc_filenames = rospy.get_param("free_body_names")
                self.pc_file_paths = self.pc_filenames.copy()
                package_path = os.path.join(rospack.get_path('obj_models'), 'obj')
                self.pc_file_paths = [f"{package_path}/{filename}.pcd" for filename in self.pc_file_paths]
                break
            else:
                rospy.sleep(0.016)
    
        self.pc_topics = self.pc_filenames.copy()
        self.pc_frames = self.pc_filenames.copy()

def main():
    pub = objPcPublisher()
    pub.execute()

if __name__ == "__main__":
    main()