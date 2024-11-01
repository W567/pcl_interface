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
        with_aff = rospy.get_param("with_aff", False)
        ext = rospy.get_param("aff_ext", "_aff0")
        while True:
            if rospy.has_param("free_body_names"):
                pc_filenames = rospy.get_param("free_body_names")
                self.pc_filenames = []
                self.pc_frames = []
                for pc_filename in pc_filenames:
                    self.pc_filenames.append(pc_filename)
                    self.pc_frames.append(pc_filename)
                    if with_aff:
                        self.pc_filenames.append(f"{pc_filename}{ext}")
                        self.pc_frames.append(pc_filename)

                self.pc_topics = self.pc_filenames.copy()
                package_path = os.path.join(rospack.get_path('obj_models'), 'obj')
                self.pc_file_paths = [f"{package_path}/{filename}.pcd" for filename in self.pc_filenames]
                break
            else:
                rospy.sleep(0.016)

def main():
    pub = objPcPublisher()
    pub.execute()

if __name__ == "__main__":
    main()