#!/usr/bin/env python
import tf
import rospy
import struct
import numpy as np
import open3d as o3d
from pcBase import *
from std_msgs.msg import Header
from sensor_msgs import point_cloud2 as pc2
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
        np_cloud_dict = self.pc22np(msg)
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


    def pc22np(self, ros_cloud):
        field_names=np.array([field.name for field in ros_cloud.fields])
        np_cloud = np.array([list(pc2.read_points(ros_cloud, skip_nans=False, field_names=field_names))])
        if np_cloud.size == 0:
            rospy.logerr("[pcPubBase] Empty cloud")
            return None

        np_cloud_dict = {}

        if 'x' in field_names:
            idx_x = np.where(field_names == 'x')[0]
            idx_y = np.where(field_names == 'y')[0]
            idx_z = np.where(field_names == 'z')[0]
            idx_xyz = np.concatenate((idx_x, idx_y, idx_z), axis=-1)
            xyz = np_cloud[:, :, idx_xyz]
            np_cloud_dict['xyz'] = xyz
        else:
            rospy.logerr_once('[pcPubBase] Received cloud without point position')

        if 'rgb' in field_names:
            idx_rgb = np.where(field_names == 'rgb')[0]
            rgb = np_cloud[:, :, idx_rgb]
            np_cloud_dict['rgb'] = split_rgb(rgb)
        else:
            rospy.logwarn_once('[pcPubBase] Received cloud without point color')

        if 'normal_x' in field_names:
            rospy.logwarn_once('[pcPubBase] Received cloud with point normal')
            idx_nx = np.where(field_names == 'normal_x')[0]
            idx_ny = np.where(field_names == 'normal_y')[0]
            idx_nz = np.where(field_names == 'normal_z')[0]
            idx_nor = np.concatenate((idx_nx, idx_ny, idx_nz), axis=-1)
            nor = np_cloud[:, :, idx_nor]
            np_cloud_dict['nor'] = nor
        else:
            rospy.logwarn_once('[pcPubBase] Received cloud without point normal')
            # rospy.logwarn_once('[pcPubBase] Received cloud without point normal, estimating')

            # # TODO normal estimation necessary?
            # try:
            #     (trans,rot) = self.tf_listener.lookupTransform(self.obj_frame, self.cam_frame, rospy.Time(0))
            #     rospy.logwarn_once("[pcPubBase] camera @ [%.3f, %.3f, %.3f]", trans[0], trans[1], trans[2])
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     rospy.logerr_once("[pcPubBase] Can't get tf information, using [0, 0, 0]")
            #     trans = np.zeros((3))

            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(xyz[0])
            # pcd.estimate_normals(
            #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
            # pcd.orient_normals_towards_camera_location(camera_location=trans)
            # nor = np.asarray([pcd.normals])
            # np_cloud_dict['nor'] = nor
        
        return np_cloud_dict


    def o3d2pc2(self, pcd, header):
        positions = np.asarray(pcd.points)
        if pcd.has_colors():
            colors = comb_rgb(np.asarray(pcd.colors))
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            curvatures = np.zeros_like(normals[:, 0])

        if pcd.has_colors() and pcd.has_normals():
            fields = xyz_nor_rgb_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2],
                                                   normals[:, 0], normals[:, 1], normals[:, 2], curvatures,
                                                   colors], names='x,y,z,normal_x,normal_y,normal_z,curvature, rgb')
        elif pcd.has_colors() and not pcd.has_normals():
            fields = xyz_rgb_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2],
                                                   colors], names='x,y,z,rgb')
        elif not pcd.has_colors() and pcd.has_normals():
            fields = xyz_nor_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2],
                                                   normals[:, 0], normals[:, 1], normals[:, 2], curvatures],
                                                   names='x,y,z,normal_x,normal_y,normal_z, curvature')
        else:
            fields = xyz_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2]], 
                                                   names='x,y,z')
        
        header.stamp = rospy.Time.now()
        return pc2.create_cloud(header, fields, np_cloud.flatten())


    def pcd_trans(self, pcd):
        return pcd


    def execute(self):
        self.init_pc()
        while not rospy.is_shutdown():
            for pcd, head, pub in zip(self.pcd_list, self.pc_head_list, self.pc_pub_list):
                if pcd == None:
                    continue
                pcd = self.pcd_trans(pcd)
                pc_msg = self.o3d2pc2(pcd, head)
                pub.publish(pc_msg)

            self.rate.sleep()