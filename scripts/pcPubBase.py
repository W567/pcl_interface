#!/usr/bin/env python
import tf
import rospy
import numpy as np
import open3d as o3d
from std_msgs.msg import Header
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

import rospkg
rospack = rospkg.RosPack()

field = lambda name, id : PointField(name=name, offset=4*id, datatype=PointField.FLOAT32, count=1)
rgb_field = lambda id : PointField(name='rgb', offset=4*id, datatype=PointField.UINT32, count=1)

class pcPubBase():

    xyz_names = ['x', 'y', 'z']
    nor_names = ['normal_x', 'normal_y', 'normal_z', 'curvature']
    xyz_fields = [field(name, id) for id, name in enumerate(xyz_names)]
    xyz_nor_fields = [field(name, id) for id, name in enumerate(xyz_names + nor_names)]
    xyz_rgb_fields = [field(name, id) for id, name in enumerate(xyz_names)] + [rgb_field(3)]
    xyz_nor_rgb_fields = [field(name, id) for id, name in enumerate(xyz_names + nor_names)] + [rgb_field(7)]

    def __init__(self):
        rospy.init_node('pcPubBase', anonymous=True)

        rate = rospy.get_param("~rate", 10)
        self.rate = rospy.Rate(rate)

        if rospy.has_param("~pkg_name"):
            pkg_name = rospy.get_param("~pkg_name")
            pkg_path = rospack.get_path(pkg_name)
        else:
            pkg_path = ""

        pc_filenames = rospy.get_param("~pc_filenames", ["/obj/ycb_010_potted_meat_can.pcd",])
        pc_topics = rospy.get_param("~pc_topics", ["/hoge",])
        pc_frames = rospy.get_param("~pc_frames", ["object",])
        pc_colors = rospy.get_param("~pc_colors", [[1.0, 0.0, 0.0],])

        assert len(pc_filenames) == len(pc_topics) == len(pc_frames) == len(pc_colors), \
            "[pcPubBase]length imcompatible"

        self.pcd_list = []
        self.pc_pub_list = []
        self.pc_head_list = []
        for i, pc_topic in enumerate(pc_topics):
            pcd = o3d.io.read_point_cloud(pkg_path + pc_filenames[i])
            if min(pc_colors[i]) >= 0 and max(pc_colors[i]) <= 1:
                pcd.paint_uniform_color(pc_colors[i])
            self.pcd_list.append(pcd)
            pc_pub = rospy.Publisher(pc_topic, PointCloud2, queue_size=1, latch=True)
            self.pc_pub_list.append(pc_pub)
            pc_header = Header()
            pc_header.frame_id = pc_frames[i]
            self.pc_head_list.append(pc_header)
        
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("[pcPubBase] Initialized")


    def comb_rgb(self, rgb):
        rgb = (rgb * 255).astype(np.uint32)
        comb_rgb = (rgb[:, 0] << 16) + (rgb[:, 1] << 8) + rgb[:, 2]
        return comb_rgb


    def split_rgb(self, rgb):
        rgb = np.nan_to_num(rgb).astype(int)
        r = (rgb & 0x00FF0000) >> 16
        g = (rgb & 0x0000FF00) >> 8
        b = (rgb & 0x000000FF)
        split_rgb = np.concatenate((r, g, b), axis=-1)
        return split_rgb


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
            split_rgb = self.split_rgb(rgb)
            np_cloud_dict['rgb'] = split_rgb
        else:
            rospy.logwarn_once('[pcPubBase] Received cloud without point color')

        if 'normal_x' in field_names:
            rospy.logwarn_once('[pcPubBase] Received cloud with point normal')
            idx_nx = np.where(field_names == 'normal_x')[0]
            idx_ny = np.where(field_names == 'normal_y')[0]
            idx_nz = np.where(field_names == 'normal_z')[0]
            idx_nor = np.concatenate((idx_nx, idx_ny, idx_nz), axis=-1)
            nor = np_cloud[:, :, idx_nor]
        else:
            rospy.logwarn_once('[pcPubBase] Received cloud without point normal, estimating')

            # TODO normal estimation necessary?
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.obj_frame, self.cam_frame, rospy.Time(0))
                rospy.logwarn_once("[pcPubBase] camera @ [%.3f, %.3f, %.3f]", trans[0], trans[1], trans[2])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr_once("[pcPubBase] Can't get tf information, using [0, 0, 0]")
                trans = np.zeros((3))

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz[0])
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
            pcd.orient_normals_towards_camera_location(camera_location=trans)
            nor = np.asarray([pcd.normals])
        np_cloud_dict['nor'] = nor
        
        return np_cloud_dict


    def o3d2pc2(self, pcd, header):
        positions = np.asarray(pcd.points)
        if pcd.has_colors():
            colors = self.comb_rgb(np.asarray(pcd.colors))
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            curvatures = np.zeros_like(normals[:, 0])

        if pcd.has_colors() and pcd.has_normals():
            fields = self.xyz_nor_rgb_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2],
                                                   normals[:, 0], normals[:, 1], normals[:, 2], curvatures,
                                                   colors], names='x,y,z,normal_x,normal_y,normal_z,curvature, rgb')
        elif pcd.has_colors() and not pcd.has_normals():
            fields = self.xyz_rgb_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2],
                                                   colors], names='x,y,z,rgb')
        elif not pcd.has_colors() and pcd.has_normals():
            fields = self.xyz_nor_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2],
                                                   normals[:, 0], normals[:, 1], normals[:, 2], curvatures],
                                                   names='x,y,z,normal_x,normal_y,normal_z, curvature')
        else:
            fields = self.xyz_fields
            np_cloud = np.core.records.fromarrays([positions[:, 0], positions[:, 1], positions[:, 2]], 
                                                   names='x,y,z')
        
        header.stamp = rospy.Time.now()
        return pc2.create_cloud(header, fields, np_cloud.flatten())


    def pcd_trans(self, pcd):
        return pcd


    def execute(self):
        while not rospy.is_shutdown():
            for pcd, head, pub in zip(self.pcd_list, self.pc_head_list, self.pc_pub_list):
                pcd = self.pcd_trans(pcd)
                pc_msg = self.o3d2pc2(pcd, head)
                pub.publish(pc_msg)

            self.rate.sleep()