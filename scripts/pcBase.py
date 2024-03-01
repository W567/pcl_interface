#!/usr/bin/env python
import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2 as pc2


field = lambda name, id : PointField(name=name, offset=4*id, datatype=PointField.FLOAT32, count=1)
rgb_field = lambda id : PointField(name='rgb', offset=4*id, datatype=PointField.UINT32, count=1)

xyz_names = ['x', 'y', 'z']
nor_names = ['normal_x', 'normal_y', 'normal_z', 'curvature']
xyz_fields = [field(name, id) for id, name in enumerate(xyz_names)]
xyz_nor_fields = [field(name, id) for id, name in enumerate(xyz_names + nor_names)]
xyz_rgb_fields = [field(name, id) for id, name in enumerate(xyz_names)] + [rgb_field(3)]
xyz_nor_rgb_fields = [field(name, id) for id, name in enumerate(xyz_names + nor_names)] + [rgb_field(7)]


'''
Input : rgb  numpy array with shape (n, 3)        splitted into r, g, b
Output: comb_rgb numpy array with shape (n, 1)    combined together to form rgb
'''
def comb_rgb(rgb):
    rgb = (rgb * 255).astype(np.uint32)
    comb_rgb = (rgb[:, 0] << 16) + (rgb[:, 1] << 8) + rgb[:, 2]
    return comb_rgb


'''
Input : rgb  numpy array with shape (n, )          rgb combined together
Output: split_rgb numpy array with shape (n, 3)    splitted into r, g, b
'''
def split_rgb(rgb):
    rgb = np.nan_to_num(rgb)
    # type of rgb should be np.uint32 instead of np.float64
    if type(rgb[0][0][0]) == np.float64:
        rgb_tmp = np.array([[[struct.unpack('I', struct.pack('f', value[0]))[0]] for value in rgb[0]]])
        # can't guarantee the correctness of the type conversion
        # for example, pure blue with its value converted from float to uint32
        # is likely to be smaller than pow(2,24)
        if any(rgb_tmp.flatten() >= pow(2, 24)):
            rgb = rgb.astype(np.uint32)
        else:
            rgb = rgb_tmp
    else:
        rgb = rgb.astype(np.uint32)
    r = (rgb & 0x00FF0000) >> 16
    g = (rgb & 0x0000FF00) >> 8
    b = (rgb & 0x000000FF)
    r = r.astype(np.float32) / 255.0
    g = g.astype(np.float32) / 255.0
    b = b.astype(np.float32) / 255.0
    split_rgb = np.concatenate((r, g, b), axis=-1)
    return split_rgb


def o3d2pc2(pcd, header):
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


def pc22np(ros_cloud):
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
    
    return np_cloud_dict