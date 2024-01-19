#!/usr/bin/env python
import struct
import numpy as np
from sensor_msgs.msg import PointField


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
    # rgb is wrongly interpreted as float32 in cpp, resolve it by reinterpreting it as uint32
    if type(rgb[0][0][0]) == np.float64:
        rgb = np.array([[[struct.unpack('I', struct.pack('f', value[0]))[0]] for value in rgb[0]]])
    else: # TODO maybe other types?
        rgb = rgb.astype(np.uint32)
    r = (rgb & 0x00FF0000) >> 16
    g = (rgb & 0x0000FF00) >> 8
    b = (rgb & 0x000000FF)
    r = r.astype(np.float32) / 255.0
    g = g.astype(np.float32) / 255.0
    b = b.astype(np.float32) / 255.0
    split_rgb = np.concatenate((r, g, b), axis=-1)
    return split_rgb

