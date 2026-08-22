"""Quaternion/Euler helpers with the tf_transformations API.

tf_transformations only ships as an apt package (ros-humble-tf-transformations)
and is not importable from the conda env, so these are implemented directly on
transforms3d (declared in environment.yml). Same conventions as tf: quaternions
are ordered (x, y, z, w) and Euler angles default to 'sxyz' (roll, pitch, yaw),
while transforms3d orders quaternions (w, x, y, z).
"""
from transforms3d.euler import euler2quat, quat2euler


def euler_from_quaternion(quaternion, axes='sxyz'):
    x, y, z, w = quaternion
    return quat2euler((w, x, y, z), axes)


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    w, x, y, z = euler2quat(ai, aj, ak, axes)
    return (x, y, z, w)
