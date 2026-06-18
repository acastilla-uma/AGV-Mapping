#!/usr/bin/env python
from __future__ import print_function

import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def main():
    if len(sys.argv) < 9:
        print(
            "Usage: static_transform_rpy.py x y z roll pitch yaw parent_frame child_frame",
            file=sys.stderr,
        )
        return 1

    x, y, z, roll, pitch, yaw = [float(value) for value in sys.argv[1:7]]
    parent_frame = sys.argv[7].lstrip("/")
    child_frame = sys.argv[8].lstrip("/")

    rospy.init_node("static_transform_rpy")

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z

    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
