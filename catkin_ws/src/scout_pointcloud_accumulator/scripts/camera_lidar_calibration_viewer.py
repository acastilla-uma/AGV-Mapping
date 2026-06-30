#!/usr/bin/env python
from __future__ import print_function

import sys

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class CalibrationViewer(object):
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'base_link').lstrip('/')
        self.lidar_topic = rospy.get_param('~lidar_topic', '/velodyne_points')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/depth/color/points')
        self.transform_timeout = float(rospy.get_param('~transform_timeout', 0.2))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lidar_pub = rospy.Publisher('/calibration/lidar_points', PointCloud2, queue_size=1)
        self.camera_pub = rospy.Publisher('/calibration/camera_points', PointCloud2, queue_size=1)
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.lidar_cb, queue_size=1)
        self.camera_sub = rospy.Subscriber(self.camera_topic, PointCloud2, self.camera_cb, queue_size=1)
        rospy.loginfo('Camera/LiDAR calibration viewer target=%s lidar=%s camera=%s', self.target_frame, self.lidar_topic, self.camera_topic)

    def transform_and_publish(self, msg, pub, label):
        source = msg.header.frame_id.lstrip('/')
        if source == self.target_frame:
            out = PointCloud2()
            out = msg
            out.header.frame_id = self.target_frame
            pub.publish(out)
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source,
                msg.header.stamp,
                rospy.Duration(self.transform_timeout),
            )
            out = do_transform_cloud(msg, transform)
            out.header.frame_id = self.target_frame
            pub.publish(out)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, '%s cloud waiting for TF %s -> %s: %s', label, self.target_frame, source, exc)

    def lidar_cb(self, msg):
        self.transform_and_publish(msg, self.lidar_pub, 'LiDAR')

    def camera_cb(self, msg):
        self.transform_and_publish(msg, self.camera_pub, 'Camera')


def main():
    rospy.init_node('camera_lidar_calibration_viewer')
    CalibrationViewer()
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
