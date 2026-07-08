#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker


class AgvDirectionMarker(object):
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.topic = rospy.get_param("~topic", "/agv/direction_marker")
        self.length = max(0.05, float(rospy.get_param("~length", 1.2)))
        self.shaft_diameter = max(0.01, float(rospy.get_param("~shaft_diameter", 0.12)))
        self.head_diameter = max(
            self.shaft_diameter, float(rospy.get_param("~head_diameter", 0.28))
        )
        self.height = float(rospy.get_param("~height", 0.35))
        self.label = rospy.get_param("~label", "FRENTE AGV")
        self.publish_rate = max(0.2, float(rospy.get_param("~publish_rate", 2.0)))

        self.red = self._color_param("red", 0.1)
        self.green = self._color_param("green", 1.0)
        self.blue = self._color_param("blue", 0.1)
        self.alpha = self._color_param("alpha", 1.0)

        self.publisher = rospy.Publisher(self.topic, Marker, queue_size=2, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_markers)
        rospy.loginfo(
            "AGV direction indicator publishing on %s in frame %s",
            self.topic,
            self.frame_id,
        )

    def _color_param(self, name, default):
        value = float(rospy.get_param("~" + name, default))
        return min(1.0, max(0.0, value))

    def _base_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "agv_direction"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.color.r = self.red
        marker.color.g = self.green
        marker.color.b = self.blue
        marker.color.a = self.alpha
        return marker

    def _publish_markers(self, _event):
        # RViz arrows point along local +X. Anchoring the marker to base_link
        # makes it follow the AGV pose and heading estimated by LeGO-LOAM.
        arrow = self._base_marker(0, Marker.ARROW)
        arrow.pose.position.z = self.height
        arrow.scale.x = self.length
        arrow.scale.y = self.shaft_diameter
        arrow.scale.z = self.head_diameter
        self.publisher.publish(arrow)

        if self.label:
            text = self._base_marker(1, Marker.TEXT_VIEW_FACING)
            text.pose.position.x = self.length * 0.5
            text.pose.position.z = self.height + self.head_diameter
            text.scale.z = max(0.15, self.head_diameter)
            text.text = self.label
            self.publisher.publish(text)


if __name__ == "__main__":
    rospy.init_node("agv_direction_marker")
    AgvDirectionMarker()
    rospy.spin()
