#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class RtabmapToMavros:
    def __init__(self):
        self.rtabmap_pose_topic = rospy.get_param("~rtabmap_pose_topic", "/rtabmap/localization_pose")
        self.fallback_odom_topic = rospy.get_param("~fallback_odom_topic", "/rtabmap/odom")
        self.mavros_pose_topic = rospy.get_param("~mavros_pose_topic", "/mavros/vision_pose/pose")
        self.rate_limit_hz = float(rospy.get_param("~rate_limit_hz", 30.0))

        self.pub = rospy.Publisher(self.mavros_pose_topic, PoseStamped, queue_size=10)
        self.last_pub_time = rospy.Time(0)
        self.pub_period = rospy.Duration.from_sec(1.0 / max(1e-3, self.rate_limit_hz))

        self.pose_sub = rospy.Subscriber(self.rtabmap_pose_topic, PoseStamped, self.pose_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.fallback_odom_topic, Odometry, self.odom_cb, queue_size=10)

        rospy.loginfo("[rtabmap_to_mavros] forwarding %s or %s --> %s",
                      self.rtabmap_pose_topic, self.fallback_odom_topic, self.mavros_pose_topic)

    def maybe_publish(self, msg):
        now = rospy.Time.now()
        if now - self.last_pub_time < self.pub_period:
            return
        self.last_pub_time = now
        self.pub.publish(msg)

    def pose_cb(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.maybe_publish(out)

    def odom_cb(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self.maybe_publish(out)

if __name__ == "__main__":
    rospy.init_node("rtabmap_to_mavros_pose")
    RtabmapToMavros()
    rospy.spin()
