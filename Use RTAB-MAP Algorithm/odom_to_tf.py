#!/usr/bin/env python
import rospy, tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def cb(msg):
    t = TransformStamped()
    t.header = msg.header
    t.child_frame_id = msg.child_frame_id or rospy.get_param("~child_frame_id", "base_link_frd")
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation      = msg.pose.pose.orientation
    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    br = tf2_ros.TransformBroadcaster()
    topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
    rospy.Subscriber(topic, Odometry, cb, queue_size=50)
    rospy.spin()



