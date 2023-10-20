#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


rospy.init_node('republisher')
in_topic = rospy.get_param('~in')
out_topic = rospy.get_param('~out')

pub = rospy.Publisher(out_topic, FloatStamped, queue_size=1)

def msg_callback(msg):
    uuv_msg = FloatStamped()
    uuv_msg.data = msg.data
    pub.publish(uuv_msg)

rospy.Subscriber(in_topic, Float64, msg_callback)
rospy.spin()
