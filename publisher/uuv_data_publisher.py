#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

pub_head = rospy.Publisher('/rexrov/pose_data/heading', Float64, queue_size=10)
pub_altitude = rospy.Publisher('/rexrov/pose_data/altitude', Float64, queue_size=10)
pub_depth = rospy.Publisher('/rexrov/pose_data/depth', Float64, queue_size=10)
pub_pitch = rospy.Publisher('/rexrov/pose_data/pitch', Float64, queue_size=10)
pub_roll = rospy.Publisher('/rexrov/pose_data/roll', Float64, queue_size=10)
pub_speed_forward = rospy.Publisher('/rexrov/pose_data/speed_forward', Float64, queue_size=10)
pub_speed_side = rospy.Publisher('/rexrov/pose_data/speed_side', Float64, queue_size=10)
pub_speed_up = rospy.Publisher('/rexrov/pose_data/speed_up', Float64, queue_size=10)
pub_accelerate_forward = rospy.Publisher('/rexrov/pose_data/accelerate_forward', Vector3, queue_size=10)
pub_accelerate_side = rospy.Publisher('/rexrov/pose_data/accelerate_side', Float64, queue_size=10)
pub_accelerate_up = rospy.Publisher('/rexrov/pose_data/accelerate_up', Float64, queue_size=10)

def publish_head(data):
    pub_head.publish(data)

def publish_altitude(data):
    pub_altitude.publish(data)

def publish_depth(data):
    pub_depth.publish(data)

def publish_pitch(data):
    pub_pitch.publish(data)

def publish_roll(data):
    pub_roll.publish(data)

def publish_speed_forward(data):
    pub_speed_forward.publish(data)

def publish_speed_side(data):
    pub_speed_side.publish(data)

def publish_speed_up(data):
    pub_speed_up.publish(data)

def publish_accelerate_forward(data):
    pub_accelerate_forward.publish(data)

def publish_accelerate_side(data):
    pub_accelerate_side.publish(data)

def publish_accelerate_up(data):
    pub_accelerate_up.publish(data)

def publisher():
    rospy.init_node('rexrov_position_publisher', anonymous=True)

    while not rospy.is_shutdown():
#        rospy.Subscriber("", Float64, publish_head)
#    	rospy.Subscriber("", Float64, publish_altitude)
#    	rospy.Subscriber("", Float64, publish_depth)
#    	rospy.Subscriber("", Float64, publish_pitch)
#    	rospy.Subscriber("", Float64, publish_roll)
#    	rospy.Subscriber("", Float64, publish_speed_forward)
#    	rospy.Subscriber("", Float64, publish_speed_side)
#    	rospy.Subscriber("", Float64, publish_speed_up)
    	rospy.Subscriber("/rexrov/imu/linear_acceleration", Vector3, publish_accelerate_forward)
#    	rospy.Subscriber("/rexrov/imu", Float64, publish_accelerate_side)
#    	rospy.Subscriber("/rexrov/imu", Float64, publish_accelerate_up)
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
