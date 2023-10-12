#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from utils import *
from math import degrees

pub_heading = rospy.Publisher('/rexrov/pose_data/heading', Float64, queue_size=10)
pub_altitude = rospy.Publisher('/rexrov/pose_data/altitude', Float64, queue_size=10)
pub_depth = rospy.Publisher('/rexrov/pose_data/depth', Float64, queue_size=10)
pub_pitch = rospy.Publisher('/rexrov/pose_data/pitch', Float64, queue_size=10)
pub_roll = rospy.Publisher('/rexrov/pose_data/roll', Float64, queue_size=10)
pub_speed_forward = rospy.Publisher('/rexrov/pose_data/speed_forward', Float64, queue_size=10)
pub_speed_side = rospy.Publisher('/rexrov/pose_data/speed_side', Float64, queue_size=10)
pub_speed_up = rospy.Publisher('/rexrov/pose_data/speed_up', Float64, queue_size=10)
pub_accelerate_forward = rospy.Publisher('/rexrov/pose_data/accelerate_forward',Float64, queue_size=10)
pub_accelerate_side = rospy.Publisher('/rexrov/pose_data/accelerate_side', Float64, queue_size=10)
pub_accelerate_up = rospy.Publisher('/rexrov/pose_data/accelerate_up', Float64, queue_size=10)

def publish_altitude(data):
    pub_altitude.publish(data)

def publish_depth(data):
    middle_range = len(data.ranges) // 2
    depth = data.ranges[middle_range]
    pub_depth.publish(depth)

def publish_speed_and_orientation(data):
    xq = data.pose.pose.orientation.x
    yq = data.pose.pose.orientation.y
    zq = data.pose.pose.orientation.z
    wq = data.pose.pose.orientation.w
    roll, pitch, yaw = euler_from_quaternion(x=xq, y=yq, z=zq, w=wq)
    deg_roll = degrees(roll)
    deg_pitch = degrees(pitch)
    deg_heading = degrees(yaw)
    pub_heading.publish(deg_heading)
    pub_pitch.publish(deg_pitch)
    pub_roll.publish(deg_roll)

    pub_speed_forward.publish(data.twist.twist.linear.x)
    pub_speed_side.publish(data.twist.twist.linear.y)
    pub_speed_up.publish(data.twist.twist.linear.z)

def publish_accelerate(data):
    pub_accelerate_forward.publish(data.linear_acceleration.x)
    pub_accelerate_side.publish(data.linear_acceleration.y)
    pub_accelerate_up.publish(data.linear_acceleration.z)

def publisher():
    rospy.init_node('rexrov_position_publisher', anonymous=True)

    while not rospy.is_shutdown():
#    	rospy.Subscriber("", Float64, publish_altitude)
    	rospy.Subscriber("/rexrov/sss_down", LaserScan, publish_depth)
    	rospy.Subscriber("/rexrov/pose_gt", Odometry, publish_speed_and_orientation)
    	rospy.Subscriber("/rexrov/imu", Imu, publish_accelerate)
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
