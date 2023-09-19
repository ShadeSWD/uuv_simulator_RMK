#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
    pub_head = rospy.Publisher('/rexrov/pose_data/heading', String, queue_size=10)
    pub_altitude = rospy.Publisher('/rexrov/pose_data/altitude', String, queue_size=10)
    pub_depth = rospy.Publisher('/rexrov/pose_data/depth', String, queue_size=10)
    pub_pitch = rospy.Publisher('/rexrov/pose_data/pitch', String, queue_size=10)
    pub_roll = rospy.Publisher('/rexrov/pose_data/roll', String, queue_size=10)
    pub_speed_forward = rospy.Publisher('/rexrov/pose_data/speed_forward', String, queue_size=10)
    pub_speed_side = rospy.Publisher('/rexrov/pose_data/speed_side', String, queue_size=10)
    pub_speed_up = rospy.Publisher('/rexrov/pose_data/speed_up', String, queue_size=10)
    pub_accelerate_forward = rospy.Publisher('/rexrov/pose_data/accelerate_forward', String, queue_size=10)
    pub_accelerate_side = rospy.Publisher('/rexrov/pose_data/accelerate_side', String, queue_size=10)
    pub_accelerate_up = rospy.Publisher('/rexrov/pose_data/accelerate_up', String, queue_size=10)

    rospy.init_node('rexrov_position_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub_head.publish('head')
    	pub_altitude.publish('alt')
    	pub_depth.publish('depth') 
    	pub_pitch.publish('pitch')
	pub_roll.publish('roll')
    	pub_speed_forward.publish('fwd')
    	pub_speed_side.publish('side')
    	pub_speed_up.publish('up')
    	pub_accelerate_forward.publish('afwd')
    	pub_accelerate_side.publish('asd')
    	pub_accelerate_up.publish('aup')
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
