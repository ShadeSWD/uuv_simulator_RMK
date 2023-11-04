#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64, Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

from joy_models import Thrustmaster


class Teleop():
    surge_scale = 1.0
    sway_scale = 0.2
    heave_scale = 1.0
    yaw_scale = 0.1
    pitch_sp_scale = 0.4
 
    pitch_shift = 5.5
    roll_shift = 2.0

    joy_ = Thrustmaster()

    enabled_ = True
    depth_pid_enabled_ = False
    heading_pid_enabled_ = False
    pitch_pid_enabled_ = False
    free_mode_enabled_ = False
    depth_ = 0.0
    heading_ = 0.0
    pitch_sp_ = 0.0

    def __init__(self):

        self.pitch_sp_pressed_ = rospy.Time.now()

        self.start_pressed_ = rospy.Time.now()
        self.button_pressed_ = rospy.Time.now()

        self.twist_publisher_ = rospy.Publisher('teleop_command', Twist, queue_size=1)
        self.roll_sp_publisher_ = rospy.Publisher('/pid_regulator/roll_pid/setpoint', Float64, queue_size=1)
        self.pitch_sp_publisher_ = rospy.Publisher('/pid_regulator/pitch_pid/setpoint', Float64, queue_size=1)
        self.depth_sp_publisher_ = rospy.Publisher('/pid_regulator/depth_pid/setpoint', Float64, queue_size=1)
        self.heading_sp_publisher_ = rospy.Publisher('/pid_regulator/heading_pid/setpoint', Float64, queue_size=1)

        self.pid_service_ = self.init_service_client('pid_regulator/switch', SetBool)
        self.depth_pid_service_ = self.init_service_client('depth_pid/enable', SetBool)
        self.heading_pid_service_ = self.init_service_client('heading_pid/enable', SetBool)
        self.pitch_pid_service_ = self.init_service_client('heading_pid/enable', SetBool)

        self.update_timer_ = rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.Subscriber('depth', Float64, self.depth_callback)
        rospy.Subscriber('heading', Float64, self.heading_callback)

        rospy.loginfo('Teleoperation started')

    def __del__(self):
        self.reset()

    def init_service_client(self, serivce_name, service_type):
        rospy.loginfo('Waiting for service: %s' % serivce_name)
        rospy.wait_for_service(serivce_name, timeout=3)
        return rospy.ServiceProxy(serivce_name, service_type)

    def switch_pid(self, state):
        try:
            resp = self.pid_service_(state)
            if resp.success:
                rospy.logwarn(resp.message)
                self.enabled_ = state
        except rospy.ServiceException as e:
            rospy.logerr('Service did not process request: %s' % e)

    def switch_depth_pid(self, en):
        if not self.enabled_:
            return

        self.depth_sp_publisher_.publish(self.depth_)
        if self.depth_pid_enabled_ == en:
            return

        try:
            resp = self.depth_pid_service_(en)
            rospy.logwarn(resp.message)
            self.depth_pid_enabled_ = en
        except rospy.ServiceException as e:
            rospy.logerr('Service did not process request: %s' % e)

    def switch_heading_pid(self, en):
        if not self.enabled_:
            return

        self.heading_sp_publisher_.publish(self.heading_)
        if self.heading_pid_enabled_ == en:
            return

        try:
            resp = self.heading_pid_service_(en)
            rospy.logwarn(resp.message)
            self.heading_pid_enabled_ = en
        except rospy.ServiceException as e:
            rospy.logerr('Service did not process request: %s' % e)

    def switch_pitch_pid(self, en):
        if not self.enabled_:
            return

        self.pitch_sp_publisher_.publish(self.pitch_shift)
        if self.pitch_pid_enabled_ == en:
            return

        try:
            resp = self.pitch_pid_service_(en)
            rospy.logwarn(resp.message)
            self.pitch_pid_enabled_ = en
        except rospy.ServiceException as e:
            rospy.logerr('Service did not process request: %s' % e)

    def switch_free_mode(self):
        if not self.enabled_:
            return

        self.free_mode_enabled_ = not self.free_mode_enabled_
        if self.free_mode_enabled_:
            self.switch_depth_pid(False)
            self.switch_pid(False)
            rospy.logwarn('Enable free mode')
        else:
            self.switch_pid(True)
            rospy.logwarn('Disable free mode')

        try:
            self.pitch_sp_publisher_.publish(self.pitch_sp_)
            resp = self.pitch_pid_service_(not self.free_mode_enabled_)
            rospy.logwarn(resp.message)
        except rospy.ServiceException as e:
            rospy.logerr('Service did not process request: %s' % e)

    def hold(self):
        rospy.logwarn('Enable hold')
        self.switch_pid(True)
        self.switch_depth_pid(True)
        self.switch_heading_pid(True)

    def reset(self):
        self.roll_sp_publisher_.publish(0)
        self.pitch_sp_publisher_.publish(0)
        self.depth_sp_publisher_.publish(0.0)
        self.twist_publisher_.publish(Twist())

    def update(self, _):
        if self.joy_.buttons['START'].clicked():
            self.switch_pid(not self.enabled_)
            self.reset()

        if self.joy_.buttons['TOP_M'].clicked():
            self.hold()
        if self.joy_.buttons['TOP_L'].clicked():
           self.switch_free_mode()

        if self.enabled_:
            self.update_twist()
            self.update_pitch_setpoint()

    def update_twist(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.joy_.axes['BIG_STICK_UD'].state() * self.surge_scale
        if self.free_mode_enabled_:
            twist_msg.linear.z = self.joy_.axes['SPEED'].state() * 0.5
            twist_msg.angular.x = self.joy_.axes['SMALL_STICK_LR'].state() * self.sway_scale
            twist_msg.angular.y = self.joy_.axes['SMALL_STICK_UD'].state() * 0.5
            twist_msg.angular.z = self.joy_.axes['BIG_STICK_ROT'].state() * self.yaw_scale
        else:
            twist_msg.linear.y = self.joy_.axes['BIG_STICK_LR'].state() * self.sway_scale
            twist_msg.angular.z = self.joy_.axes['BIG_STICK_ROT'].state() * self.yaw_scale

        if self.joy_.buttons['LU1'].pressed():
            twist_msg.linear.z = self.joy_.axes['SPEED'].state() * 0.5
            self.switch_depth_pid(False)
            self.switch_heading_pid(False)

        self.twist_publisher_.publish(twist_msg)

    def update_pitch_setpoint(self):
        if not self.free_mode_enabled_:
            self.pitch_sp_ = self.pitch_shift + self.joy_.axes['SMALL_STICK_UD'].state() * self.pitch_sp_scale
            self.pitch_sp_publisher_.publish(self.pitch_sp_)

    def joy_callback(self, msg):
        self.joy_.update(msg.axes, msg.buttons)

    def depth_callback(self, msg):
        self.depth_ = msg.data

    def heading_callback(self, msg):
        self.heading_ = msg.data


if __name__ == '__main__':
    rospy.init_node('joy_teleop')
    teleop = Teleop()
    rospy.spin()

