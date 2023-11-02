#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class PID:
    kp_ = 0.0
    ki_ = 0.0
    kd_ = 0.0
    max_effort_ = 0.0
    angular_ = False
    inverted_ = False
    timeout_ = 1

    setpoint_ = 0.0
    e_sum_ = 0.0
    e_prev_ = 0.0
    effort_ = 0.0

    def __init__(self, name, enabled=False):
        self.ns_ = name
        self.enabled_ = enabled
        self.last_update_time_ = rospy.Time.now()

        if not rospy.has_param(self.ns_):
            rospy.logwarn("Didn't find %s pid configuration. Assign zeros." % self.ns_)

        self.ddynrec = DDynamicReconfigure(self.ns_)
        self.ddynrec.add_variable('kp', 'proportional', 0.0, 0.0, 100.0)
        self.ddynrec.add_variable('ki', 'integral', 0.0, 0.0, 100.0)
        self.ddynrec.add_variable('kd', 'differential', 0.0, 0.0, 100.0)
        self.ddynrec.add_variable('max_effort', 'max effort', 0.0, 0.0, 1.0)
        self.ddynrec.add_variable('angular', 'angular', False)
        self.ddynrec.add_variable('inverted', 'inverted', False)
        self.ddynrec.add_variable('timeout', 'state timeout', 1)
        self.ddynrec.start(self.ddynamic_reconfigure_callback_)

        self.switch_service = rospy.Service('~%s_pid/enable' % self.ns_, SetBool,
                                            self.handle_switch_service_)

        rospy.Subscriber('%s_pid/setpoint' % self.ns_, Float64, self.setpoint_callback_)

    def ddynamic_reconfigure_callback_(self, config, level):
        self.kp_ = config['kp']
        self.ki_ = config['ki']
        self.kd_ = config['kd']
        self.max_effort_ = config['max_effort']
        self.angular_ = config['angular']
        self.inverted_ = config['inverted']
        self.timeout_ = config['timeout']
        self.reset_()
        return config

    def handle_switch_service_(self, req):
        self.switch_(req.data)
        resp = SetBoolResponse()
        resp.success = True
        resp.message = '%s pid %s' % (self.ns_, ('disabled', 'enabled')[self.enabled_])
        return resp

    def setpoint_callback_(self, msg):
        self.setpoint_ = msg.data

    def switch_(self, state):
        self.enabled_ = state
        self.reset_()

    def reset_(self):
        self.e_sum_ = 0.0
        self.e_prev_ = 0.0
        self.effort_ = 0.0

    def angular_constraint_(self, e):
        return (e + 180) % (2 * 180) - 180

    def clamp_(self, v):
        return min(max(v, -self.max_effort_), self.max_effort_)

    def update(self, state):
        if not self.enabled_:
            return

        self.last_update_time_ = rospy.Time.now()

        error = self.setpoint_ - state if not self.inverted_ else state - self.setpoint_
        if self.angular_:
            error = self.angular_constraint_(error)
        p = self.kp_ * error
        self.e_sum_ = self.clamp_(self.e_sum_ + self.ki_ * error)
        d = self.kd_ * (error - self.e_prev_)
        self.e_prev_ = error
        self.effort_ = self.clamp_(p + self.e_sum_ + d)

    def get_effort(self):
        if (rospy.Time.now() - self.last_update_time_).secs > self.timeout_:
            self.reset_()
        return self.effort_

