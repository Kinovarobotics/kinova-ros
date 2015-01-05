#!/usr/bin/env python

### A very basic admittance controller to demonstrate the Wrench output and
### cartesian velocity input of the Jaco/Mico node.
### See the associated launch file for running the complete demo.
###
### Defines a deadzone box for the applied forces.
### When the force exceed the deadzone bounds in a dimension, a cartesian
### velocity is applied as a factor (k_f) times the distance from the crossed
### bound (err).
### NOTE: Ignores torque for now.
### NOTE: The wrench output of the Jaco node represents the force that the robot
### applies, and not that it perceives, and is oriented in the base frame.
### For instance, this means that gravity is part of the wrench output as force
### in the positive part of the Z axis.
### This explains the higher upper bound of the deadzone in the Z axis, as this
### deadzone is applied on the raw input, and not the output.
###
### Topics:
###  - wrench:  Input force (geometry_msgs/StampedWrench).
###  - cmd_vel: Cartesian velocity output (geometry_msgs/TwistStamped).
###
### Parameters:
###  - dz_x_min: Lower bound of the deadzone in the X axis. Default: -10.0 N.
###  - dz_x_max: Upper bound of the deadzone in the X axis. Default:  10.0 N.
###  - dz_y_min: Lower bound of the deadzone in the Y axis. Default: -10.0 N.
###  - dz_y_max: Upper bound of the deadzone in the Y axis. Default:  10.0 N.
###  - dz_z_min: Lower bound of the deadzone in the Z axis. Default: -10.0 N.
###  - dz_z_max: Upper bound of the deadzone in the Z axis. Default:  30.0 N.
###  - k_f:      Response factor (k_f * err). Default: -0.1 m/(s * N).
###  - vel_max:  Maximum velocity in any dimension. Default: 0.1.
###  - period:   Update period, in s. Default: 0.10 s (10 Hz)

import roslib; roslib.load_manifest('jaco_demo')
import rospy

from geometry_msgs.msg import Wrench, WrenchStamped, Twist, TwistStamped

class AdmittanceCtrl:
    def __init__(self):
        self.cur_wrench = Wrench()

        self.dz_x_min = rospy.get_param("~dz_x_min",  -10)
        self.dz_x_max = rospy.get_param("~dz_x_max",   10)
        self.dz_y_min = rospy.get_param("~dz_y_min",  -10)
        self.dz_y_max = rospy.get_param("~dz_y_max",   10)
        self.dz_z_min = rospy.get_param("~dz_z_min",  -10)
        self.dz_z_max = rospy.get_param("~dz_z_max",   30)
        self.k_f      = rospy.get_param("~k_f",      -0.1)
        self.vel_max  = rospy.get_param("~vel_max",   0.1)
        period        = rospy.get_param("~period",    0.1)

        self.sub_wrench  = rospy.Subscriber("wrench", WrenchStamped, self.wrench_cb)
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", TwistStamped, queue_size=1)
        self.timer       = rospy.Timer(rospy.Duration(period), self.timer_cb)

    def wrench_cb(self, msg):
        # NOTE: This assumes that the input wrench is in the base frame of the
        # robot, or at least the same frame as the cartesian velocity input.
        self.cur_wrench = msg.wrench

    def timer_cb(self, event):
        enable = False

        twist = Twist()

        fx = self.cur_wrench.force.x
        fy = self.cur_wrench.force.y
        fz = self.cur_wrench.force.z

        if   (fx > self.dz_x_max):
            twist.linear.x = self.k_f * (fx - self.dz_x_max)
            enable = True
        elif (fx < self.dz_x_min):
            twist.linear.x = self.k_f * (fx - self.dz_x_min)
            enable = True
        if   (fy > self.dz_y_max):
            twist.linear.y = self.k_f * (fy - self.dz_y_max)
            enable = True
        elif (fy < self.dz_y_min):
            twist.linear.y = self.k_f * (fy - self.dz_y_min)
            enable = True
        if   (fz > self.dz_z_max):
            twist.linear.z = self.k_f * (fz - self.dz_z_max)
            enable = True
        elif (fz < self.dz_z_min):
            twist.linear.z = self.k_f * (fz - self.dz_z_min)
            enable = True

        if enable:
            twist.linear.x = self.saturate_vel(twist.linear.x)
            twist.linear.y = self.saturate_vel(twist.linear.y)
            twist.linear.z = self.saturate_vel(twist.linear.z)

            twist_s = TwistStamped()
            twist_s.header.stamp = rospy.get_rostime()
            twist_s.twist        = twist
            self.pub_cmd_vel.publish(twist_s)

    def saturate_vel(self, vel):
        avel = abs(vel)
        if avel > self.vel_max: 
            return vel / avel * self.vel_max
        else:
            return vel

if __name__ == "__main__":
    rospy.init_node("admittance_ctrl")
    node = AdmittanceCtrl()

    rospy.spin()

