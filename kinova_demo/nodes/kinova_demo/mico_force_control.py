#!/usr/bin/env python
#
# A basic node to start/stop force control on Kinova arms with force sensors
# such as Mico.
# See 'robot' parameter to change the default driver namespace.

import roslib; roslib.load_manifest('kinova_demo')
import rospy

from geometry_msgs.msg import Vector3
from jaco_msgs.srv     import SetForceControlParams, SetForceControlParamsRequest, Start, Stop

rospy.init_node("force_control")

robot     = rospy.get_param("robot", "mico")

inertia_lin   = rospy.get_param("inertia_lin",    10.0)
inertia_ang   = rospy.get_param("inertia_ang",     3.0)
damping_lin   = rospy.get_param("damping_lin",    80.0)
damping_ang   = rospy.get_param("damping_ang",    10.0)
force_min_lin = rospy.get_param("force_min_lin",   0.1)
force_min_ang = rospy.get_param("force_min_ang",   0.1)
force_max_lin = rospy.get_param("force_max_lin",  40.0)
force_max_ang = rospy.get_param("force_max_ang",  40.0)

srv_start = rospy.ServiceProxy(robot + "_arm_driver/in/start_force_control",      Start)
srv_stop  = rospy.ServiceProxy(robot + "_arm_driver/in/stop_force_control",       Stop)
srv_set   = rospy.ServiceProxy(robot + "_arm_driver/in/set_force_control_params", SetForceControlParams)

srv_start.call()

set_req = SetForceControlParamsRequest()
set_req.inertia_linear.x    = inertia_lin
set_req.inertia_linear.y    = inertia_lin
set_req.inertia_linear.z    = inertia_lin
set_req.inertia_angular.x   = inertia_ang
set_req.inertia_angular.y   = inertia_ang
set_req.inertia_angular.z   = inertia_ang
set_req.damping_linear.x    = damping_lin
set_req.damping_linear.y    = damping_lin
set_req.damping_linear.z    = damping_lin
set_req.damping_angular.x   = damping_ang
set_req.damping_angular.y   = damping_ang
set_req.damping_angular.z   = damping_ang
set_req.force_min_linear.x  = force_min_lin
set_req.force_min_linear.y  = force_min_lin
set_req.force_min_linear.z  = force_min_lin
set_req.force_min_angular.x = force_min_ang
set_req.force_min_angular.y = force_min_ang
set_req.force_min_angular.z = force_min_ang
set_req.force_max_linear.x  = force_max_lin
set_req.force_max_linear.y  = force_max_lin
set_req.force_max_linear.z  = force_max_lin
set_req.force_max_angular.x = force_max_ang
set_req.force_max_angular.y = force_max_ang
set_req.force_max_angular.z = force_max_ang

srv_set.call(set_req)


while not rospy.is_shutdown():
    rospy.spin()

srv_stop.call()

#rospy.spin()

