#! /usr/bin/env python

import roslib
import rospy
from control_msgs.msg import *
import math
import getopt
# Brings in the SimpleActionClient
import actionlib


def mico_gripper_client(argv):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    try:
      opts, args = getopt.getopt(argv,"")
      if len(args)!=1:
         print 'gripper_action_client.py <open/close>'
         return False
    except getopt.GetoptError:
      print 'gripper_action_client.py <open/close>'
      sys.exit(2)
    client = actionlib.SimpleActionClient('gripper_action_server', control_msgs.msg.GripperCommandAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.GripperCommandGoal()
    if args[0]=="close":
	goal.command.position=60.0*math.pi/180.0
    elif args[0]=="open":
	goal.command.position=0.0
    else:
	return False

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mico_gripper_client_py')
        result = mico_gripper_client(sys.argv[1:])
        print "Result:" + str(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
