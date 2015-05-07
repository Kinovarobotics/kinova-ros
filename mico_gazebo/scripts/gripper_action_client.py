#! /usr/bin/env python

import roslib
import rospy
from control_msgs.msg import *
import math

# Brings in the SimpleActionClient
import actionlib


def mico_gripper_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('gripper_action_server', control_msgs.msg.GripperCommandAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position=60.0*math.pi/180.0

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
        result = mico_gripper_client()
        print "Result:" + str(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
