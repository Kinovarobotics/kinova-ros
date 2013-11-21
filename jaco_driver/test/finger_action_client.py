#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_driver')
import rospy

import actionlib

import jaco_driver.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/jaco/finger_joint_angles', jaco_driver.msg.SetFingersPositionAction)

    goal = jaco_driver.msg.SetFingersPositionGoal()

    if len(sys.argv) < 4:
        goal.fingers.Finger_1 = 4000
        goal.fingers.Finger_2 = 4000
        goal.fingers.Finger_3 = 4000

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.fingers.Finger_1 = float(sys.argv[1])
        goal.fingers.Finger_2 = float(sys.argv[2])
        goal.fingers.Finger_3 = float(sys.argv[3])

    client.wait_for_server()
    rospy.loginfo("Connected to Finger server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('finger_pose_client')
        result = pose_client()
        rospy.loginfo("Result: \n%s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
