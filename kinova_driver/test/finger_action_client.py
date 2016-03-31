#!/usr/bin/env python

import roslib; roslib.load_manifest('kinova_msgs')
import rospy

import actionlib

import kinova_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/kinova/finger_joint_angles', kinova_msgs.msg.SetFingersPositionAction)

    goal = kinova_msgs.msg.SetFingersPositionGoal()

    if len(sys.argv) < 4:
        goal.fingers.finger1 = 4000
        goal.fingers.finger2 = 4000
        goal.fingers.finger3 = 4000

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.fingers.finger1 = float(sys.argv[1])
        goal.fingers.finger2 = float(sys.argv[2])
        goal.fingers.finger3 = float(sys.argv[3])

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
        
