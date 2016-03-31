#!/usr/bin/env python

import roslib; roslib.load_manifest('kinova_driver')
import rospy

import actionlib

import kinova_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/kinova/arm_joint_angles', kinova_msgs.msg.ArmJointAnglesAction)

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    if len(sys.argv) < 7:
        goal.angles.joint1 = 1.5285271406173706 
        goal.angles.joint2 = -1.3800612688064575
        goal.angles.joint3 = -0.1439174860715866
        goal.angles.joint4 = 0.15510250627994537
        goal.angles.joint5 = 0.6960597634315491
        goal.angles.joint6 = 3.3098342418670654

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.angles.joint1 = float(sys.argv[1])
        goal.angles.joint2 = float(sys.argv[2])
        goal.angles.joint3 = float(sys.argv[3])
        goal.angles.joint4 = float(sys.argv[4])
        goal.angles.joint5 = float(sys.argv[5])
        goal.angles.joint6 = float(sys.argv[6])


    client.wait_for_server()
    rospy.loginfo("Connected to Pose server")

    client.send_goal(goal)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted, pre-empting goal")
        client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('arm_pose_client')
        result = pose_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
