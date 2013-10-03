#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_driver')
import rospy

import actionlib

import jaco_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/jaco/arm_joint_angles', jaco_msgs.msg.ArmJointAnglesAction)

    goal = jaco_msgs.msg.ArmJointAnglesGoal()

    if len(sys.argv) < 7:
        goal.angles.Angle_J1 = 1.5285271406173706 
        goal.angles.Angle_J2 = -1.3800612688064575
        goal.angles.Angle_J3 = -0.1439174860715866
        goal.angles.Angle_J4 = 0.15510250627994537
        goal.angles.Angle_J5 = 0.6960597634315491
        goal.angles.Angle_J6 = 3.3098342418670654

        rospy.logwarn("Using test goal: \n%s", goal)
    else:
        goal.angles.Angle_J1 = float(sys.argv[1])
        goal.angles.Angle_J2 = float(sys.argv[2])
        goal.angles.Angle_J3 = float(sys.argv[3])
        goal.angles.Angle_J4 = float(sys.argv[4])
        goal.angles.Angle_J5 = float(sys.argv[5])
        goal.angles.Angle_J6 = float(sys.argv[6])


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
        
