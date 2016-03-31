#!/usr/bin/env python

import roslib; roslib.load_manifest('kinova_driver')
import rospy

import actionlib

import kinova_msgs.msg

import sys


def pose_client():
    client = actionlib.SimpleActionClient('/kinova/tool_pose', kinova_msgs.msg.ArmPoseAction)

    goal = kinova_msgs.msg.ArmPoseGoal()

    goal.pose.header.frame_id = "/kinova_api_origin"
    pose = goal.pose.pose

    if len(sys.argv) < 8: # default pose
        pose.position.x = -0.314269423485
        pose.position.y = -0.339179039001
        pose.position.z = 0.600132465363

        pose.orientation.x = -0.590686044496
        pose.orientation.y = -0.519369415388
        pose.orientation.z = 0.324703360925
        pose.orientation.w = 0.525274342226

        rospy.logwarn("Using test goal: \n%s", goal)
    else: # use pose from command line
        pose.position.x = float(sys.argv[1])
        pose.position.y = float(sys.argv[2])
        pose.position.z = float(sys.argv[3])

        pose.orientation.x = float(sys.argv[4])
        pose.orientation.y = float(sys.argv[5])
        pose.orientation.z = float(sys.argv[6])
        pose.orientation.w = float(sys.argv[7])

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
        rospy.init_node('tool_pose_client')
        result = pose_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
