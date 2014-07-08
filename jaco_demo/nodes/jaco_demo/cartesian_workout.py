#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys
import numpy as np

import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import goal_generators


def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + str(sys.argv[1]) + '_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(str(sys.argv[1]) + '_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None


if __name__ == '__main__':
    if len(sys.argv) not in [3, 4, 9] or 'help' in str(sys.argv):
        print('Usage:')
        print('    cartesian_workout.py node_name random num          - randomly generate num poses')
        print('    cartesian_workout.py node_name file_path           - use poses from file')
        print('    cartesian_workout.py node_name x y z qx qy qz qw   - use that specific pose')
        exit()

    try:
        rospy.init_node(str(sys.argv[1]) + '_cartesian_workout')

        if str(sys.argv[2]) == 'random' and len(sys.argv) == 4:
            print('Using {} randomly generated poses'.format(int(sys.argv[3])))
            poses = goal_generators.random_pose_generator(int(sys.argv[3]))
        elif len(sys.argv) == 3:
            print('Using poses from file: {}'.format(sys.argv[2]))
            poses = goal_generators.poses_from_file(str(sys.argv[2]))
        else:
            print('Using the specified pose:')
            raw_pose = [float(n) for n in sys.argv[2:]]
            mag = np.sqrt(sum(np.power(raw_pose[3:], 2)))
            """poses = [(raw_pose[:3], raw_pose[3:] / mag)]"""
            poses = [(raw_pose[:3], raw_pose[3:])]

        for pos, orient in poses:
            print('    position: {},  orientation: {}'.format(pos, orient))
            result = cartesian_pose_client(pos, orient)

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
