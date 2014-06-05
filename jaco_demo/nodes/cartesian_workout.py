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
    client = actionlib.SimpleActionClient('jaco/arm_pose', jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id='arm_base')
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
    if len(sys.argv) not in [2, 3, 8] or 'help' in str(sys.argv):
        print('Usage:')
        print('    cartesian_workout.py random num          - randomly generate num poses')
        print('    cartesian_workout.py file_path           - use poses from file')
        print('    cartesian_workout.py x y z qx qy qz qw   - use that specific pose')
        exit()

    try:
        rospy.init_node('cartesian_workout')

        if str(sys.argv[1]) == 'random' and len(sys.argv) == 3:
            print('Using {} randomly generated poses'.format(int(sys.argv[2])))
            poses = goal_generators.random_pose_generator(int(sys.argv[2]))
        elif len(sys.argv) == 2:
            print('Using poses from file: {}'.format(sys.argv[1]))
            poses = goal_generators.poses_from_file(str(sys.argv[1]))
        else:
            print('Using the specified pose:')
            raw_pose = [float(n) for n in sys.argv[1:]]
            mag = np.sqrt(sum(np.power(raw_pose[3:], 2)))
            poses = [(raw_pose[:3], raw_pose[3:] / mag)]

        for pos, orient in poses:
            print('    position: {},  orientation: {}'.format(pos, orient))
            result = cartesian_pose_client(pos, orient)

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
