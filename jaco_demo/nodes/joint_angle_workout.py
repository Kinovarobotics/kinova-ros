#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys

import actionlib
import jaco_msgs.msg

import goal_generators


def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    client = actionlib.SimpleActionClient('jaco/arm_joint_angles', jaco_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the joint angle action timed-out')
        return None


if __name__ == '__main__':
    if len(sys.argv) not in [2, 3, 7] or 'help' in str(sys.argv):
        print('Usage:')
        print('    joint_angle_workout.py random num          - randomly generate num joint angle sets')
        print('    joint_angle_workout.py file_path           - use poses from file')
        print('    joint_angle_workout.py j1 j2 j3 j4 j5 j6   - use these specific angle')
        exit()

    try:
        rospy.init_node('joint_angle_workout')

        if str(sys.argv[1]) == 'random' and len(sys.argv) == 3:
            print('Using {} randomly generated joint angle sets'.format(int(sys.argv[2])))
            angles = goal_generators.random_joint_angles_generator(int(sys.argv[2]))
        elif len(sys.argv) == 2:
            print('Using joint angles from file: {}'.format(sys.argv[1]))
            angles = goal_generators.joint_angles_from_file(str(sys.argv[1]))
        else:
            print('Using the specified joint angles:')
            raw_angles = [float(n) for n in sys.argv[2:]]
            angles = [raw_angles]

        for angle_set in angles:
            print('    angles: {}'.format(angle_set))
            result = joint_angle_client(angle_set)

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
