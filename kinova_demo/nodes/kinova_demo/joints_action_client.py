#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse


""" Global variable """
numJoint = 0
numFinger = 0
prefix = 'NO_ROBOT_TYPE_DEFINED'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger


def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/' + prefix + '_arm_driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


def argumentParser(argument):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    parser.add_argument('robotType', metavar='robotType', type=int, choices=range(7),
                        help='Index for robotType: JACOV1_ASSISTIVE_3FINGERS = 0, MICO_6DOF_SERVICE_2FINGERS = 1, MICO_4DOF_SERVICE_2FINGERS = 2, JACOV2_6DOF_SERVICE_3FINGERS = 3, JACOV2_4DOF_SERVICE_3FINGERS = 4, MICO_6DOF_ASSISTIVE_2FINGER2 = 5, JACOV2_6DOF_ASSISTIVE_3FINGERS = 6')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='degree',
                        choices={'degree', 'radian'},
                        help='Unit of joiint motion command, in degree, radian')
    parser.add_argument('joint_value', nargs='*', type=float, help='joint values, length equals to number of joints.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display joint values in alternative convention(degree or radian)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument)
    return args_


def robotTypeParser(robotType):
    """ Argument robotType """
    global numJoint, numFinger, prefix, finger_maxDist, finger_maxTurn
    if robotType == 0:
        numJoint = 6
        numFinger = 3
        # prefix = 'j16a3'
        prefix = 'jaco'
    elif robotType == 1:
        numJoint = 6
        numFinger = 2
        # prefix = 'm16s2'
        prefix = 'mico'
    elif robotType == 2:
        numJoint = 4
        numFinger = 2
        # prefix = 'm14s2'
        prefix = 'mico'
    elif robotType == 3:
        numJoint = 6
        numFinger = 3
        # prefix = 'j26s3'
        prefix = 'jaco'
    elif robotType == 4:
        numJoint = 4
        numFinger = 3
        # prefix = 'j24s3'
        prefix = 'jaco'
    elif robotType == 5:
        numJoint = 6
        numFinger = 2
        finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
        finger_maxTurn = 6800  # max thread turn for one finger
        # prefix = 'm16a2' # refefine robotType m6a2-->mico-6DOF-assistive-2Fingers
        prefix = 'mico'
    elif robotType == 6:
        numJoint = 6
        numFinger = 3
        # prefix = 'j26a3'
        prefix = 'jaco'
    else:
        raise Exception('Undefined robotType: {}'.format(robotType))


def unitParser(unit, joint_value):
    """ Argument unit """
    if unit == 'degree':
        joint_degree = joint_value
        joint_radian = list(map(math.radians, joint_degree))
    elif unit == 'radian':
        joint_radian = joint_value
        joint_degree = list(map(math.degrees, joint_radian))
    else:
        raise Exception("Joint value have to be in degree, or radian")

    return joint_degree, joint_radian


def verboseParser(verbose, joint_degree):
    """ Argument verbose """
    if verbose:
        joint_radian = list(map(math.radians, joint_degree))
        print('Joint values in degree are: ')
        print(', '.join('joint{:1.0f} {:3.1f}'.format(k[0] + 1, k[1]) for k in enumerate(joint_degree)))
        print('Joint values in radian are: ')
        print(', '.join('joint{:1.0f} {:1.3f}'.format(k[0]+1, k[1]) for k in enumerate(joint_radian)))


if __name__ == '__main__':

    args = argumentParser(None)

    robotTypeParser(args.robotType)

    if len(args.joint_value) != numJoint:
        print('Number of input values {} is not equal to number of joints {}. Please run help to check number of joints with different robot type.'.format(len(args.joint_value), numJoint))
        sys.exit(0)

    joint_degree, joint_radian = unitParser(args.unit, args.joint_value)

    try:
        rospy.init_node(prefix + '_gripper_workout')

        if numJoint == 0:
            print('Joint number is 0, check with "-h" to see how to use this node.')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            sys.exit()
        else:
            positions = [float(n) for n in joint_degree]

        result = joint_angle_client(positions)

        print('Joint angular position sent!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')

    verboseParser(args.verbose, positions)