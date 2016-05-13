#! /usr/bin/env python
"""A helper program to test gripper goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys

import actionlib
import kinova_msgs.msg

import argparse


""" Global variable """
numJoint = 0
numFinger = 0
prefix = 'NO_ROBOT_TYPE_DEFINED'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger


def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + '_arm_driver/fingers_action/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])

    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive fingers to command position')
    parser.add_argument('robotType', metavar='robotType', type=int, choices=range(7),
                        help='Index for robotType: JACOV1_ASSISTIVE_3FINGERS = 0, MICO_6DOF_SERVICE_2FINGERS = 1, MICO_4DOF_SERVICE_2FINGERS = 2, JACOV2_6DOF_SERVICE_3FINGERS = 3, JACOV2_4DOF_SERVICE_3FINGERS = 4, MICO_6DOF_ASSISTIVE_2FINGER2 = 5, JACOV2_6DOF_ASSISTIVE_3FINGERS = 6')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='turn',
                        choices={'turn', 'mm', 'percent'},
                        help='Unit of finger motion command, in turn[0, 6800], mm[0, 9.45], percent[0,100]')
    parser.add_argument('finger_value', nargs='*', type=float, help='finger values, length equals to number of fingers.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display finger values in alternative convention(turn, mm or percent)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    return args_


def robotTypeParser(robotType_):
    """ Argument robotType """
    global numJoint, numFinger, prefix, finger_maxDist, finger_maxTurn
    if robotType_ == 0:
        numJoint = 6
        numFinger = 3
        # prefix = 'j16a3'
        prefix = 'jaco'
    elif robotType_ == 1:
        numJoint = 6
        numFinger = 2
        # prefix = 'm16s2'
        prefix = 'mico'
    elif robotType_ == 2:
        numJoint = 4
        numFinger = 2
        # prefix = 'm14s2'
        prefix = 'mico'
    elif robotType_ == 3:
        numJoint = 6
        numFinger = 3
        # prefix = 'j26s3'
        prefix = 'jaco'
    elif robotType_ == 4:
        numJoint = 4
        numFinger = 3
        # prefix = 'j24s3'
        prefix = 'jaco'
    elif robotType_ == 5:
        numJoint = 6
        numFinger = 2
        finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
        finger_maxTurn = 6800  # max thread turn for one finger
        # prefix = 'm16a2' # refefine robotType m6a2-->mico-6DOF-assistive-2Fingers
        prefix = 'mico'
    elif robotType_ == 6:
        numJoint = 6
        numFinger = 3
        # prefix = 'j26a3'
        prefix = 'jaco'
    else:
        raise Exception('Undefined robotType: {}'.format(robotType_))


def unitParser(unit_, finger_value_):
    """ Argument unit """
    if unit_ == 'turn':
        finger_turn_ = finger_value_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_value_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_value_]
    elif unit_ == 'mm':
        finger_turn_ = [x/1000 * finger_maxTurn / finger_maxDist for x in finger_value_]
        finger_meter_ = finger_value_
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn]
    elif unit_ == 'percent':
        finger_turn_ = [x * finger_maxTurn / 100.0 for x in finger_value_]
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn]
        finger_percent_ = finger_value_
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn_, finger_meter_, finger_percent_


def verboseParser(verbose_, finger_turn_):
    """ Argument verbose """
    if verbose_:
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
        print('Finger values in turn are: ')
        print(', '.join('finger{:1.0f} {:4.0f}'.format(k[0] + 1, k[1]) for k in enumerate(finger_turn_)))
        print('Finger values in mm are: ')
        print(', '.join('finger{:1.0f} {:2.1f}'.format(k[0]+1, k[1]*1000) for k in enumerate(finger_meter_)))
        print('Finger values in percentage are: ')
        print(', '.join('finger{:1.1f} {:3.1f}%'.format(k[0]+1, k[1]) for k in enumerate(finger_percent_)))


if __name__ == '__main__':

    args = argumentParser(None)

    robotTypeParser(args.robotType)

    if len(args.finger_value) != numFinger:
        print('Number of input values {} is not equal to number of fingers {}. Please run help to check number of fingers with different robot type.'.format(len(args.finger_value), numFinger))
        sys.exit(0)

    finger_turn, finger_meter, finger_percent = unitParser(args.unit, args.finger_value)

    try:
        rospy.init_node(prefix + '_gripper_workout')

        if numFinger == 0:
            print('Finger number is 0, check with "-h" to see how to use this node.')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            exit()
        else:
            positions_temp1 = [max(0.0, n) for n in finger_turn]
            positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
            positions = [float(n) for n in positions_temp2]

        result = gripper_client(positions)

        print('Finger position sent!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')


    verboseParser(args.verbose, positions)
