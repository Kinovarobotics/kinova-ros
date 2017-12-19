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
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentJointCommand = [] # number of joints is defined in __main__

def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/' + prefix + 'driver/joints_action/joint_angles'
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
    goal.angles.joint7 = angle_set[6]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


def getcurrentJointCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print 'position listener obtained message for joint position. '


def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])

    # print 'currentJointCommand is: '
    # print currentJointCommand


def argumentParser(argument):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='degree',
                        choices={'degree', 'radian'},
                        help='Unit of joiint motion command, in degree, radian')
    parser.add_argument('joint_value', nargs='*', type=float, help='joint values, length equals to number of joints.')
    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display joint values in alternative convention(degree or radian)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument)
    return args_


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit, joint_value, relative_):
    """ Argument unit """
    global currentJointCommand

    if unit == 'degree':
        joint_degree_command = joint_value
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_
        joint_radian = list(map(math.radians, joint_degree_absolute_))
    elif unit == 'radian':
        joint_degree_command = list(map(math.degrees, joint_value))
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_
        joint_radian = list(map(math.radians, joint_degree_absolute_))
    else:
        raise Exception("Joint value have to be in degree, or radian")

    return joint_degree, joint_radian


def verboseParser(verbose, joint_degree):
    """ Argument verbose """
    if verbose:
        joint_radian = list(map(math.radians, joint_degree))
        print('Joint values sent in degree are: ')
        print(', '.join('joint{:1.0f} {:3.1f}'.format(k[0] + 1, k[1]) for k in enumerate(joint_degree)))
        print('Joint values sent in radian are: ')
        print(', '.join('joint{:1.0f} {:1.3f}'.format(k[0]+1, k[1]) for k in enumerate(joint_radian)))


if __name__ == '__main__':

    args = argumentParser(None)

    kinova_robotTypeParser(args.kinova_robotType)
    rospy.init_node(prefix + 'gripper_workout')

    # currentJointCommand = [0]*arm_joint_number
    # KinovaType defines AngularInfo has 7DOF, so for published topics on joints.
    currentJointCommand = [0]*7

    if len(args.joint_value) != arm_joint_number:
        print('Number of input values {} is not equal to number of joints {}. Please run help to check number of joints with different robot type.'.format(len(args.joint_value), arm_joint_number))
        sys.exit(0)

    # get Current finger position if relative position
    getcurrentJointCommand(prefix)
    joint_degree, joint_radian = unitParser(args.unit, args.joint_value, args.relative)

    positions = [0]*7
    try:

        if arm_joint_number < 1:
            print('Joint number is 0, check with "-h" to see how to use this node.')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            sys.exit() 
        else:
            for i in range(0,arm_joint_number):
              positions[i] = joint_degree[i]               

        result = joint_angle_client(positions)

    except rospy.ROSInterruptException:
        print('program interrupted before completion')

    verboseParser(args.verbose, joint_degree)
