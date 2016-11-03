#! /usr/bin/env python
"""A test program to test action servers for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import actionlib

import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
from kinova_msgs.srv import *

prefix = 'j2s7s300_'


def joint_position_client(angle_set):
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
    
    client.wait_for_result(rospy.Duration(100.0))

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(200.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(50.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None

def homeRobot():
	service_address = '/' + prefix + 'driver/in/home_arm'
	rospy.wait_for_service(service_address)
        try:
           home = rospy.ServiceProxy(service_address, HomeArm)
           home()
           return None
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_action_servers')
	
        #test joint srv - move robot to 180 pos
        #jointCommand180 = [180]*7
        #result = joint_position_client(jointCommand180)

	#test cartesian srv
	#move robot in joint space to init pose
        jointCmdInitPose = [270,220,0,90,180,270,0]
        result = joint_position_client(jointCmdInitPose)

	#homeRobot()

        #set cartesian pose 1
	position = [0.4,-0.5,0]
        quaternion = tf.transformations.quaternion_from_euler(90*3.1415/180, 0, 0,'rxyz')	
	result = cartesian_pose_client(position, quaternion)	

	#open gripper
	result = gripper_client([0,0,0])

	#set cartesian pose 2
	position = [0.0,-0.5,0.2]
	quaternion = tf.transformations.quaternion_from_euler(180*3.1415/180, 0, 0,'rxyz')
	result = cartesian_pose_client(position, quaternion)

	#close gripper
	result = gripper_client([0,6800,0])
	
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
