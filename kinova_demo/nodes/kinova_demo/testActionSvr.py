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
import argparse

prefix = 'j2s7s300_'
nbJoints = 7
interactive = True


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

def activateNullSpaceMode():
	service_address = '/' + prefix + 'driver/in/set_null_space_mode_state'
	rospy.wait_for_service(service_address)
        try:
           SetNullSpaceMode = rospy.ServiceProxy(service_address, SetNullSpaceModeState)
           SetNullSpaceMode(1)           
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	rospy.sleep(10)
	try:           
           SetNullSpaceMode(0)
           return None
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e

def argumentParser(argument):
    	""" Argument parser """
    	parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    	parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
	args_ = parser.parse_args(argument)
	prefix = args_.kinova_robotType + "_"
	nbJoints = int(args_.kinova_robotType[3])	

def publishVelCmd(jointCmds):
	topic_name = '/' + prefix + 'driver/in/joint_velocity'
	pub = rospy.Publisher(topic_name, kinova_msgs.msg.JointVelocity, queue_size=1)
        jointCmd = kinova_msgs.msg.JointVelocity()
	jointCmd.joint1 = jointCmds[0];
	jointCmd.joint2 = jointCmds[1];
	jointCmd.joint3 = jointCmds[2];
	jointCmd.joint4 = jointCmds[3];
	jointCmd.joint5 = jointCmds[4];
	jointCmd.joint6 = jointCmds[5];
	jointCmd.joint7 = jointCmds[6];
	count = 0		
	rate = rospy.Rate(100)
	while (count < 500):
		count = count + 1
		#rospy.loginfo("I will publish to the topic %d", count)
		pub.publish(jointCmd)
		rate.sleep()

def publishTorqueCmd(jointCmds):
	
	#use service to set torque control parameters	
	service_address = '/' + prefix + 'driver/in/set_torque_control_parameters'	
	rospy.wait_for_service(service_address)
	try:
           setTorqueParameters = rospy.ServiceProxy(service_address, SetTorqueControlParameters)
           setTorqueParameters()           
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	   return None	

	#use service to switch to torque control	
	service_address = '/' + prefix + 'driver/in/set_torque_control_mode'	
	rospy.wait_for_service(service_address)
	try:
           switchTorquemode = rospy.ServiceProxy(service_address, SetTorqueControlMode)
           switchTorquemode(1)           
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	   return None	

	#publish joint torque commands
	topic_name = '/' + prefix + 'driver/in/joint_torque'
	pub = rospy.Publisher(topic_name, kinova_msgs.msg.JointTorque, queue_size=1)
        jointCmd = kinova_msgs.msg.JointTorque()
	jointCmd.joint1 = jointCmds[0];
	jointCmd.joint2 = jointCmds[1];
	jointCmd.joint3 = jointCmds[2];
	jointCmd.joint4 = jointCmds[3];
	jointCmd.joint5 = jointCmds[4];
	jointCmd.joint6 = jointCmds[5];
	jointCmd.joint7 = jointCmds[6];
	count = 0		
	rate = rospy.Rate(100)
	while (count < 1000):
		count = count + 1
		#rospy.loginfo("I will publish to the topic %d", count)
		pub.publish(jointCmd)
		rate.sleep()

	#use service to switch to position control	
	try:           
           switchTorquemode(0)
           return None
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	   return None

def publishCatesianVelocityCommands(cartVel):
	topic_name = '/' + prefix + 'driver/in/cartesian_velocity'
	#publish joint torque commands
	pub = rospy.Publisher(topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)
	poseVelCmd = kinova_msgs.msg.PoseVelocity()
	poseVelCmd.twist_linear_x = cartVel[0];
	poseVelCmd.twist_linear_y = cartVel[1];
	poseVelCmd.twist_linear_z = cartVel[2];
	poseVelCmd.twist_angular_x = cartVel[3];
	poseVelCmd.twist_angular_x = cartVel[4];
	poseVelCmd.twist_angular_x = cartVel[5];
	count = 0
	rate = rospy.Rate(100)
	while (count < 500):
		count = count + 1		
		pub.publish(poseVelCmd)
		rate.sleep()
	

if __name__ == '__main__':
    try:        
	args = argumentParser(None)	
        rospy.init_node('test_action_servers')	  
		
	#test joint srv - move robot to 180 pos
	if (interactive == True):        
		nb = raw_input('Moving robot to candle like position, press key')
        result = joint_position_client([180]*7)

	#test joint velocity control
	if (interactive == True):        
		nb = raw_input('Testing velocity control, press key')
	if (nbJoints == 7):
		publishVelCmd([10,0,-10,0,10,0,-10])
	else:
		publishVelCmd([10,0,0,-10,0,0,0])
	rospy.sleep(5)
	
	#test joint torque control
	if (interactive == True):        
		nb = raw_input('Testing torque control, enabled for 10 sec, press key')
	publishTorqueCmd([0,0,0,0,0,0,0])
	
	
	#test cartesian srv
	if (interactive == True):        
		nb = raw_input('Testing Cartesian control, press key')
	#move robot in joint space to init pose
	if (nbJoints == 7):
        	result = joint_position_client([270,220,0,90,180,270,0])
	else:
		result = joint_position_client([270,220,90,180,270,0,0])
        

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
	result = gripper_client([6800,6800,6800])

	#test cartesian velocity publisher
	if (interactive == True):        
		nb = raw_input('Testing Cartesian velocity control, press key')
	publishCatesianVelocityCommands([-0.1, 0, -0.1, 0, 0, 0])
	publishCatesianVelocityCommands([0.1, 0, 0.1, 0, 0, 0])

	#set null space mode
	if (interactive == True):        
		nb = raw_input('Testing Null space control control,active for 10s, use joystick to move robot in Null space,press key')
	activateNullSpaceMode()

	print("done!")
	
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
