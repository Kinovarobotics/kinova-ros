#! /usr/bin/env python
"""A set of example functions that can be used to control the arm"""
import rospy
import actionlib

import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
import thread
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import argparse

def argumentParser(argument):
    	""" Argument parser """
    	parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    	parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
	args_ = parser.parse_args(argument)
	prefix = args_.kinova_robotType + "_"
	nbJoints = int(args_.kinova_robotType[3])	
	return prefix, nbJoints


def joint_position_client(angle_set, prefix):
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
    return client.get_result()  


def cartesian_pose_client(position, orientation, prefix):
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


def gripper_client(finger_positions, prefix):
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


def homeRobot(prefix):
	service_address = '/' + prefix + 'driver/in/home_arm'
	rospy.wait_for_service(service_address)
        try:
           home = rospy.ServiceProxy(service_address, HomeArm)
           home()
           return None
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e


def activateNullSpaceMode(duration_sec, prefix):
	service_address = '/' + prefix + 'driver/in/set_null_space_mode_state'
	rospy.wait_for_service(service_address)
        try:
           SetNullSpaceMode = rospy.ServiceProxy(service_address, SetNullSpaceModeState)
           SetNullSpaceMode(1)           
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	rospy.sleep(duration_sec)
	try:           
           SetNullSpaceMode(0)
           return None
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e


def publishVelCmd(jointCmds, duration_sec, prefix):
	
  #subscriber to get feedback    
  topic_name = '/' + prefix + 'driver/out/joint_state'
  max_error = [0,0,0,0,0,0,0]
  counter = [0]
  sub = rospy.Subscriber(topic_name, JointState, getFeedbackCallback, (jointCmds,'velocity',max_error,counter))

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
  joint_cmd_for_error_comp = jointCmd
  count = 0		
  rate = rospy.Rate(100)
  L = []
  thread.start_new_thread(input_thread, (L,))
  while (count < 100*duration_sec):
    count = count + 1
    #rospy.loginfo("I will publish to the topic %d", count)
    pub.publish(jointCmd)
    rate.sleep()    
    if L:            
      break
  sub.unregister()
  print "max error %f %f %f %f %f %f %f" %(max_error[0], max_error[1], max_error[2], max_error[3], max_error[4], max_error[5], max_error[6])


def publishCatesianVelocityCommands(cartVel, duration_sec, prefix):
	topic_name = '/' + prefix + 'driver/in/cartesian_velocity'
	#publish joint torque commands
	pub = rospy.Publisher(topic_name, kinova_msgs.msg.PoseVelocity, queue_size=1)
	poseVelCmd = kinova_msgs.msg.PoseVelocity()
	poseVelCmd.twist_linear_x = cartVel[0];
	poseVelCmd.twist_linear_y = cartVel[1];
	poseVelCmd.twist_linear_z = cartVel[2];
	poseVelCmd.twist_angular_x = cartVel[3];
	poseVelCmd.twist_angular_y = cartVel[4];
	poseVelCmd.twist_angular_z = cartVel[5];
	count = 0	
	rate = rospy.Rate(100)
	while (count < 100*duration_sec):
		count = count + 1		
		pub.publish(poseVelCmd)
		rate.sleep()
		

def publishForceCmd(force_cmds, duration_sec, prefix):
	
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
	topic_name = '/' + prefix + 'driver/in/cartesian_force'
	pub = rospy.Publisher(topic_name, kinova_msgs.msg.CartesianForce, queue_size=1)
        force = kinova_msgs.msg.CartesianForce()
	force.force_x = force_cmds[0];
	force.force_y = force_cmds[1];
	force.force_z = force_cmds[2];
	force.torque_x = force_cmds[3];
	force.torque_y = force_cmds[4];
	force.torque_z = force_cmds[5];
	count = 0		
	rate = rospy.Rate(100)
	L = []
	thread.start_new_thread(input_thread, (L,))
	while (count < 100*duration_sec):
		count = count + 1		
		pub.publish(force)
		rate.sleep()
		if L: break

	#use service to switch to position control	
	try:           
		switchTorquemode(0)
		return None
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return None


def publishTorqueCmd(jointCmds, duration_sec, prefix):	

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

  #subscriber to get feedback    
  topic_name = '/' + prefix + 'driver/out/joint_state'
  max_error = [0,0,0,0,0,0,0]
  counter = [0]
  sub = rospy.Subscriber(topic_name, JointState, getFeedbackCallback, (jointCmds,'torque',max_error,counter))

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
  L = []
  thread.start_new_thread(input_thread, (L,))
  while (count<100*duration_sec):		
	  pub.publish(jointCmd)
	  count = count + 1
	  rate.sleep()
	  if L: break
  sub.unregister()
  print "max error %f %f %f %f %f %f %f" %(max_error[0], max_error[1], max_error[2], max_error[3], max_error[4], max_error[5], max_error[6])

  #use service to switch to position control	
  try:           
	  switchTorquemode(0)
	  return None
  except rospy.ServiceException, e:
	  print "Service call failed: %s"%e
	  return None


def ZeroTorque(prefix):
	#move robot to candle like pose
	#result = joint_position_client([180]*7)

	print "torque before setting zero"
	topic_name = '/' + prefix + 'driver/out/joint_torques'
	sub_once = rospy.Subscriber(topic_name, kinova_msgs.msg.JointAngles, printTorqueVaules) 
	rospy.wait_for_message(topic_name, kinova_msgs.msg.JointAngles, timeout=2)
	sub_once.unregister()
	
	#call zero torque
	service_address = '/' + prefix + 'driver/in/set_zero_torques'
	rospy.wait_for_service(service_address)
	try:
		zeroTorques = rospy.ServiceProxy(service_address, ZeroTorques)
		zeroTorques()           
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return None	
	
	rospy.sleep(0.5)
	print "torque after setting zero"
	sub_once = rospy.Subscriber(topic_name, kinova_msgs.msg.JointAngles, printTorqueVaules) 
	rospy.wait_for_message(topic_name, kinova_msgs.msg.JointAngles, timeout=2)
	sub_once.unregister()


def printTorqueVaules(torques):
	print "Torque - {}, {}, {}, {}, {}, {}, {}".format(torques.joint1, 
	torques.joint2, torques.joint3, torques.joint4, 
	torques.joint5, torques.joint6, torques.joint7)


def input_thread(L):
    raw_input("Press return to return to position control mode")
    L.append(None)

def getFeedbackCallback(data,args): 
    #generic but joint_state/effort is not published by kinova_driver
    joint_cmd = args[0]
    error_type = args[1]
    max_error = args[2]
    count = args[3]
    for i in range(0,len(joint_cmd)):
      if error_type == 'velocity':
       error = abs(joint_cmd[i] - data.velocity[i]*180/3.1415)
      if error_type == 'torque':
       error = abs(joint_cmd[i] - data.effort[i])     
      if count[0]>50:     
        max_error[i] = max(error,max_error[i])
      count[0] = count[0] +1    
