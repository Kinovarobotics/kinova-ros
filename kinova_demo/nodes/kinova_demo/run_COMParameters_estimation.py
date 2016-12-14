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
    return client.get_result()  

def argumentParser(argument):
    	""" Argument parser """
    	parser = argparse.ArgumentParser(description='Drive robot joint to command position')
    	parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
	args_ = parser.parse_args(argument)
	prefix = args_.kinova_robotType + "_"
	nbJoints = int(args_.kinova_robotType[3])	

def ZeroTorque():
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

def runCOMParameterEstimation():
	service_address = '/' + prefix + 'driver/in/run_COM_parameters_estimation'
	rospy.wait_for_service(service_address)
	try:
           runEstimation = rospy.ServiceProxy(service_address, RunCOMParametersEstimation)
           runEstimation()           
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
	   return None	

def printTorqueVaules(torques):
	print "Torque - {}, {}, {}, {}, {}, {}, {}".format(torques.joint1, 
	torques.joint2, torques.joint3, torques.joint4, 
	torques.joint5, torques.joint6, torques.joint7)

if __name__ == '__main__':
    try:        
	args = argumentParser(None)	
        rospy.init_node('torque_compensated_mode')
	if (interactive == True):        
		nb = raw_input('Moving robot to candle like position, press return to start')
        result = joint_position_client([180]*7)

	if (interactive == True):        
		nb = raw_input('Setting torques to zero, press return')
	#test zero torque
	ZeroTorque()

	if (interactive == True):        
		nb = raw_input('Sarting COM parameters estimation, press return')
	runCOMParameterEstimation()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
