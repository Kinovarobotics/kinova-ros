#! /usr/bin/env python
# Wrappers around the services provided by rosified gazebo

import sys
import rospy
import os
import time

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench

#3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732, 3.1415927410125732

def spawn_sdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rospy.loginfo("Waiting for service %s/spawn_sdf_model"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/spawn_sdf_model')
    try:
      spawn_sdf_model = rospy.ServiceProxy(gazebo_namespace+'/spawn_sdf_model', SpawnModel)
      rospy.loginfo("Calling service %s/spawn_sdf_model"%gazebo_namespace)
      resp = spawn_sdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
      rospy.loginfo("Spawn status: %s"%resp.status_message)
      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def spawn_urdf_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace):
    rospy.loginfo("Waiting for service %s/spawn_urdf_model"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/spawn_urdf_model')
    try:
      spawn_urdf_model = rospy.ServiceProxy(gazebo_namespace+'/spawn_urdf_model', SpawnModel)
      rospy.loginfo("Calling service %s/spawn_urdf_model"%gazebo_namespace)
      resp = spawn_urdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
      rospy.loginfo("Spawn status: %s"%resp.status_message)
      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def set_model_configuration_client(model_name, model_param_name, joint_names, joint_positions, gazebo_namespace):
    rospy.loginfo("Waiting for service %s/set_model_configuration"%gazebo_namespace)
    rospy.wait_for_service(gazebo_namespace+'/set_model_configuration')
    rospy.loginfo("temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.");
    time.sleep(1)
    try:
      set_model_configuration = rospy.ServiceProxy(gazebo_namespace+'/set_model_configuration', SetModelConfiguration)
      rospy.loginfo("Calling service %s/set_model_configuration"%gazebo_namespace)
      resp = set_model_configuration(model_name, model_param_name, joint_names, joint_positions)
      rospy.loginfo("Set model configuration status: %s"%resp.status_message)

      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        set_model_configuration_client("mico_arm", "/robot_description", ['mico_joint_1','mico_joint_2','mico_joint_3','mico_joint_4','mico_joint_5','mico_joint_6'], [3.14,3.14,3.14,3.14,3.14,3.14], "/gazebo")
	print "Setting home posistion"
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
