#! /usr/bin/env python
"""A test program to test action servers for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

from robot_control_modules import *

prefix = 'j2s7s300_'
nbJoints = 7
interactive = True
duration_sec = 5

if __name__ == '__main__':
  try:        
		prefix, nbJoints = argumentParser(None)	
		rospy.init_node('test_action_servers')		

		#test joint srv - move robot to 180 pos
		if (interactive == True):        
			nb = raw_input('Moving robot to candle like position, press return, n to skip')
		if (nb != 'n' and nb != 'N'):
			result = joint_position_client([180]*7, prefix)

		#test joint velocity control
		if (interactive == True):        
			nb = raw_input('Testing velocity control, press return, n to skip')
		if (nb != 'n' and nb != 'N'):
			if (nbJoints == 7):
				publishVelCmd([10,0,-10,0,10,0,-10], duration_sec, prefix)
			else:
				publishVelCmd([10,0,0,-10,0,0,0], duration_sec, prefix)
			rospy.sleep(1)

		#test torque control
		if (interactive == True):        
			nb = raw_input('Testing torque control, press return, n to skip')
		if (nb != 'n' and nb != 'N'):		
			publishTorqueCmd([5,0,0,0,0,0,0], duration_sec, prefix)			
			rospy.sleep(1)

		#test cartesian srv
		if (interactive == True):        
			nb = raw_input('Testing Cartesian control, press return, n to skip')
		if (nb != 'n' and nb != 'N'):
		#move robot in joint space to init pose
			if (nbJoints == 7):
							result = joint_position_client([270,220,0,90,180,270,0], prefix)
			else:
				result = joint_position_client([270,220,90,180,270,0,0], prefix)		

		  #set cartesian pose 1
			position = [0.4,-0.5,0]
			quaternion = tf.transformations.quaternion_from_euler(90*3.1415/180, 0, 0,'rxyz')	
			result = cartesian_pose_client(position, quaternion, prefix)	

			#open gripper
			result = gripper_client([0,0,0], prefix)

			#set cartesian pose 2
			position = [0.0,-0.5,0.2]
			quaternion = tf.transformations.quaternion_from_euler(180*3.1415/180, 0, 0,'rxyz')
			result = cartesian_pose_client(position, quaternion, prefix)

			#close gripper
			result = gripper_client([6800,6800,6800], prefix)

		#test cartesian velocity publisher
		if (interactive == True):        
			nb = raw_input('Testing Cartesian velocity control, press return, n to skip')
		if (nb != 'n' and nb != 'N'):
			publishCatesianVelocityCommands([-0.1, 0, -0.1, 0, 0, 0], duration_sec, prefix)
			publishCatesianVelocityCommands([0.1, 0, 0.1, 0, 0, 0], duration_sec, prefix)

    #test cartesian force publisher
		if (interactive == True):
			nb = raw_input('Testing Cartesian force control, press return, n to skip')
		if (nb != 'n' and nb != 'N'):
			publishForceCmd([0, 0, -2, 0, 0, 0], 10, prefix)
			#publishForceCmd([0, 0, 5, 0, 0, 0], 2, prefix)

		#set null space mode
		if (interactive == True):        
			nb = raw_input('Testing Null space control control,active for 10s, use joystick to move robot in Null space,press key')
		if (nb != 'n' and nb != 'N'):
			activateNullSpaceMode(10, prefix)

		print("done!")

  except rospy.ROSInterruptException:
      print "program interrupted before completion"
