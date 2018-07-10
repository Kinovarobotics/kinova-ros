#!/usr/bin/env python

#remove or add the library/libraries for ROS
import rospy
import math

#remove or add the message type
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointVelocity

torque = 0
#define function/functions to provide the required functionaI needlity

def fnc_callbacktool(msg):
	global torque
	torque = msg.effort[0]

if __name__=='__main__':
	#Add here the name of the ROS. In ROS, names are unique named.
	rospy.init_node('example')
	#subscribe to a topic using rospy.Subscriber class
	subtool=rospy.Subscriber('/j2n6s300_driver/out/joint_state', JointState, fnc_callbacktool)
	pub=rospy.Publisher('j2n6s300_driver/in/joint_velocity', JointVelocity, queue_size=10)
	rate=rospy.Rate(100)

	while not rospy.is_shutdown():
		cmd = JointVelocity()
		
		if torque > 0.3:
			cmd.joint6=48
		elif torque < -0.3:
			cmd.joint6=-48
		else:
			cmd.joint6=0
		print cmd
		print torque
			
		pub.publish(cmd)

		rate.sleep()
		
