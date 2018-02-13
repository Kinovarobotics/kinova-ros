import kinova_msgs.msg
import geometry_msgs.msg
from kinova_msgs.srv import *
import rospy

def cmd_to_JointTorqueMsg(cmd, n_joints):
	"""
	Returns a JointTorque Kinova msg from an array of torques
	"""
	jointCmd = kinova_msgs.msg.JointTorque()
        if n_joints>=4:
		jointCmd.joint1 = cmd[0][0];
		jointCmd.joint2 = cmd[1][1];
		jointCmd.joint3 = cmd[2][2];
		jointCmd.joint4 = cmd[3][3];
	if n_joints>=6:
		jointCmd.joint5 = cmd[4][4];
		jointCmd.joint6 = cmd[5][5];
	if n_joints==7:
		jointCmd.joint7 = cmd[6][6];
	
	return jointCmd

def cmd_to_JointVelocityMsg(cmd, n_joints):
	"""
	Returns a JointVelocity Kinova msg from an array of velocities
	"""
	jointCmd = kinova_msgs.msg.JointVelocity()
	if n_joints>=4:
		jointCmd.joint1 = cmd[0][0];
		jointCmd.joint2 = cmd[1][1];
		jointCmd.joint3 = cmd[2][2];
		jointCmd.joint4 = cmd[3][3];
	if n_joints>=6:
		jointCmd.joint5 = cmd[4][4];
		jointCmd.joint6 = cmd[5][5];
	if n_joints==7:
		jointCmd.joint7 = cmd[6][6];
	
	return jointCmd


def waypts_to_PoseArrayMsg(cart_waypts):
	"""
	Returns a PoseArray msg from an array of 3D carteian waypoints
	"""
	poseArray = geometry_msgs.msg.PoseArray()
	poseArray.header.stamp = rospy.Time.now()
	poseArray.header.frame_id = "/root"

	for i in range(len(cart_waypts)):
		somePose = geometry_msgs.msg.Pose()
		somePose.position.x = cart_waypts[i][0]
		somePose.position.y = cart_waypts[i][1]
		somePose.position.z = cart_waypts[i][2]

		somePose.orientation.x = 0.0
		somePose.orientation.y = 0.0
		somePose.orientation.z = 0.0
		somePose.orientation.w = 1.0
		poseArray.poses.append(somePose)

	return poseArray
