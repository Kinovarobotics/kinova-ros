//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================


#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"

int main(int argc, char **argv)
{
	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_driver");
	ros::NodeHandle nh;
	//ros::NodeHandle param_nh("~");

	jaco::JacoComm comm;

	ROS_INFO("Initializing the Arm");

	comm.HomeArm();
	comm.InitializeFingers();

	//create the arm object
	jaco::JacoArm jaco(comm, nh);
	jaco::JacoPoseActionServer pose_server(comm, nh);
	jaco::JacoAnglesActionServer angles_server(comm, nh);
	jaco::JacoFingersActionServer fingers_server(comm, nh);

	ros::spin();
}

