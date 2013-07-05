//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================


#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_action.h"

int main(int argc, char **argv)
{
	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_driver");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	jaco::JacoComm comm;

	//create the arm object
	jaco::JacoArm jaco(comm, nh, param_nh);
	jaco::JacoAction action(comm, nh);

	ros::spin();
}

