//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================


#include <jaco_driver/jaco_arm.h>

int main(int argc, char **argv)
{
	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_driver");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	//create the arm object
	jaco::JacoArm jaco(nh, param_nh);

	ros::spin();
}

