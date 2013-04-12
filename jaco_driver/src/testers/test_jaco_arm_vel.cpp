//============================================================================
// Name        : Jaco.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/**
 * @file jaco_arm_control.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <jaco_driver/joint_velocity.h>

using namespace std;
ros::Publisher pub;
void TimerCallback(const ros::TimerEvent&)
{
	jaco_driver::joint_velocity test_msg;

			test_msg.Velocity_J1 = 10.0;
			test_msg.Velocity_J2 = 0;
			test_msg.Velocity_J3 = 0;
			test_msg.Velocity_J4 = 0;
			test_msg.Velocity_J5 = 0;
			test_msg.Velocity_J6 = 0;


			pub.publish(test_msg);


}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_jaco_arm_vel");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string JointVelocity("JointVelocity"); ///String containing the topic name for cartesian commands

	 pub = nh.advertise<jaco_driver::joint_velocity>(JointVelocity,
			2);

	ros::Timer timer = nh.createTimer(ros::Duration(0.1),TimerCallback);


	ros::spin();
}

