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
#include <kinova_msgs/cartesian_velocity.h>

using namespace std;
ros::Publisher pub;
void TimerCallback(const ros::TimerEvent&)
{
	kinova_msgs::cartesian_velocity test_msg;

			test_msg.Velocity_X = 0.0;
			test_msg.Velocity_Y = 10;
			test_msg.Velocity_Z = 0;
			test_msg.Velocity_TX = 0;
			test_msg.Velocity_TY = 0;
			test_msg.Velocity_TZ = 0;


			pub.publish(test_msg);


}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_jaco_arm_vel");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string JointVelocity("CartesianVelocity"); ///String containing the topic name for cartesian commands

	 pub = nh.advertise<kinova_msgs::cartesian_velocity>(JointVelocity,
			2);

	ros::Timer timer = nh.createTimer(ros::Duration(0.01),TimerCallback);


	ros::spin();
}

