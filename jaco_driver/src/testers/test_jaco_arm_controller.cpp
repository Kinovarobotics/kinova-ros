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

using namespace std;




int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_jaco_arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string ArmPose("ObjectPose"); ///String containing the topic name for cartesian commands

	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(ArmPose,
			2);

	while (ros::ok()) {
		double x, y, z, rx, ry, rz;
		geometry_msgs::PoseStamped test_msg;

		std::cout << "Enter X:";
		std::cin >> x;
		std::cout << "Enter Y:";
		std::cin >> y;
		std::cout << "Enter Z:";
		std::cin >> z;
		std::cout << "Enter RX:";
		std::cin >> rx;
		std::cout << "Enter RY:";
		std::cin >> ry;
		std::cout << "Enter RZ:";
		std::cin >> rz;

		test_msg.pose.position.x = x;
		test_msg.pose.position.y = y;
		test_msg.pose.position.z = z;

		tf::Quaternion q;

		q.setRPY(rx,ry,rz);

		tf::quaternionTFToMsg(q,test_msg.pose.orientation);
		test_msg.header.frame_id = "/arm_base";
		pub.publish(test_msg);
		ros::spinOnce();

	}


}

