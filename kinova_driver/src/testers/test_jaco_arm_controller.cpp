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
#include <kinova_driver/JacoPositionConfig.h>
#include <dynamic_reconfigure/server.h>
using namespace std;



ros::Publisher *pub;

float x_pose=0;
float	 y_pose=0;
float	 z_pose=0;
float	 rx_pose=0;
float	 ry_pose=0;
float	 rz_pose=0;


void callback(kinova_driver::JacoPositionConfig &config, uint32_t level) {

	 x_pose=config.X_Pose;
	 y_pose=config.Y_Pose;
	 z_pose=config.Z_Pose;
	 rx_pose=config.RX_Pose;
	 ry_pose=config.RY_Pose;
	 rz_pose=config.RZ_Pose;

	//	ROS_INFO("Reconfigure");


}



void TimerCallback(const ros::TimerEvent&)
{
	geometry_msgs::PoseStamped test_msg;

			test_msg.pose.position.x = x_pose;
			test_msg.pose.position.y = y_pose;
			test_msg.pose.position.z = z_pose;

			tf::Quaternion q;

			q.setRPY(rx_pose,ry_pose,rz_pose);

			tf::quaternionTFToMsg(q,test_msg.pose.orientation);
			test_msg.header.frame_id = "/arm_base";
			test_msg.header.stamp = ros::Time().now();
			pub->publish(test_msg);

}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_jaco_arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string ArmPose("object_pose"); ///String containing the topic name for cartesian commands

	ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseStamped>(ArmPose,
			2);

	pub = &pub2;
	dynamic_reconfigure::Server<kinova_driver::JacoPositionConfig>  dr_server;
 dynamic_reconfigure::Server<kinova_driver::JacoPositionConfig>::CallbackType	dr_call = boost::bind(&callback, _1, _2);
	dr_server.setCallback(dr_call);
	  ros::Timer	timer = nh.createTimer(ros::Duration(0.1),&TimerCallback);

		ros::spin();

}

