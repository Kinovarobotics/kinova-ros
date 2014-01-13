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
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <jaco_msgs/JacoVelocityConfig.h>
#include <std_msgs/String.h>





class CartesianVelTest {
public:
	CartesianVelTest(ros::NodeHandle nh, std::string JointVelocity);

private:
	void TimerCallback(const ros::TimerEvent&);

	void callback(jaco_msgs::JacoVelocityConfig &config, uint32_t level);
	ros::Publisher pub;
	ros::Timer timer;
	dynamic_reconfigure::Server<jaco_msgs::JacoVelocityConfig> dr_server;
	dynamic_reconfigure::Server<jaco_msgs::JacoVelocityConfig>::CallbackType dr_call;

	float x_vel ;
	float y_vel ;
	float z_vel ;
	float rx_vel ;
	float ry_vel ;
	float rz_vel ;

};
