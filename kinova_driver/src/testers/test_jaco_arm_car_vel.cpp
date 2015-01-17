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
#include <kinova_driver/test_jaco_arm_car_vel.h>

using namespace std;


CartesianVelTest::CartesianVelTest(ros::NodeHandle nh,std::string JointVelocity)
{
	 pub = nh.advertise<geometry_msgs::TwistStamped>(JointVelocity,
				2);
	  x_vel = 0;
	  y_vel = 0;
	  z_vel = 0;
	  rx_vel = 0;
	  ry_vel = 0;
	  rz_vel = 0;

		dr_call = boost::bind(&CartesianVelTest::callback,this, _1, _2);
		  dr_server.setCallback(dr_call);
		timer = nh.createTimer(ros::Duration(0.1),&CartesianVelTest::TimerCallback,this);
		//ROS_INFO("Start");

}
void CartesianVelTest::TimerCallback(const ros::TimerEvent&)
{
	geometry_msgs::TwistStamped test_msg;


	test_msg.header.frame_id	= "/jaco_arm_api";
	test_msg.header.stamp	= ros::Time().now();



			test_msg.twist.linear.x = x_vel;
			test_msg.twist.linear.y = y_vel;
			test_msg.twist.linear.z = z_vel;
			test_msg.twist.angular.x= rx_vel;
			test_msg.twist.angular.y = ry_vel;
			test_msg.twist.angular.z = rz_vel;


			pub.publish(test_msg);
		//	ROS_INFO("Timer");


}

void CartesianVelTest::callback(kinova_msgs::JacoVelocityConfig &config, uint32_t level) {

	 x_vel=config.X_Vel;
	 y_vel=config.Y_Vel;
	 z_vel=config.Z_Vel;
	 rx_vel=config.RX_Vel;
	 ry_vel=config.RY_Vel;
	 rz_vel=config.RZ_Vel;

	//	ROS_INFO("Reconfigure");


}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "test_jaco_arm_car_vel");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string JointVelocity("CartesianVelocity"); ///String containing the topic name for cartesian commands

	CartesianVelTest test_node(nh,JointVelocity);


	ros::spin();
}

