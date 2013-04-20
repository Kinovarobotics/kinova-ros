/*
 * jaco_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef JACO_ARM_DRIVER_H_
#define JACO_ARM_DRIVER_H_

/* Define to debug without arm */
//#define DEBUG_WITHOUT_ARM
//#define PRINT_DEBUG_INFO
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jaco_driver/jaco_api.h>
#include "jaco_driver/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_driver/KinovaTypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "jaco_driver/joint_velocity.h"
#include "jaco_driver/cartesian_velocity.h"
#include "jaco_driver/joint_angles.h"
#include <jaco_driver/jaco_arm_kinematics.h>
#include <time.h>


namespace jaco {

class JacoArm {
public:
	JacoArm(ros::NodeHandle nh, std::string ArmPose, std::string JointVelocity, std::string JointAngles,std::string CartesianVelocity,std::string ToolPosition);
	void SetAngles(AngularInfo angles, int timeout = 0, bool push = true);
	void SetPosition(CartesianInfo position, int timeout = 0, bool push = true);
	void SetFingers(FingersPosition fingers, int timeout = 0, bool push = true);
	void SetVelocities(AngularInfo joint_vel);
	void SetCartesianVelocities(CartesianInfo velocities);
	void GetAngles(AngularInfo &angles);
	void GetPosition(CartesianInfo &position);
	void GetFingers(FingersPosition &fingers);
	void GetConfig(ClientConfigurations &config);
	void PrintAngles(AngularInfo angles);
	void PrintPosition(CartesianInfo position);
	void PrintFingers(FingersPosition fingers);
	void PrintConfig(ClientConfigurations config);
	void GoHome(void);
	void PoseMSG_Sub(const geometry_msgs::PoseStampedConstPtr& position);
	void CalculatePostion(void);
	void PositionTimer(const ros::TimerEvent&);
	void CartesianVelTimer(const ros::TimerEvent&);
	void JointVelTimer(const ros::TimerEvent&);
	void StatusTimer(const ros::TimerEvent&);
	void VelocityMSG_Sub(const jaco_driver::joint_velocityConstPtr& joint_vel);
	void CartesianVelocityMSG_Sub(	const geometry_msgs::TwistStampedConstPtr&  cartesian_vel);
	void BroadCastAngles(void);
	void BroadCastPosition(void);

private:
	jaco::JacoAPI* API;
	ros::Subscriber ArmPose_sub;
	ros::Subscriber JointVelocity_sub;
	ros::Subscriber CartesianVelocity_sub;
	ros::Publisher JointAngles_pub;
	ros::Publisher ToolPosition_pub;

	ros::Timer status_timer;
	ros::Timer cartesian_vel_timer;
	ros::Timer joint_vel_timer;

	jaco::JacoKinematics kinematics;

	AngularInfo joint_velocities;
	CartesianInfo cartesian_velocities;


ros::Time last_joint_update;
ros::Time last_cartesian_update;



	tf::TransformListener listener;

	ros::Time last_update_time;
	ros::Duration update_time;
};

}

#endif /* JACO_ARM_DRIVER_H_ */
