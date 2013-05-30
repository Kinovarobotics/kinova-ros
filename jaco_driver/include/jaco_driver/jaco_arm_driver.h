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
#include "jaco_driver/finger_position.h"
#include "jaco_driver/joint_angles.h"
#include <robot_base_msgs/SoftwareStop.h>

#include <time.h>

namespace jaco
{

	class JacoArm
	{
		public:
			JacoArm(ros::NodeHandle nh, ros::NodeHandle param_nh);
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
			void VelocityMSG(const jaco_driver::joint_velocityConstPtr& joint_vel);
			void CartesianVelocityMSG(const geometry_msgs::TwistStampedConstPtr& cartesian_vel);
			void SetFingerPositionMSG(const jaco_driver::finger_positionConstPtr& finger_pos);
			void BroadCastAngles(void);
			void BroadCastPosition(void);
			void BroadCastFingerPosition(void);
			void SoftwarePauseMSG(const robot_base_msgs::SoftwareStopConstPtr& software_pause);

		private:
			jaco::JacoAPI* API;
			/* Subscribers */
			ros::Subscriber ArmPose_sub;
			ros::Subscriber JointVelocity_sub;
			ros::Subscriber CartesianVelocity_sub;
			ros::Subscriber SetFingerPosition_sub;
			ros::Subscriber SoftwarePause_sub;
			/* Publishers */
			ros::Publisher JointAngles_pub;
			ros::Publisher ToolPosition_pub;
			ros::Publisher FingerPosition_pub;

			ros::Timer status_timer;
			ros::Timer cartesian_vel_timer;
			ros::Timer joint_vel_timer;
			bool cartesian_vel_timer_flag;
			bool joint_vel_timer_flag;

			AngularInfo joint_velocities;
			CartesianInfo cartesian_velocities;

			ros::Time last_joint_update;
			ros::Time last_cartesian_update;

			tf::TransformListener listener;

			ros::Time last_update_time;
			ros::Duration update_time;

			/* Remove this */
			tf::TransformListener tf_listener;

			bool software_pause;
	};

}

#endif /* JACO_ARM_DRIVER_H_ */
