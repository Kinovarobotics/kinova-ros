/*
 * jaco_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef JACO_ARM_DRIVER_H_
#define JACO_ARM_DRIVER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jaco_driver/jaco_api.h>
#include "jaco_driver/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_driver/KinovaTypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "jaco_driver/JointVelocity.h"
#include "jaco_driver/FingerPosition.h"
#include "jaco_driver/JointAngles.h"
#include "jaco_driver/ZeroArm.h"
#include "sensor_msgs/JointState.h"

#include <jaco_driver/SoftwareStop.h>

#include <jaco_driver/Stop.h>
#include <jaco_driver/Start.h>
#include <jaco_driver/HomeArm.h>

#include <time.h>
#include <math.h>
#include <vector>

#define PI 3.14159265358

namespace jaco
{
	class JacoArm
	{
		public:
			JacoArm(ros::NodeHandle nh, ros::NodeHandle param_nh);
			bool HomeState(void);
			void ZeroArm(void);
			void SetAngles(AngularInfo angles, int timeout = 0, bool push = true);
			void SetPosition(CartesianInfo position, int timeout = 0, bool push = true);
			void SetFingers(FingersPosition fingers, int timeout = 0, bool push = true);
			void SetVelocities(AngularInfo joint_vel);
			void SetCartesianVelocities(CartesianInfo velocities);
			void SetConfig(ClientConfigurations config);
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
			void VelocityMSG(const jaco_driver::JointVelocityConstPtr& joint_vel);
			void ZeroArmMSG(const jaco_driver::ZeroArmConstPtr& zero);

			void CartesianVelocityMSG(const geometry_msgs::TwistStampedConstPtr& cartesian_vel);
			void SetFingerPositionMSG(const jaco_driver::FingerPositionConstPtr& finger_pos);
			void BroadCastAngles(void);
			void BroadCastPosition(void);
			void BroadCastFingerPosition(void);
			void SoftwarePauseMSG(const jaco_driver::SoftwareStopConstPtr& software_pause);
			void SetJointAnglesMSG(const jaco_driver::JointAnglesConstPtr& angles);

			bool StopSRV(jaco_driver::Stop::Request &req, jaco_driver::Stop::Response &res);
			bool StartSRV(jaco_driver::Start::Request &req, jaco_driver::Start::Response &res);
			bool HomeArmSRV(jaco_driver::HomeArm::Request &req, jaco_driver::HomeArm::Response &res);
		private:
			jaco::JacoAPI* API;
			/* Subscribers */
			ros::Subscriber ArmPose_sub;
			ros::Subscriber JointVelocity_sub;
			ros::Subscriber CartesianVelocity_sub;
			ros::Subscriber SetFingerPosition_sub;
			ros::Subscriber SoftwarePause_sub;
			ros::Subscriber SetJoint_sub;
			ros::Subscriber ZeroArm_sub;


			/* Publishers */
			ros::Publisher JointAngles_pub;
			ros::Publisher ToolPosition_pub;
			ros::Publisher FingerPosition_pub;
			ros::Publisher ZeroArm_pub;
			ros::Publisher JointState_pub;

			/* Services */
			ros::ServiceServer stop_service;
			ros::ServiceServer start_service;
			ros::ServiceServer homing_service;

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
			uint8_t previous_state;

			bool software_pause;
	};

}

#endif /* JACO_ARM_DRIVER_H_ */
