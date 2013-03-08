/*
 * jaco_arm_control.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mdedonato
 */

#ifndef JACO_ARM_CONTROL_H_
#define JACO_ARM_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jaco_driver/jacolib.h>
#include "jaco_driver/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_driver/KinovaTypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <jaco_driver/jaco_arm.h>

/* Define to debug without arm */
//#define DEBUG_WITHOUT_ARM
namespace jaco_arm {

class JacoArm {
public:
	JacoArm(ros::NodeHandle nh, std::string ArmPose);
	void SetAngles(AngularInfo angles);
	void SetPosition(CartesianInfo position);
	void SetFingers(FingersPosition fingers);
	void GetAngles(AngularInfo &angles);
	void GetPosition(CartesianInfo &position);
	void GetFingers(FingersPosition &fingers);
	void PrintAngles(AngularInfo angles);
	void PrintPosition(CartesianInfo position);
	void PrintFingers(FingersPosition fingers);
	void GoHome(void);
	void PoseMSG_Sub(const geometry_msgs::PoseStampedConstPtr& position);
	void CalculatePostion(void);
	void TimerCallback(const ros::TimerEvent&);

private:

	ros::NodeHandle nh_;
	ros::Subscriber sub;
	tf::TransformBroadcaster br;
	jaco_arm::jaco_kinematics kinematics;
	ros::Timer timer;
};

}

#endif /* JACO_ARM_CONTROL_H_ */
