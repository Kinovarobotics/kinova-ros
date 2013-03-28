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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <jaco_driver/jaco_arm_kinematics.h>
#include <time.h>



namespace jaco {

class JacoArm {
public:
	JacoArm(ros::NodeHandle nh, std::string ArmPose);
	void SetAngles(AngularInfo angles,  int timeout = 0, bool push = true);
	void SetPosition(CartesianInfo position,  int timeout = 0, bool push = true);
	void SetFingers(FingersPosition fingers,  int timeout = 0, bool push = true);
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
	jaco::JacoAPI* API;
	jaco::JacoKinematics kinematics;
	ros::NodeHandle nh_;
	ros::Subscriber sub;
	ros::Timer timer;

	tf::TransformListener listener;

	ros::Time last_update_time;
	ros::Duration update_time;
};

}

#endif /* JACO_ARM_DRIVER_H_ */
