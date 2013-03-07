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
	void GoToPosition(const geometry_msgs::PoseStampedConstPtr& position);
	void GetPostion(void);

private:

	ros::NodeHandle nh_;
	ros::Subscriber sub;
	tf::TransformBroadcaster br;
	jaco_arm::jaco_kinematics kinematics;

//     ros::Publisher vel_pub_;
//   ros::Subscriber joy_sub_;

};

}

#endif /* JACO_ARM_CONTROL_H_ */
