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
#include <jaco_arm/jacolib.h>
#include "jaco_arm/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_arm/KinovaTypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>


namespace jaco_arm {

class JacoArm {
public:
	JacoArm();
	void GoToPosition(const geometry_msgs::PoseStampedConstPtr& position);


private:

	ros::NodeHandle nh_;
	ros::Subscriber position_sub;		///Publisher for sending xyz_position messages to controller


//     ros::Publisher vel_pub_;
//   ros::Subscriber joy_sub_;

};

}

#endif /* JACO_ARM_CONTROL_H_ */
