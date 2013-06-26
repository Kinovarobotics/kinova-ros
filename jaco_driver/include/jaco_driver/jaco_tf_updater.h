/*
 * jaco_tf_updater.h
 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#ifndef JACO_TF_UPDATER_H_
#define JACO_TF_UPDATER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "jaco_driver/JointAngles.h"
#include <jaco_driver/jaco_arm_kinematics.h>
#include <time.h>

namespace jaco {


class JacoTFTree {
public:
	JacoTFTree(ros::NodeHandle nh, ros::NodeHandle param_nh);

private:
	void JointAnglesMSG(const jaco_driver::JointAnglesConstPtr& joint_angles);
	void CalculatePostion(void);
	void TFUpdateTimer(const ros::TimerEvent&);


	jaco::JacoKinematics kinematics;
	jaco_driver::JointAngles current_angles;
	ros::Time last_angle_update;
	ros::Subscriber joint_angles_sub;
	ros::Timer tf_update_timer;


};

}

#endif /* JACO_TF_UPDATER_H_ */
