/*
 * jaco_tf_updater.h
 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#ifndef JACO_DRIVER_JACO_TF_UPDATER_H
#define JACO_DRIVER_JACO_TF_UPDATER_H

#include <time.h>

#include <jaco_driver/jaco_arm_kinematics.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "jaco_msgs/JointAngles.h"


namespace jaco
{

class JacoTFTree
{
 public:
    explicit JacoTFTree(ros::NodeHandle nh);

 private:
    void jointAnglesMsgHandler(const jaco_msgs::JointAnglesConstPtr& joint_angles);
    void calculatePostion(void);
    void tfUpdateHandler(const ros::TimerEvent&);

    jaco::JacoKinematics kinematics_;
    jaco_msgs::JointAngles current_angles_;
    ros::Time last_angle_update_;
    ros::Subscriber joint_angles_subscriber_;
    ros::Timer tf_update_timer_;
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_TF_UPDATER_H
