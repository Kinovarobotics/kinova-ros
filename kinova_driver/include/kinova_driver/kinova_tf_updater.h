/*
 * kinova_tf_updater.h
 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#ifndef KINOVA_DRIVER_KINOVA_TF_UPDATER_H
#define KINOVA_DRIVER_KINOVA_TF_UPDATER_H

#include <time.h>

#include <kinova_driver/kinova_arm_kinematics.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "kinova_msgs/JointAngles.h"


namespace kinova
{

class KinovaTFTree
{
 public:
    explicit KinovaTFTree(ros::NodeHandle nh, std::string& kinova_robotType);

 private:
    void jointAnglesMsgHandler(const kinova_msgs::JointAnglesConstPtr& joint_angles);
    void calculatePostion(void);
    void tfUpdateHandler(const ros::TimerEvent&);

    kinova::KinovaKinematics kinematics_;
    kinova_msgs::JointAngles current_angles_;
    ros::Time last_angle_update_;
    ros::Subscriber joint_angles_subscriber_;
    ros::Timer tf_update_timer_;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_TF_UPDATER_H
