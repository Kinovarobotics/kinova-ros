/**
 *  File: jaco_trajectory_angles_action.h
 *  Desc: Class for interfacing moveIt with the jaco arm's gripper.
 *  Auth: Plinio Moreno
 *  email: plinio@isr.tecnico.ulisboa.pt
 *
 *
 */


#ifndef JACO_DRIVER_JACO_GRIPPER_ACTION_SERVER_H
#define JACO_DRIVER_JACO_GRIPPER_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <jaco_msgs/ArmJointAnglesAction.h>
#include "jaco_driver/jaco_comm.h"

namespace jaco
{

class JacoGripperActionServer
{
 public:
    JacoGripperActionServer(JacoComm &, const ros::NodeHandle &n);
    ~JacoGripperActionServer();

    void actionCallback(const control_msgs::GripperCommandGoalConstPtr &goal);

 private:
    ros::NodeHandle node_handle_;
    JacoComm &arm_comm_;
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;

    ros::Time last_nonstall_time_;
    jaco::FingerAngles last_nonstall_finger_positions_;

    // Parameters
    double stall_interval_seconds_;
    double stall_threshold_;
    double rate_hz_;
    float tolerance_;
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_ANGLES_ACTION_H
