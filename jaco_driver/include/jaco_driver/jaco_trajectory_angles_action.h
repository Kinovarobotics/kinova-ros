/**
 *  File: jaco_trajectory_angles_action.h
 *  Desc: Class for interfacing with moveIt and the jaco arm.
 *  Auth: Plinio Moreno
 *
 *
 */


#ifndef JACO_DRIVER_JACO_TRAJECTORY_ANGLES_ACTION_H
#define JACO_DRIVER_JACO_TRAJECTORY_ANGLES_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <jaco_msgs/ArmJointAnglesAction.h>
#include <ecl/geometry.hpp>
#include "jaco_driver/jaco_comm.h"

#define NUM_JACO_JOINTS 6
#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0572 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 1.5 //keep the trajectory at a followable speed
#define PI 3.14159265359
#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)
#define KP 300.0
#define KV 20.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success
namespace jaco
{

class JacoTrajectoryAnglesActionServer
{
 public:
    JacoTrajectoryAnglesActionServer(JacoComm &, const ros::NodeHandle &n);
    ~JacoTrajectoryAnglesActionServer();

    void actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

 private:
    ros::NodeHandle node_handle_;
    JacoComm &arm_comm_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;

    ros::Time last_nonstall_time_;
    JacoAngles last_nonstall_angles_;

    // Parameters
    double stall_interval_seconds_;
    double stall_threshold_;
    double rate_hz_;
    float tolerance_;
    double j6o_;
    double        max_curvature_;
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_ANGLES_ACTION_H
