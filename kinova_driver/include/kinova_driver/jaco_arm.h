/*
 * jaco_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef JACO_DRIVER_JACO_ARM_H
#define JACO_DRIVER_JACO_ARM_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <kinova_msgs/Stop.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/SetForceControlParams.h>
#include <kinova_msgs/SetEndEffectorOffset.h>

#include <time.h>
#include <math.h>
#include <vector>

#include "kinova/KinovaTypes.h"
#include "kinova_driver/jaco_comm.h"
#include "kinova_driver/jaco_api.h"


namespace kinova
{

// Maximum number of joints on Jaco-like robots:
static const int     JACO_JOINTS_COUNT = 9;

class JacoArm
{
 public:
    JacoArm(JacoComm& arm, const ros::NodeHandle &node_handle);
    ~JacoArm();

    void jointVelocityCallback(const kinova_msgs::JointVelocityConstPtr& joint_vel);
    void cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel);

    bool stopServiceCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res);
    bool startServiceCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res);
    bool homeArmServiceCallback(kinova_msgs::HomeArm::Request &req, kinova_msgs::HomeArm::Response &res);
    
    bool setForceControlParamsCallback(kinova_msgs::SetForceControlParams::Request &req,
                                       kinova_msgs::SetForceControlParams::Response &res);
    bool startForceControlCallback(kinova_msgs::Start::Request &req,
                                   kinova_msgs::Start::Response &res);
    bool stopForceControlCallback(kinova_msgs::Stop::Request &req,
                                  kinova_msgs::Stop::Response &res);

    bool setEndEffectorOffsetCallback(kinova_msgs::SetEndEffectorOffset::Request& req,
                                      kinova_msgs::SetEndEffectorOffset::Response& res);

 private:
    void positionTimer(const ros::TimerEvent&);
    void cartesianVelocityTimer(const ros::TimerEvent&);
    void jointVelocityTimer(const ros::TimerEvent&);
    void statusTimer(const ros::TimerEvent&);

    void publishJointAngles(void);
    void publishToolPosition(void);
    void publishToolWrench(void);
    void publishFingerPosition(void);

    tf::TransformListener tf_listener_;
    ros::NodeHandle node_handle_;
    JacoComm &jaco_comm_;

    // Publishers, subscribers, services
    ros::Subscriber joint_velocity_subscriber_;
    ros::Subscriber cartesian_velocity_subscriber_;

    ros::Publisher joint_angles_publisher_;
    ros::Publisher tool_position_publisher_;
    ros::Publisher tool_wrench_publisher_;
    ros::Publisher finger_position_publisher_;
    ros::Publisher joint_state_publisher_;

    ros::ServiceServer stop_service_;
    ros::ServiceServer start_service_;
    ros::ServiceServer homing_service_;

    ros::ServiceServer set_force_control_params_service_;
    ros::ServiceServer start_force_control_service_;
    ros::ServiceServer stop_force_control_service_;

    ros::ServiceServer set_end_effector_offset_service_;

    // Timers for control loops
    ros::Timer status_timer_;
    ros::Timer cartesian_vel_timer_;
    ros::Timer joint_vel_timer_;

    // Parameters
    double status_interval_seconds_;
    double joint_vel_timeout_seconds_;
    double cartesian_vel_timeout_seconds_;
    double joint_vel_interval_seconds_;
    double cartesian_vel_interval_seconds_;
    std::string tf_prefix_;
    double finger_conv_ratio_;
    bool convert_joint_velocities_;

    // State tracking or utility members
    bool cartesian_vel_timer_flag_;
    bool joint_vel_timer_flag_;

    AngularInfo joint_velocities_;
    CartesianInfo cartesian_velocities_;

    ros::Time last_joint_vel_cmd_time_;
    ros::Time last_cartesian_vel_cmd_time_;

    std::vector< std::string > joint_names_;
};

}  // namespace kinova
#endif  // JACO_DRIVER_JACO_ARM_H
