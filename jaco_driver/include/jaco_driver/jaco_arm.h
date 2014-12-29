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

#include <jaco_msgs/Stop.h>
#include <jaco_msgs/Start.h>
#include <jaco_msgs/HomeArm.h>
#include <jaco_msgs/JointVelocity.h>
#include <jaco_msgs/FingerPosition.h>
#include <jaco_msgs/JointAngles.h>
#include <jaco_msgs/SetForceControlParams.h>

#include <time.h>
#include <math.h>
#include <vector>

#include "kinova/KinovaTypes.h"
#include "jaco_driver/jaco_comm.h"
#include "jaco_driver/jaco_api.h"


namespace jaco
{

// Maximum number of joints on Jaco-like robots:
static const int     JACO_JOINTS_COUNT = 9;

class JacoArm
{
 public:
    JacoArm(JacoComm& arm, const ros::NodeHandle &node_handle);
    ~JacoArm();

    void jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel);
    void cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel);

    bool stopServiceCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res);
    bool startServiceCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res);
    bool homeArmServiceCallback(jaco_msgs::HomeArm::Request &req, jaco_msgs::HomeArm::Response &res);
    
    bool setForceControlParamsCallback(jaco_msgs::SetForceControlParams::Request &req,
                                       jaco_msgs::SetForceControlParams::Response &res);
    bool startForceControlCallback(jaco_msgs::Start::Request &req,
                                   jaco_msgs::Start::Response &res);
    bool stopForceControlCallback(jaco_msgs::Stop::Request &req,
                                  jaco_msgs::Stop::Response &res);

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

    // State tracking or utility members
    bool cartesian_vel_timer_flag_;
    bool joint_vel_timer_flag_;

    AngularInfo joint_velocities_;
    CartesianInfo cartesian_velocities_;

    ros::Time last_joint_vel_cmd_time_;
    ros::Time last_cartesian_vel_cmd_time_;

    std::vector< std::string > joint_names_;
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_ARM_H
