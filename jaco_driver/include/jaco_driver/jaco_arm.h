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
#include "jaco_driver/jaco_kinematic_controller.h"
#include "jaco_driver/jaco_mpc_controller.h"


namespace jaco
{

// Maximum number of joints on Jaco-like robots:
static const int     JACO_JOINTS_COUNT = 9;

class JacoArm
{
 public:
    JacoArm(JacoComm& arm, const ros::NodeHandle &node_handle, JacoKinematicController& jaco_controller);
    //JacoArm(JacoComm& arm, const ros::NodeHandle &node_handle, JacoMPCController& jaco_controller);
    ~JacoArm();

    void jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel);
    void cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel);
    void fingerVelocityCallback(const jaco_msgs::FingerPositionConstPtr& finger_vel);
    
    void jointPositionCallback(const jaco_msgs::JointAnglesConstPtr& joint_pos);
    void cartesianPositionCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_pos);

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
    void cartesianPositionTimer(const ros::TimerEvent&);
    void jointPositionTimer(const ros::TimerEvent&);
    void statusTimer(const ros::TimerEvent&);
    void fingerVelocityTimer(const ros::TimerEvent&);
    void publishJointAngles(void);
    void publishToolPosition(void);
    void publishToolWrench(void);
    void publishFingerPosition(void);

    tf::TransformListener tf_listener_;
    ros::NodeHandle node_handle_;
    JacoComm &jaco_comm_;
    JacoKinematicController &jaco_controller_;
    //JacoMPCController &jaco_controller_;

    // Publishers, subscribers, services
    ros::Subscriber joint_velocity_subscriber_;
    ros::Subscriber cartesian_velocity_subscriber_;
    ros::Subscriber finger_velocity_subscriber_;
    ros::Subscriber joint_position_subscriber_;
    ros::Subscriber cartesian_position_subscriber_;

    ros::Publisher joint_angles_publisher_;
    ros::Publisher tool_position_publisher_;
    ros::Publisher tool_wrench_publisher_;
    ros::Publisher finger_position_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher joint_position_state_publisher_;

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
    ros::Timer finger_vel_timer_;
    ros::Timer cartesian_pos_timer_;
    ros::Timer joint_pos_timer_;

    // Parameters
    double status_interval_seconds_;
    
    double joint_vel_timeout_seconds_;
    double cartesian_vel_timeout_seconds_;
    double finger_vel_timeout_seconds_;
    double joint_pos_timeout_seconds_;
    double cartesian_pos_timeout_seconds_;
    
    
    double joint_vel_interval_seconds_;
    double cartesian_vel_interval_seconds_;
    double finger_vel_interval_seconds_;
    double joint_pos_interval_seconds_;
    double cartesian_pos_interval_seconds_;
        
    std::string tf_prefix_;
    double finger_conv_ratio_;
    bool convert_joint_velocities_;

    // State tracking or utility members
    bool joint_vel_timer_flag_;
    bool cartesian_vel_timer_flag_;
    bool finger_vel_timer_flag_;
    bool joint_pos_timer_flag_;
    bool cartesian_pos_timer_flag_;

    AngularInfo joint_velocities_;
    CartesianInfo cartesian_velocities_;
    FingerAngles finger_velocities_;
    JacoAngles joint_position_;
    JacoPose cartesian_position_;


    ros::Time last_joint_vel_cmd_time_;
    ros::Time last_cartesian_vel_cmd_time_;
    ros::Time last_finger_vel_cmd_time_;
    ros::Time last_joint_pos_cmd_time_;
    ros::Time last_cartesian_pos_cmd_time_;   

    std::vector< std::string > joint_names_;
    std::vector< std::string > joint_urdf_names_;
    
    bool use_kinematic_controller_;
    bool null_space_controller_;
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_ARM_H
