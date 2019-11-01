/*
 * kinova_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef KINOVA_DRIVER_KINOVA_ARM_H
#define KINOVA_DRIVER_KINOVA_ARM_H

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
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/PoseVelocityWithFingers.h>
#include <kinova_msgs/JointTorque.h>
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/KinovaPose.h>
#include <kinova_msgs/SetForceControlParams.h>
#include <kinova_msgs/SetEndEffectorOffset.h>
#include <kinova_msgs/SetNullSpaceModeState.h>
#include <kinova_msgs/SetTorqueControlMode.h>
#include <kinova_msgs/SetTorqueControlParameters.h>
#include <kinova_msgs/ClearTrajectories.h>
#include <kinova_msgs/AddPoseToCartesianTrajectory.h>
#include <kinova_msgs/ZeroTorques.h>
#include <kinova_msgs/RunCOMParametersEstimation.h>
#include <kinova_msgs/CartesianForce.h>

#include <time.h>
#include <math.h>
#include <vector>

#include "kinova/KinovaTypes.h"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_api.h"


namespace kinova
{

struct robot_info
{
    int id;
    std::string name;
    std::string type;
    std::string serial;
};

class KinovaArm
{
 public:
    KinovaArm(KinovaComm& arm, const ros::NodeHandle &node_handle, const std::string &kinova_robotType, const std::string &kinova_robotName);
    ~KinovaArm();

    //Subscriber callbacks --------------------------------------------------------
    void jointVelocityCallback(const kinova_msgs::JointVelocityConstPtr& joint_vel);
    void cartesianVelocityCallback(const kinova_msgs::PoseVelocityConstPtr& cartesian_vel);
    void cartesianVelocityWithFingersCallback(const kinova_msgs::PoseVelocityWithFingersConstPtr& cartesian_vel_with_fingers);
    void jointTorqueSubscriberCallback(const kinova_msgs::JointTorqueConstPtr& joint_torque);
    void forceSubscriberCallback(const kinova_msgs::CartesianForceConstPtr& force);

    // Service callbacks -----------------------------------------------------------
    bool stopServiceCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res);
    bool startServiceCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res);
    bool homeArmServiceCallback(kinova_msgs::HomeArm::Request &req, kinova_msgs::HomeArm::Response &res);
    bool ActivateNullSpaceModeCallback(kinova_msgs::SetNullSpaceModeState::Request &req,
                                       kinova_msgs::SetNullSpaceModeState::Response &res);
    bool addCartesianPoseToTrajectory(kinova_msgs::AddPoseToCartesianTrajectory::Request &req,
                                kinova_msgs::AddPoseToCartesianTrajectory::Response &res);
    bool clearTrajectoriesServiceCallback(kinova_msgs::ClearTrajectories::Request &req,
                                          kinova_msgs::ClearTrajectories::Response &res);

    bool setEndEffectorOffsetCallback(kinova_msgs::SetEndEffectorOffset::Request& req,
                                      kinova_msgs::SetEndEffectorOffset::Response& res);

    //Torque control
    bool setForceControlParamsCallback(kinova_msgs::SetForceControlParams::Request &req,
                                       kinova_msgs::SetForceControlParams::Response &res);
    bool startForceControlCallback(kinova_msgs::Start::Request &req,
                                   kinova_msgs::Start::Response &res);
    bool stopForceControlCallback(kinova_msgs::Stop::Request &req,
                                  kinova_msgs::Stop::Response &res);

    bool setTorqueControlModeService(kinova_msgs::SetTorqueControlMode::Request &req,
                                     kinova_msgs::SetTorqueControlMode::Response &res);
    bool setTorqueControlParametersService(kinova_msgs::SetTorqueControlParameters::Request &req,
                                           kinova_msgs::SetTorqueControlParameters::Response &res);
    bool setJointTorquesToZeroService(kinova_msgs::ZeroTorques::Request &req,
                                      kinova_msgs::ZeroTorques::Response &res);
    bool runCOMParameterEstimationService(kinova_msgs::RunCOMParametersEstimation::Request &req,
                                          kinova_msgs::RunCOMParametersEstimation::Response &res);

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
    KinovaComm &kinova_comm_;

    // Publishers, subscribers, services
    ros::Subscriber joint_velocity_subscriber_;
    ros::Subscriber cartesian_velocity_subscriber_;
    ros::Subscriber cartesian_velocity_with_fingers_subscriber_;
    ros::Subscriber joint_torque_subscriber_;
    ros::Subscriber cartesian_force_subscriber_;

    ros::Publisher joint_angles_publisher_;
    ros::Publisher tool_position_publisher_;
    ros::Publisher joint_torque_publisher_;
    ros::Publisher tool_wrench_publisher_;
    ros::Publisher finger_position_publisher_;
    ros::Publisher joint_state_publisher_;

    ros::Publisher joint_command_publisher_;
    ros::Publisher cartesian_command_publisher_;

    ros::ServiceServer stop_service_;
    ros::ServiceServer start_service_;
    ros::ServiceServer homing_service_;
    ros::ServiceServer start_null_space_service_;
    ros::ServiceServer add_trajectory_;
    ros::ServiceServer clear_trajectories_;

    ros::ServiceServer set_torque_control_mode_service_;
    ros::ServiceServer set_torque_control_parameters_service_;
    ros::ServiceServer set_actuator_torques_to_zero_;
    ros::ServiceServer set_force_control_params_service_;
    ros::ServiceServer start_force_control_service_;
    ros::ServiceServer stop_force_control_service_;
    ros::ServiceServer run_COM_parameter_estimation_service_;

    ros::ServiceServer set_end_effector_offset_service_;

    // Timers for control loops
    ros::Timer status_timer_;

    // Parameters
    std::string kinova_robotType_;
    std::string kinova_robotName_;
    std::string tf_prefix_;

    char robot_category_;
    int robot_category_version_;
    char wrist_type_;
    int arm_joint_number_;
    char robot_mode_;
    int finger_number_;
    int joint_total_number_;
    ROBOT_TYPE robot_type_;


    double status_interval_seconds_;
    double finger_conv_ratio_;
    bool convert_joint_velocities_;

    // State tracking or utility members
    AngularInfo joint_velocities_;
    float l_joint_torque_[COMMAND_SIZE];
    float l_force_cmd_[COMMAND_SIZE];
    CartesianInfo cartesian_velocities_;

    std::vector< std::string > joint_names_;

    //multiple robots
    int active_robot_id_;
    std::vector<robot_info> robots_;

};


}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_H
