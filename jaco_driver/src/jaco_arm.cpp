//============================================================================
// Name        : jaco_arm.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================

#include "jaco_driver/jaco_arm.h"
#include <string>
#include <vector>

#define PI 3.14159265359


namespace jaco
{

JacoArm::JacoArm(JacoComm &arm, const ros::NodeHandle &nodeHandle)
    : jaco_comm_(arm), node_handle_(nodeHandle)
{
    /* Set up Services */
    stop_service_ = node_handle_.advertiseService("in/stop", &JacoArm::stopServiceCallback, this);
    start_service_ = node_handle_.advertiseService("in/start", &JacoArm::startServiceCallback, this);
    homing_service_ = node_handle_.advertiseService("in/home_arm", &JacoArm::homeArmServiceCallback, this);

    /* Set up Publishers */
    joint_angles_publisher_ = node_handle_.advertise<jaco_msgs::JointAngles>("out/joint_angles", 2);
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("out/joint_state", 2);
    tool_position_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("out/tool_position", 2);
    tool_wrench_publisher_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("out/tool_wrench", 2);
    finger_position_publisher_ = node_handle_.advertise<jaco_msgs::FingerPosition>("out/finger_position", 2);

    /* Set up Subscribers*/
    joint_velocity_subscriber_ = node_handle_.subscribe("in/joint_velocity", 1,
                                                      &JacoArm::jointVelocityCallback, this);
    cartesian_velocity_subscriber_ = node_handle_.subscribe("in/cartesian_velocity", 1,
                                                          &JacoArm::cartesianVelocityCallback, this);

    node_handle_.param<double>("status_interval_seconds", status_interval_seconds_, 0.1);
    node_handle_.param<double>("joint_angular_vel_timeout", joint_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("cartesian_vel_timeout", cartesian_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("joint_angular_vel_timeout", joint_vel_interval_seconds_, 0.1);
    node_handle_.param<double>("cartesian_vel_timeout", cartesian_vel_interval_seconds_, 0.1);

    node_handle_.param<std::string>("tf_prefix", tf_prefix_, "jaco_");

    // Approximative conversion ratio from finger position (0..6000) to joint angle 
    // in radians (0..0.7).
    node_handle_.param("finger_angle_conv_ratio", finger_conv_ratio_, 0.7 / 5000.0);

    joint_names_.resize(JACO_JOINTS_COUNT);
    joint_names_[0] = tf_prefix_ + "joint_1";
    joint_names_[1] = tf_prefix_ + "joint_2";
    joint_names_[2] = tf_prefix_ + "joint_3";
    joint_names_[3] = tf_prefix_ + "joint_4";
    joint_names_[4] = tf_prefix_ + "joint_5";
    joint_names_[5] = tf_prefix_ + "joint_6";
    joint_names_[6] = tf_prefix_ + "joint_finger_1";
    joint_names_[7] = tf_prefix_ + "joint_finger_2";
    joint_names_[8] = tf_prefix_ + "joint_finger_3";

    status_timer_ = node_handle_.createTimer(ros::Duration(status_interval_seconds_),
                                           &JacoArm::statusTimer, this);

    joint_vel_timer_ = node_handle_.createTimer(ros::Duration(joint_vel_interval_seconds_),
                                              &JacoArm::jointVelocityTimer, this);
    joint_vel_timer_.stop();
    joint_vel_timer_flag_ = false;

    cartesian_vel_timer_ = node_handle_.createTimer(ros::Duration(cartesian_vel_interval_seconds_),
                                                  &JacoArm::cartesianVelocityTimer, this);
    cartesian_vel_timer_.stop();
    cartesian_vel_timer_flag_ = false;

    ROS_INFO("The arm is ready to use.");


}


JacoArm::~JacoArm()
{
}


bool JacoArm::homeArmServiceCallback(jaco_msgs::HomeArm::Request &req, jaco_msgs::HomeArm::Response &res)
{
    jaco_comm_.homeArm();
    jaco_comm_.initFingers();
    res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";
    return true;
}


void JacoArm::jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel)
{
    if (!jaco_comm_.isStopped())
    {
        joint_velocities_.Actuator1 = joint_vel->joint1;
        joint_velocities_.Actuator2 = joint_vel->joint2;
        joint_velocities_.Actuator3 = joint_vel->joint3;
        joint_velocities_.Actuator4 = joint_vel->joint4;
        joint_velocities_.Actuator5 = joint_vel->joint5;
        joint_velocities_.Actuator6 = joint_vel->joint6;
        last_joint_vel_cmd_time_ = ros::Time().now();

        if (joint_vel_timer_flag_ == false)
        {
            joint_vel_timer_.start();
            joint_vel_timer_flag_ = true;
        }
    }
}


/*!
 * \brief Handler for "stop" service.
 *
 * Instantly stops the arm and prevents further movement until start service is
 * invoked.
 */
bool JacoArm::stopServiceCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res)
{
    jaco_comm_.stopAPI();
    res.stop_result = "Arm stopped";
    ROS_DEBUG("Arm stop requested");
    return true;
}


/*!
 * \brief Handler for "start" service.
 *
 * Re-enables control of the arm after a stop.
 */
bool JacoArm::startServiceCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res)
{
    jaco_comm_.startAPI();
    res.start_result = "Arm started";
    ROS_DEBUG("Arm start requested");
    return true;
}


void JacoArm::cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel)
{
    if (!jaco_comm_.isStopped())
    {
        cartesian_velocities_.X = cartesian_vel->twist.linear.x;
        cartesian_velocities_.Y = cartesian_vel->twist.linear.y;
        cartesian_velocities_.Z = cartesian_vel->twist.linear.z;
        cartesian_velocities_.ThetaX = cartesian_vel->twist.angular.x;
        cartesian_velocities_.ThetaY = cartesian_vel->twist.angular.y;
        cartesian_velocities_.ThetaZ = cartesian_vel->twist.angular.z;

        last_cartesian_vel_cmd_time_ = ros::Time().now();

        if (cartesian_vel_timer_flag_ == false)
        {
            cartesian_vel_timer_.start();
            cartesian_vel_timer_flag_ = true;
        }
    }
}


void JacoArm::cartesianVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_cartesian_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > cartesian_vel_timeout_seconds_)
    {
        ROS_DEBUG("Cartesian vel timed out: %f", elapsed_time_seconds);
        cartesian_vel_timer_.stop();
        cartesian_vel_timer_flag_ = false;
    }
    else
    {
        ROS_DEBUG("Cart vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
                  cartesian_velocities_.X, cartesian_velocities_.Y, cartesian_velocities_.Z,
                  cartesian_velocities_.ThetaX, cartesian_velocities_.ThetaY, cartesian_velocities_.ThetaZ);
        jaco_comm_.setCartesianVelocities(cartesian_velocities_);
    }
}


void JacoArm::jointVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_joint_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > joint_vel_timeout_seconds_)
    {
        ROS_DEBUG("Joint vel timed out: %f", elapsed_time_seconds);
        joint_vel_timer_.stop();
        joint_vel_timer_flag_ = false;
    }
    else
    {
        ROS_DEBUG("Joint vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
                  joint_velocities_.Actuator1, joint_velocities_.Actuator2, joint_velocities_.Actuator3,
                  joint_velocities_.Actuator4, joint_velocities_.Actuator5, joint_velocities_.Actuator6);
        jaco_comm_.setJointVelocities(joint_velocities_);
    }
}


/*!
 * \brief Publishes the current joint angles.
 *
 * Joint angles are published in both their raw state as obtained from the arm
 * (JointAngles), and transformed & converted to radians (joint_state) as per
 * the Jaco Kinematics PDF.
 *
 * Velocities and torques (effort) are only published in the JointStates
 * message, only for the first 6 joints as these values are not available for
 * the fingers.
 */
void JacoArm::publishJointAngles(void)
{
    static const double PI_180 = (PI / 180.0);
    FingerAngles fingers;
    jaco_comm_.getFingerPositions(fingers);

    // Query arm for current joint angles
    JacoAngles current_angles;
    jaco_comm_.getJointAngles(current_angles);
    jaco_msgs::JointAngles jaco_angles = current_angles.constructAnglesMsg();

    jaco_angles.joint1 = current_angles.Actuator1;
    jaco_angles.joint2 = current_angles.Actuator2;
    jaco_angles.joint3 = current_angles.Actuator3;
    jaco_angles.joint4 = current_angles.Actuator4;
    jaco_angles.joint5 = current_angles.Actuator5;
    jaco_angles.joint6 = current_angles.Actuator6;

    sensor_msgs::JointState joint_state;
    joint_state.name = joint_names_;
    joint_state.header.stamp = ros::Time::now();

    // Transform from Kinova DH algorithm to physical angles in radians, then place into vector array
    joint_state.position.resize(9);

    joint_state.position[0] = (180- jaco_angles.joint1) * (PI / 180);
    joint_state.position[1] = (jaco_angles.joint2 - 270) * (PI / 180);
    joint_state.position[2] = (90-jaco_angles.joint3) * (PI / 180);
    joint_state.position[3] = (180-jaco_angles.joint4) * (PI / 180);
    joint_state.position[4] = (180-jaco_angles.joint5) * (PI / 180);
    joint_state.position[5] = (260-jaco_angles.joint6) * (PI / 180);
    joint_state.position[6] = finger_conv_ratio_ * fingers.Finger1;
    joint_state.position[7] = finger_conv_ratio_ * fingers.Finger2;
    joint_state.position[8] = finger_conv_ratio_ * fingers.Finger3;

    // Joint velocities
    JacoAngles current_vels;
    jaco_comm_.getJointVelocities(current_vels);
    joint_state.velocity.resize(9);
    joint_state.velocity[0] = current_vels.Actuator1;
    joint_state.velocity[1] = current_vels.Actuator2;
    joint_state.velocity[2] = current_vels.Actuator3;
    joint_state.velocity[3] = current_vels.Actuator4;
    joint_state.velocity[4] = current_vels.Actuator5;
    joint_state.velocity[5] = current_vels.Actuator6;
    for (int i = 0; i < 6; ++i) {
        // Angle velocities from the API are 0..180 for positive values,
        // and 360..181 for negative ones, in a kind of 2-complement setup.
        double& qd = joint_state.velocity[i];
        if (qd > 180.0) {
            qd -= 360.0;
        }
        qd *= PI_180;
    }
    // No velocity for the fingers:
    joint_state.velocity[6] = 0.0;
    joint_state.velocity[7] = 0.0;
    joint_state.velocity[8] = 0.0;

    // Joint torques (effort)
    JacoAngles joint_tqs;
    joint_state.effort.resize(9);
    joint_state.effort[0] = joint_tqs.Actuator1;
    joint_state.effort[1] = joint_tqs.Actuator2;
    joint_state.effort[2] = joint_tqs.Actuator3;
    joint_state.effort[3] = joint_tqs.Actuator4;
    joint_state.effort[4] = joint_tqs.Actuator5;
    joint_state.effort[5] = joint_tqs.Actuator6;
    joint_state.effort[6] = 0.0;
    joint_state.effort[6] = 0.0;
    joint_state.effort[8] = 0.0;

    joint_angles_publisher_.publish(jaco_angles);
    joint_state_publisher_.publish(joint_state);
}


/*!
 * \brief Publishes the current cartesian coordinates
 */
void JacoArm::publishToolPosition(void)
{
    JacoPose pose;
    geometry_msgs::PoseStamped current_position;

    jaco_comm_.getCartesianPosition(pose);

    current_position.pose            = pose.constructPoseMsg();
    current_position.header.stamp    = ros::Time::now();
    current_position.header.frame_id = tf_prefix_ + "link_base";

    tool_position_publisher_.publish(current_position);
}

/*!
 * \brief Publishes the current cartesian forces at the end effector. 
 */
void JacoArm::publishToolWrench(void)
{
    JacoPose wrench;
    geometry_msgs::WrenchStamped current_wrench;

    jaco_comm_.getCartesianForce(wrench);

    current_wrench.wrench          = wrench.constructWrenchMsg();
    current_wrench.header.stamp    = ros::Time::now();
    // TODO: Rotate wrench to fit the end effector frame.
    // Right now, the orientation of the wrench is in the API's (base) frame.
    current_wrench.header.frame_id = tf_prefix_ + "api_origin";

    tool_wrench_publisher_.publish(current_wrench);
}

/*!
 * \brief Publishes the current finger positions.
 */
void JacoArm::publishFingerPosition(void)
{
    FingerAngles fingers;
    jaco_comm_.getFingerPositions(fingers);
    finger_position_publisher_.publish(fingers.constructFingersMsg());
}


void JacoArm::statusTimer(const ros::TimerEvent&)
{
    publishJointAngles();
    publishToolPosition();
    publishToolWrench();
    publishFingerPosition();
}

}  // namespace jaco
