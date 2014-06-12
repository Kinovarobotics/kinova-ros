//============================================================================
// Name        : jaco_arm.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================

#include "jaco_driver/jaco_arm.h"


namespace jaco {

JacoArm::JacoArm(JacoComm &arm, ros::NodeHandle &nodeHandle)
    : jaco_api_(arm), nodeHandle_(nodeHandle)
{
    ROS_INFO("Starting Up Jaco Arm Controller...");

    QuickStatus qs;
    ClientConfigurations configuration;
    jaco_api_.getQuickStatus(qs);
    jaco_api_.getConfig(configuration);
    jaco_api_.getQuickStatus(qs);
    jaco_api_.printConfig(configuration);
    jaco_api_.getQuickStatus(qs);

    /* Set up Services */
    stop_service_ = nodeHandle_.advertiseService("in/stop", &JacoArm::stopServiceCallback, this);
    start_service_ = nodeHandle_.advertiseService("in/start", &JacoArm::startServiceCallback, this);
    homing_service_ = nodeHandle_.advertiseService("in/home_arm", &JacoArm::homeArmServiceCallback, this);

    /* Set up Publishers */
    joint_angles_publisher_ = nodeHandle_.advertise<jaco_msgs::JointAngles>("out/joint_angles", 2);
    joint_state_publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("out/joint_state", 2);
    tool_position_publisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("out/tool_position", 2);
    finger_position_publisher_ = nodeHandle_.advertise<jaco_msgs::FingerPosition>("out/finger_position", 2);

    /* Set up Subscribers*/
    joint_velocity_subscriber_ = nodeHandle_.subscribe("in/joint_velocity", 1,
                                                      &JacoArm::jointVelocityCallback, this);
    cartesian_velocity_subscriber_ = nodeHandle_.subscribe("in/cartesian_velocity", 1,
                                                          &JacoArm::cartesianVelocityCallback, this);

    // TODO: Change these defaults to something safer
    nodeHandle_.param<double>("status_interval_seconds", status_interval_seconds_, 1.0);
    nodeHandle_.param<double>("joint_angular_vel_timeout", joint_angular_vel_timeout_seconds_, 1.0);
    nodeHandle_.param<double>("cartesian_vel_timeout", cartesian_vel_timeout_seconds_, 1.0);

    status_timer_ = nodeHandle_.createTimer(ros::Duration(status_interval_seconds_),
                                           &JacoArm::statusTimer, this);

    joint_vel_timer_ = nodeHandle_.createTimer(ros::Duration(joint_angular_vel_timeout_seconds_),
                                              &JacoArm::jointVelocityTimer, this);
    joint_vel_timer_.stop();
    joint_vel_timer_flag_ = false;

    cartesian_vel_timer_ = nodeHandle_.createTimer(ros::Duration(cartesian_vel_timeout_seconds_),
                                                  &JacoArm::cartesianVelocityTimer, this);
    cartesian_vel_timer_.stop();
    cartesian_vel_timer_flag_ = false;


    jaco_api_.getQuickStatus(qs);
    // Set the angular velocity of each of the joints to zero
    TrajectoryPoint jaco_velocity;
    memset(&jaco_velocity, 0, sizeof(jaco_velocity));
    jaco_api_.setCartesianVelocities(jaco_velocity.Position.CartesianPosition);

    jaco_api_.getQuickStatus(qs);

    ROS_INFO("The arm is ready to use.");
}

JacoArm::~JacoArm() {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
}

bool JacoArm::homeArmServiceCallback(jaco_msgs::HomeArm::Request &req, jaco_msgs::HomeArm::Response &res) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    jaco_api_.homeArm();
    res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";

    return true;
}

void JacoArm::jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    if (!jaco_api_.isStopped()) {
        joint_velocities_.Actuator1 = joint_vel->joint1;
        joint_velocities_.Actuator2 = joint_vel->joint2;
        joint_velocities_.Actuator3 = joint_vel->joint3;
        joint_velocities_.Actuator4 = joint_vel->joint4;
        joint_velocities_.Actuator5 = joint_vel->joint5;
        joint_velocities_.Actuator6 = joint_vel->joint6;
        last_joint_vel_cmd_time_ = ros::Time().now();

        if (joint_vel_timer_flag_ == false) {
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
bool JacoArm::stopServiceCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    jaco_api_.stop();
    res.stop_result = "JACO ARM HAS BEEN STOPPED";
    ROS_DEBUG("JACO ARM STOP REQUEST");

    return true;
}

/*!
 * \brief Handler for "start" service.
 *
 * Re-enables control of the arm after a stop.
 */
bool JacoArm::startServiceCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    jaco_api_.start();
    res.start_result = "JACO ARM CONTROL HAS BEEN ENABLED";
    ROS_INFO("JACO ARM START REQUEST");

    return true;
}


void JacoArm::cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    if (!jaco_api_.isStopped()) {
        cartesian_velocities_.X = cartesian_vel->twist.linear.x;
        cartesian_velocities_.Y = cartesian_vel->twist.linear.y;
        cartesian_velocities_.Z = cartesian_vel->twist.linear.z;
        cartesian_velocities_.ThetaX = cartesian_vel->twist.angular.x;
        cartesian_velocities_.ThetaY = cartesian_vel->twist.angular.y;
        cartesian_velocities_.ThetaZ = cartesian_vel->twist.angular.z;

        last_cartesian_vel_cmd_time_ = ros::Time().now();

        if (cartesian_vel_timer_flag_ == false) {
            cartesian_vel_timer_.start();
            cartesian_vel_timer_flag_ = true;
        }
    }
}

void JacoArm::cartesianVelocityTimer(const ros::TimerEvent&) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    jaco_api_.setCartesianVelocities(cartesian_velocities_);

    if ((ros::Time().now().toSec() - last_cartesian_vel_cmd_time_.toSec()) > 1) {
        cartesian_vel_timer_.stop();
        cartesian_vel_timer_flag_ = false;
    }
}

void JacoArm::jointVelocityTimer(const ros::TimerEvent&){
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    jaco_api_.setVelocities(joint_velocities_);

    if ((ros::Time().now().toSec() - last_joint_vel_cmd_time_.toSec()) > 1) {
        joint_vel_timer_.stop();
        joint_vel_timer_flag_ = false;
    }
}

///*!
// * \brief Contains coordinates for an alternate "Home" position
// *
// * GoHome() function must be enabled in the initialization routine for this to
// * work.
// */
//void JacoArm::goHome(void) {
//    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
///*
//    AngularInfo joint_home;

//    joint_home.Actuator1 = 176.0;
//    joint_home.Actuator2 = 111.0;
//    joint_home.Actuator3 = 107.0;
//    joint_home.Actuator4 = 459.0;
//    joint_home.Actuator5 = 102.0;
//    joint_home.Actuator6 = 106.0;

//    SetAngles(joint_home, 10); //send joints to home position

//    API->SetCartesianControl();
//*/
//}

/*!
 * \brief Publishes the current joint angles.
 *
 * Joint angles are published in both their raw state as obtained from the arm
 * (JointAngles), and transformed & converted to radians (joint_state) as per
 * the Jaco Kinematics PDF.
 *
 * JointState will eventually also publish the velocity and effort for each
 * joint, when this data is made available by the C++ API.  Currenty velocity
 * and effort are reported as being zero (0.0) for all joints.
 */
void JacoArm::publishJointAngles(void) {
//    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);

    // Query arm for current joint angles
    JacoAngles current_angles;
    jaco_api_.getJointAngles(current_angles);
    jaco_msgs::JointAngles jaco_angles = current_angles.constructAnglesMsg();

    jaco_angles.joint1 = current_angles.Actuator1;
    jaco_angles.joint2 = current_angles.Actuator2;
    jaco_angles.joint3 = current_angles.Actuator3;
    jaco_angles.joint4 = current_angles.Actuator4;
    jaco_angles.joint5 = current_angles.Actuator5;
    jaco_angles.joint6 = current_angles.Actuator6;

    // TODO: change the joint prefix into a parameter
    sensor_msgs::JointState joint_state;
    const char* nameArgs[] = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
    std::vector<std::string> joint_names(nameArgs, nameArgs + 6);
    joint_state.name = joint_names;

    // Transform from Kinova DH algorithm to physical angles in radians, then place into vector array
    joint_state.position.resize(6);
    joint_state.position[0] = jaco_angles.joint1;
    joint_state.position[1] = jaco_angles.joint2;
    joint_state.position[2] = jaco_angles.joint3;
    joint_state.position[3] = jaco_angles.joint4;
    joint_state.position[4] = jaco_angles.joint5;
    joint_state.position[5] = jaco_angles.joint6;

    // TODO: Add joint velocity
    // joint_state.velocity.resize(6);

    // TODO: Place the arm actuator forces into the array
    // JacoForces arm_forces;
    // arm_.GetForcesInfo(arm_forces);
    // joint_state.effort.resize(6);
    // joint_state.effort[0] = arm_forces.Actuator1;
    // joint_state.effort[1] = arm_forces.Actuator2;
    // joint_state.effort[2] = arm_forces.Actuator3;
    // joint_state.effort[3] = arm_forces.Actuator4;
    // joint_state.effort[4] = arm_forces.Actuator5;
    // joint_state.effort[5] = arm_forces.Actuator6;

    joint_angles_publisher_.publish(jaco_angles);
    joint_state_publisher_.publish(joint_state);
}

/*!
 * \brief Publishes the current cartesian coordinates
 */
void JacoArm::publishToolPosition(void) {
//    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    JacoPose pose;
    geometry_msgs::PoseStamped current_position;

    jaco_api_.getCartesianPosition(pose);
    current_position.pose = pose.constructPoseMsg();

    tool_position_publisher_.publish(current_position);
}

/*!
 * \brief Publishes the current finger positions.
 */
void JacoArm::publishFingerPosition(void) {
//    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);

    FingerAngles fingers;
    //jaco_msgs::FingerPosition finger_position;

    jaco_api_.getFingerPositions(fingers);
    finger_position_publisher_.publish(fingers.constructFingersMsg());
}


void JacoArm::statusTimer(const ros::TimerEvent&) {
//    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    publishJointAngles();
    publishToolPosition();
    publishFingerPosition();

//    ROS_INFO_STREAM("Getting quick status for the timer");
    QuickStatus current_status;
    jaco_api_.getQuickStatus(current_status);
}

}  // namespace jaco
