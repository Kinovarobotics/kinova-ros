//============================================================================
// Name        : kinova_arm.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_arm.h"
#include <string>
#include <boost/lexical_cast.hpp>
#include <kinova_driver/kinova_ros_types.h>


namespace 
{
    /// \brief Convert Kinova-specific angle degree variations (0..180, 360-181) to
    ///        a more regular representation (0..180, -180..0).
    inline void convertKinDeg(double& qd)
    {
        static const double PI_180 = (M_PI / 180.0);
        // Angle velocities from the API are 0..180 for positive values,
        // and 360..181 for negative ones, in a kind of 2-complement setup.
        if (qd > 180.0) {
            qd -= 360.0;
        }
        qd *= PI_180;
    }

    inline void convertKinDeg(std::vector<double>& qds)
    {
        for (int i = 0; i < qds.size(); ++i) {
            double& qd = qds[i];
            convertKinDeg(qd);
        }
    }

    inline void convertKinDeg(geometry_msgs::Vector3& qds)
    {
        convertKinDeg(qds.x);
        convertKinDeg(qds.y);
        convertKinDeg(qds.z);
    }
}

namespace kinova
{

KinovaArm::KinovaArm(KinovaComm &arm, const ros::NodeHandle &nodeHandle, const std::string &kinova_robotType)
    : kinova_comm_(arm), node_handle_(nodeHandle), kinova_robotType_(kinova_robotType)
{
    /* Set up parameters for different robot type */
    // example for a kinova_robotType: j2n6s300

    if (valid_kinovaRobotType(kinova_robotType_) == false)
    {
        ROS_WARN("Invalid kinova_robotType error! Obtained: %s.", kinova_robotType_.c_str());
        return;
    }

//    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_ + "_";

    // Maximum number of joints on Kinova-like robots:
    robot_category_ = kinova_robotType_[0];
    robot_category_version_ = kinova_robotType_[1]-'0';
    wrist_type_ = kinova_robotType_[2];
    arm_joint_number_ = kinova_robotType_[3]-'0';
    robot_mode_ = kinova_robotType_[4];
    finger_number_ = kinova_robotType_[5]-'0';
    joint_total_number_ = arm_joint_number_ + finger_number_;

    if (robot_category_=='j') // jaco robot
    {
        // special parameters for jaco
    }
    else if (robot_category_ == 'm') // mico robot
    {
        // special parameters for mico
    }
    else if (robot_category_ == 'r') // roco robot
    {
        // special parameters for roco
    }
    else
    {
        // special parameters for custom robot or other cases
    }


    if (finger_number_==3)
    {
        // finger_angle_conv_ratio used to display finger position properly in Rviz
        // Approximative conversion ratio from finger position (0..6400) to joint angle
        // in radians (0.. 1.4) for 3 finger hand
        finger_conv_ratio_= 1.4 / 6400.0;
    }
    else if(finger_number_==2)
    {
        // the two finger hand may different
        finger_conv_ratio_= 1.4 / 6400.0;
    }
    else
    {
        // no fingers
    }

    for (int i = 0; i<arm_joint_number_; i++)
    {
        joint_names_.resize(joint_total_number_);
        joint_names_[i] = tf_prefix_ + "joint_" + boost::lexical_cast<std::string>(i+1);
    }
    for (int i = 0; i<finger_number_; i++)
    {
        joint_names_[arm_joint_number_+i] = tf_prefix_ + "joint_finger_" + boost::lexical_cast<std::string>(i+1);
    }

    /* Set up Services */
    stop_service_ = node_handle_.advertiseService("in/stop", &KinovaArm::stopServiceCallback, this);
    start_service_ = node_handle_.advertiseService("in/start", &KinovaArm::startServiceCallback, this);
    homing_service_ = node_handle_.advertiseService("in/home_arm", &KinovaArm::homeArmServiceCallback, this);

    set_force_control_params_service_ = node_handle_.advertiseService("in/set_force_control_params", &KinovaArm::setForceControlParamsCallback, this);
    start_force_control_service_ = node_handle_.advertiseService("in/start_force_control", &KinovaArm::startForceControlCallback, this);
    stop_force_control_service_ = node_handle_.advertiseService("in/stop_force_control", &KinovaArm::stopForceControlCallback, this);
    
    set_end_effector_offset_service_ = node_handle_.advertiseService("in/set_end_effector_offset",
        &KinovaArm::setEndEffectorOffsetCallback, this);

    /* Set up Publishers */
    joint_angles_publisher_ = node_handle_.advertise<kinova_msgs::JointAngles>("out/joint_angles", 2);
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("out/joint_state", 2);
    tool_position_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("out/tool_pose", 2);
    tool_wrench_publisher_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("out/tool_wrench", 2);
    finger_position_publisher_ = node_handle_.advertise<kinova_msgs::FingerPosition>("out/finger_position", 2);

    // Publish last command for relative motion (read current position cause arm drop)
    joint_command_publisher_ = node_handle_.advertise<kinova_msgs::JointAngles>("out/joint_command", 2);
    cartesian_command_publisher_ = node_handle_.advertise<kinova_msgs::KinovaPose>("out/cartesian_command", 2);

    /* Set up Subscribers*/
    joint_velocity_subscriber_ = node_handle_.subscribe("in/joint_velocity", 1,
                                                      &KinovaArm::jointVelocityCallback, this);
    cartesian_velocity_subscriber_ = node_handle_.subscribe("in/cartesian_velocity", 1,
                                                          &KinovaArm::cartesianVelocityCallback, this);

    node_handle_.param<double>("status_interval_seconds", status_interval_seconds_, 0.1);

    // Depending on the API version, the arm might return velocities in the
    // 0..360 range (0..180 for positive values, 181..360 for negative ones).
    // This indicates that the ROS node should convert them first before
    // updating the joint_state topic.
    node_handle_.param("convert_joint_velocities", convert_joint_velocities_, true);

    status_timer_ = node_handle_.createTimer(ros::Duration(status_interval_seconds_),
                                           &KinovaArm::statusTimer, this);

    ROS_INFO("The arm is ready to use.");
}


KinovaArm::~KinovaArm()
{
}


bool KinovaArm::homeArmServiceCallback(kinova_msgs::HomeArm::Request &req, kinova_msgs::HomeArm::Response &res)
{
    kinova_comm_.homeArm();
    kinova_comm_.initFingers();
    res.homearm_result = "KINOVA ARM HAS BEEN RETURNED HOME";
    return true;
}


void KinovaArm::jointVelocityCallback(const kinova_msgs::JointVelocityConstPtr& joint_vel)
{
    if (!kinova_comm_.isStopped())
    {
        joint_velocities_.Actuator1 = joint_vel->joint1;
        joint_velocities_.Actuator2 = joint_vel->joint2;
        joint_velocities_.Actuator3 = joint_vel->joint3;
        joint_velocities_.Actuator4 = joint_vel->joint4;
        joint_velocities_.Actuator5 = joint_vel->joint5;
        joint_velocities_.Actuator6 = joint_vel->joint6;

        kinova_comm_.setJointVelocities(joint_velocities_);

    }
}


/*!
 * \brief Handler for "stop" service.
 *
 * Instantly stops the arm and prevents further movement until start service is
 * invoked.
 */
bool KinovaArm::stopServiceCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res)
{
    kinova_comm_.stopAPI();
    res.stop_result = "Arm stopped";
    ROS_DEBUG("Arm stop requested");
    return true;
}


/*!
 * \brief Handler for "start" service.
 *
 * Re-enables control of the arm after a stop.
 */
bool KinovaArm::startServiceCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res)
{
    kinova_comm_.startAPI();
    res.start_result = "Arm started";
    ROS_DEBUG("Arm start requested");
    return true;
}

bool KinovaArm::setForceControlParamsCallback(kinova_msgs::SetForceControlParams::Request &req, kinova_msgs::SetForceControlParams::Response &res)
{
    CartesianInfo inertia, damping, force_min, force_max;
    inertia.X      = req.inertia_linear.x;
    inertia.Y      = req.inertia_linear.y;
    inertia.Z      = req.inertia_linear.z;
    inertia.ThetaX = req.inertia_angular.x;
    inertia.ThetaY = req.inertia_angular.y;
    inertia.ThetaZ = req.inertia_angular.z;
    damping.X      = req.damping_linear.x;
    damping.Y      = req.damping_linear.y;
    damping.Z      = req.damping_linear.z;
    damping.ThetaX = req.damping_angular.x;
    damping.ThetaY = req.damping_angular.y;
    damping.ThetaZ = req.damping_angular.z;

    kinova_comm_.setCartesianInertiaDamping(inertia, damping);

    force_min.X      = req.force_min_linear.x;
    force_min.Y      = req.force_min_linear.y;
    force_min.Z      = req.force_min_linear.z;
    force_min.ThetaX = req.force_min_angular.x;
    force_min.ThetaY = req.force_min_angular.y;
    force_min.ThetaZ = req.force_min_angular.z;
    force_max.X      = req.force_max_linear.x;
    force_max.Y      = req.force_max_linear.y;
    force_max.Z      = req.force_max_linear.z;
    force_max.ThetaX = req.force_max_angular.x;
    force_max.ThetaY = req.force_max_angular.y;
    force_max.ThetaZ = req.force_max_angular.z;

    kinova_comm_.setCartesianForceMinMax(force_min, force_max);

    return true;
}

bool KinovaArm::startForceControlCallback(kinova_msgs::Start::Request &req, kinova_msgs::Start::Response &res)
{
    kinova_comm_.startForceControl();
    res.start_result = "Start force control requested.";
    return true;
}

bool KinovaArm::stopForceControlCallback(kinova_msgs::Stop::Request &req, kinova_msgs::Stop::Response &res)
{
    kinova_comm_.stopForceControl();
    res.stop_result = "Stop force control requested.";
    return true;
}

bool KinovaArm::setEndEffectorOffsetCallback(kinova_msgs::SetEndEffectorOffset::Request &req, kinova_msgs::SetEndEffectorOffset::Response &res)
{
    kinova_comm_.setEndEffectorOffset(req.status, req.offset.x, req.offset.y, req.offset.z);

    return true;
}

void KinovaArm::cartesianVelocityCallback(const kinova_msgs::PoseVelocityConstPtr& cartesian_vel)
{
    if (!kinova_comm_.isStopped())
    {
        cartesian_velocities_.X = cartesian_vel->twist_linear_x;
        cartesian_velocities_.Y = cartesian_vel->twist_linear_y;
        cartesian_velocities_.Z = cartesian_vel->twist_linear_z;
        cartesian_velocities_.ThetaX = cartesian_vel->twist_angular_x;
        cartesian_velocities_.ThetaY = cartesian_vel->twist_angular_y;
        cartesian_velocities_.ThetaZ = cartesian_vel->twist_angular_z;

        // orientation velocity of cartesian_velocities_ is based on twist.angular
        kinova_comm_.setCartesianVelocities(cartesian_velocities_);
    }
}

/*!
 * \brief Publishes the current joint angles.
 *
 * Joint angles are published in both their raw state as obtained from the arm
 * (JointAngles), and transformed & converted to radians (joint_state) as per
 * the Kinova Kinematics PDF.
 *
 * Velocities and torques (effort) are only published in the JointStates
 * message, only for the first 6 joints as these values are not available for
 * the fingers.
 */
void KinovaArm::publishJointAngles(void)
{

    FingerAngles fingers;
    kinova_comm_.getFingerPositions(fingers);

    if (arm_joint_number_ != 4 && arm_joint_number_ != 6)
    {
         ROS_WARN_ONCE("The joint_state publisher only supports 4DOF and 6DOF for now.: %d", arm_joint_number_);
    }

    // Query arm for current joint angles
    KinovaAngles current_angles;
    kinova_comm_.getJointAngles(current_angles);
    kinova_msgs::JointAngles kinova_angles = current_angles.constructAnglesMsg();

    AngularPosition joint_command;
    kinova_comm_.getAngularCommand(joint_command);
    kinova_msgs::JointAngles joint_command_msg = KinovaAngles(joint_command.Actuators).constructAnglesMsg();

    sensor_msgs::JointState joint_state;
    joint_state.name = joint_names_;
    joint_state.header.stamp = ros::Time::now();

    // Transform from Kinova DH algorithm to physical angles in radians, then place into vector array
    joint_state.position.resize(joint_total_number_);
    joint_state.position[0] = kinova_angles.joint1 * M_PI/180;
    joint_state.position[1] = kinova_angles.joint2 * M_PI/180;
    joint_state.position[2] = kinova_angles.joint3 * M_PI/180;
    joint_state.position[3] = kinova_angles.joint4 * M_PI/180;
    if (arm_joint_number_ == 6)
    {
        joint_state.position[4] = kinova_angles.joint5 * M_PI/180;
        joint_state.position[5] = kinova_angles.joint6 * M_PI/180;
    }

    if(finger_number_==2)
    {
        joint_state.position[joint_total_number_-2] = 0;
        joint_state.position[joint_total_number_-1] = 0;
    }
    else if(finger_number_==3)
    {
        joint_state.position[joint_total_number_-3] = 0;
        joint_state.position[joint_total_number_-2] = 0;
        joint_state.position[joint_total_number_-1] = 0;
    }


    // Joint velocities
    KinovaAngles current_vels;
    kinova_comm_.getJointVelocities(current_vels);
    joint_state.velocity.resize(joint_total_number_);
    joint_state.velocity[0] = current_vels.Actuator1;
    joint_state.velocity[1] = current_vels.Actuator2;
    joint_state.velocity[2] = current_vels.Actuator3;
    joint_state.velocity[3] = current_vels.Actuator4;
    // no velocity info for fingers
    if(finger_number_==2)
    {
        joint_state.velocity[joint_total_number_-2] = 0;
        joint_state.velocity[joint_total_number_-1] = 0;
    }
    else if(finger_number_==3)
    {
        joint_state.velocity[joint_total_number_-3] = 0;
        joint_state.velocity[joint_total_number_-2] = 0;
        joint_state.velocity[joint_total_number_-1] = 0;
    }

    if (arm_joint_number_ == 6)
    {
        joint_state.velocity[4] = current_vels.Actuator5;
        joint_state.velocity[5] = current_vels.Actuator6;
    }

    ROS_DEBUG_THROTTLE(0.1,
                       "Raw joint velocities: %f %f %f %f %f %f",
                       current_vels.Actuator1,
                       current_vels.Actuator2,
                       current_vels.Actuator3,
                       current_vels.Actuator4,
                       current_vels.Actuator5,
                       current_vels.Actuator6);

    if (convert_joint_velocities_) {
        convertKinDeg(joint_state.velocity);
    }


    // Joint torques (effort)
    // NOTE: Currently invalid.
    KinovaAngles joint_tqs;
    joint_state.effort.resize(joint_total_number_);
    joint_state.effort[0] = joint_tqs.Actuator1;
    joint_state.effort[1] = joint_tqs.Actuator2;
    joint_state.effort[2] = joint_tqs.Actuator3;
    joint_state.effort[3] = joint_tqs.Actuator4;
    // no effort info for fingers
    if(finger_number_==2)
    {
        joint_state.effort[joint_total_number_-2] = 0;
        joint_state.effort[joint_total_number_-1] = 0;
    }
    else if(finger_number_==3)
    {
        joint_state.effort[joint_total_number_-3] = 0;
        joint_state.effort[joint_total_number_-2] = 0;
        joint_state.effort[joint_total_number_-1] = 0;
    }
    if (arm_joint_number_ == 6)
    {
        joint_state.effort[4] = joint_tqs.Actuator5;
        joint_state.effort[5] = joint_tqs.Actuator6;
    }

    joint_angles_publisher_.publish(kinova_angles);
    joint_command_publisher_.publish(joint_command_msg);
    joint_state_publisher_.publish(joint_state);

}

/*!
 * \brief Publishes the current cartesian coordinates
 */
void KinovaArm::publishToolPosition(void)
{
    KinovaPose pose;
    geometry_msgs::PoseStamped current_position;
    kinova_comm_.getCartesianPosition(pose);


    CartesianPosition cartesian_command;
    kinova_comm_.getCartesianCommand(cartesian_command);
    kinova_msgs::KinovaPose cartesian_command_msg = KinovaPose(cartesian_command.Coordinates).constructKinovaPoseMsg();

    current_position.pose            = pose.constructPoseMsg();
    current_position.header.stamp    = ros::Time::now();
    current_position.header.frame_id = tf_prefix_ + "link_base";

    tool_position_publisher_.publish(current_position);
    cartesian_command_publisher_.publish(cartesian_command_msg);
}

/*!
 * \brief Publishes the current cartesian forces at the end effector.
 */
void KinovaArm::publishToolWrench(void)
{
    KinovaPose wrench;
    geometry_msgs::WrenchStamped current_wrench;

    kinova_comm_.getCartesianForce(wrench);

    current_wrench.wrench          = wrench.constructWrenchMsg();
    current_wrench.header.stamp    = ros::Time::now();
    // TODO: Rotate wrench to fit the end effector frame.
    // Right now, the orientation of the wrench is in the API's (base) frame.
    current_wrench.header.frame_id = tf_prefix_ + "link_base";


    // Same conversion issue as with velocities:
    if (convert_joint_velocities_) {
        convertKinDeg(current_wrench.wrench.torque);
    }

    tool_wrench_publisher_.publish(current_wrench);
}

/*!
 * \brief Publishes the current finger positions.
 */
void KinovaArm::publishFingerPosition(void)
{
    FingerAngles fingers;
    kinova_comm_.getFingerPositions(fingers);
    finger_position_publisher_.publish(fingers.constructFingersMsg());
}


void KinovaArm::statusTimer(const ros::TimerEvent&)
{
    publishJointAngles();
    publishToolPosition();
    publishToolWrench();
    publishFingerPosition();
}

}  // namespace kinova
