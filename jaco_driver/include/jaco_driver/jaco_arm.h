/*
 * jaco_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef JACO_ARM_DRIVER_H_
#define JACO_ARM_DRIVER_H_

#include <ros/ros.h>
#include <jaco_driver/jaco_api.h>
#include "kinova/KinovaTypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "jaco_msgs/JointVelocity.h"
#include "jaco_msgs/FingerPosition.h"
#include "jaco_msgs/JointAngles.h"
#include "sensor_msgs/JointState.h"

#include <jaco_msgs/Stop.h>
#include <jaco_msgs/Start.h>
#include <jaco_msgs/HomeArm.h>

#include "jaco_driver/jaco_comm.h"

#include <time.h>
#include <math.h>
#include <vector>


namespace jaco {

class JacoArm {
 public:
    JacoArm(JacoComm& arm, ros::NodeHandle &nodeHandle);
    ~JacoArm();

    void jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel);
    void cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel);

    bool stopServiceCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res);
    bool startServiceCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res);
    bool homeArmServiceCallback(jaco_msgs::HomeArm::Request &req, jaco_msgs::HomeArm::Response &res);

 private:
    //void goHome(void);  // TODO: See if this is needed
    //void calculatePostion(void);

    void positionTimer(const ros::TimerEvent&);
    void cartesianVelocityTimer(const ros::TimerEvent&);
    void jointVelocityTimer(const ros::TimerEvent&);
    void statusTimer(const ros::TimerEvent&);

    void publishJointAngles(void);
    void publishToolPosition(void);
    void publishFingerPosition(void);

    JacoComm &jaco_api_;

    ros::Subscriber joint_velocity_subscriber_;
    ros::Subscriber cartesian_velocity_subscriber_;
    // ros::Subscriber software_pause_sub_;  // TODO: Does this get used?

    ros::Publisher joint_angles_publisher_;
    ros::Publisher tool_position_publisher_;
    ros::Publisher finger_position_publisher_;
    ros::Publisher joint_state_publisher_;

    ros::ServiceServer stop_service_;
    ros::ServiceServer start_service_;
    ros::ServiceServer homing_service_;

    ros::Timer status_timer_;
    ros::Timer cartesian_vel_timer_;
    ros::Timer joint_vel_timer_;

    double status_interval_seconds_;
    double joint_angular_vel_timeout_seconds_;
    double cartesian_vel_timeout_seconds_;

    bool cartesian_vel_timer_flag_;
    bool joint_vel_timer_flag_;

    AngularInfo joint_velocities_;
    CartesianInfo cartesian_velocities_;

    ros::Time last_joint_vel_cmd_time_;
    ros::Time last_cartesian_vel_cmd_time_;

    tf::TransformListener tf_listener_;

    ros::NodeHandle nodeHandle_;

//    ros::Time last_update_time_;
//    ros::Duration update_time_;
//    uint8_t previous_state_;
};

}  // namespace jaco
#endif  // JACO_ARM_DRIVER_H_
