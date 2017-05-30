#ifndef KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_JOINT_TRAJECTORY_CONTROLLER_H


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Duration.h>
#include <kinova_msgs/JointVelocity.h>
#include <boost/thread.hpp>

#include "kinova_ros_types.h"
#include "kinova_comm.h"
#include "kinova_api.h"

namespace kinova
{

class JointTrajectoryController
{
public:
    JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle &n);
    ~JointTrajectoryController();


private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_command_;
    ros::Publisher pub_joint_feedback_;
    ros::Publisher pub_joint_velocity_;

    ros::Time previous_pub_;
    ros::Time time_pub_joint_vel_;

    ros::Timer timer_pub_joint_vel_;
    boost::mutex terminate_thread_mutex_;
    boost::thread* thread_update_state_;
    bool terminate_thread_;

    std_msgs::Duration time_from_start_;
    sensor_msgs::JointState current_joint_state_;

    KinovaComm kinova_comm_;
    TrajectoryPoint kinova_traj_point_;

//    trajectory_msgs::JointTrajectory joint_traj_;
//    trajectory_msgs::JointTrajectoryPoint joint_traj_point_;
    std::string traj_frame_id_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_command_points_;
    control_msgs::FollowJointTrajectoryFeedback traj_feedback_msg_;

    // stores the command to send to robot, in Kinova type (KinovaAngles)
    std::vector<KinovaAngles> kinova_angle_command_;

    uint number_joint_;
    int traj_command_points_index_;
    std::vector<std::string> joint_names_;
    std::string prefix_;

    struct Segment
    {
        double start_time;
        double duration;
        std::vector<double> positions;
        std::vector<double> velocities;
    };


    // call back function when receive a trajectory command
    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg);

    // reflash the robot state and publish the joint state: either by timer or thread
    void update_state(); // by thread

    void pub_joint_vel(const ros::TimerEvent&); // by timer
    int test;

};






}


#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
