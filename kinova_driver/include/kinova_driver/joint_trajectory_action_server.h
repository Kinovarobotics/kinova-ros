#ifndef JOINT_TRAJECTORY_ACTION_SERVER_H
#define JOINT_TRAJECTORY_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace kinova
{
    class JointTrajectoryActionController
    {
        typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
        typedef std::vector<trajectory_msgs::JointTrajectoryPoint> JTPointVector;

    public:
        JointTrajectoryActionController(ros::NodeHandle &n, std::string &robot_name);
        ~JointTrajectoryActionController();


    private:
        ros::NodeHandle nh_;
        boost::shared_ptr<FJTAS> action_server_follow_;

        ros::Publisher pub_controller_command_;
        ros::Subscriber sub_controller_state_;
        ros::Timer watchdog_timer_;

        bool has_active_goal_;
        bool first_fb_;
        ros::Time start_time_;
        FJTAS::GoalHandle active_goal_;
        trajectory_msgs::JointTrajectory current_traj_;
        control_msgs::FollowJointTrajectoryFeedbackConstPtr last_controller_state_;

        std::vector<std::string> joint_names_;
        std::map<std::string,double> goal_constraints_;
        std::map<std::string,double> trajectory_constraints_;
        double goal_time_constraint_;
        double stopped_velocity_tolerance_;

        void goalCBFollow(FJTAS::GoalHandle gh);
        void cancelCBFollow(FJTAS::GoalHandle gh);
        void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);
        void watchdog(const ros::TimerEvent &e);

    };
}


#endif // JOINT_TRAJECTORY_ACTION_SERVER_H
