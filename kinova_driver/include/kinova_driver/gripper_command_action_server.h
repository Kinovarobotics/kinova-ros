#ifndef GRIPPER_COMMAND_ACTION_SERVER_H
#define GRIPPER_COMMAND_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

#include <kinova_msgs/SetFingersPositionAction.h>


namespace kinova
{
    class GripperCommandActionController
    {
        typedef actionlib::ActionServer<control_msgs::GripperCommandAction> GCAS;
        typedef actionlib::ActionClient<kinova_msgs::SetFingersPositionAction> SFPAC;

    public:
        GripperCommandActionController(ros::NodeHandle &n, std::string &robot_name);
        ~GripperCommandActionController();


    private:
        ros::NodeHandle nh_;
        boost::shared_ptr<GCAS> action_server_gripper_command_;
        boost::shared_ptr<SFPAC> action_client_set_finger_;

        ros::Subscriber sub_fingers_state_;

        bool has_active_goal_;
        GCAS::GoalHandle active_goal_;
        kinova_msgs::FingerPositionConstPtr last_finger_state_;

        std::vector<std::string> gripper_joint_names_;
        double gripper_command_goal_constraint_;
        double gripper_joint_num_;

        double finger_max_turn_ ; // maximum turn (KinovaFinger defalt unit) value
        double finger_conv_ratio_; // finger value convert ratio defined in kinova_driver/kinova_arm

        void goalCBFollow(GCAS::GoalHandle gh);
        void cancelCBFollow(GCAS::GoalHandle gh);
        void controllerStateCB(const kinova_msgs::FingerPositionConstPtr &msg);

    };
}



#endif // GRIPPER_COMMAND_ACTION_SERVER_H
