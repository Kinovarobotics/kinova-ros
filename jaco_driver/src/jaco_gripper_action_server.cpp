
/**
 *  File: jaco_gripper_action_server.cpp
 *  Desc: Class for interfacing moveIt with the jaco arm's gripper.
 *  Auth: Plinio Moreno
 *  email: plinio@isr.tecnico.ulisboa.pt
 *
 */


#include <kinova/KinovaTypes.h>

#include "jaco_driver/jaco_gripper_action_server.h"

#include "jaco_driver/jaco_types.h"

namespace jaco
{

JacoGripperActionServer::JacoGripperActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh, "fingers"),
      action_server_(node_handle_, "gripper_action_server",
                     boost::bind(&JacoGripperActionServer::actionCallback, this, _1), false)
{
    double tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
    node_handle_.param<double>("tolerance", tolerance, 2.0);
    tolerance_ = static_cast<float>(tolerance);
    action_server_.start();
}


JacoGripperActionServer::~JacoGripperActionServer()
{
}


/*****************************************/
/***********  Gripper Control ************/
/*****************************************/

void JacoGripperActionServer::actionCallback(const control_msgs::GripperCommandGoalConstPtr &goal)
{
   
    control_msgs::GripperCommandFeedback feedback;
    control_msgs::GripperCommandResult result;
    FingerAngles current_finger_positions;
    ros::Time current_time = ros::Time::now();
    result.reached_goal = false;
    try
    {
        arm_comm_.getFingerPositions(current_finger_positions);

        if (arm_comm_.isStopped())
        {
            ROS_INFO("Could not complete finger action because the arm is stopped");
            result.position = current_finger_positions.Finger1;
            action_server_.setAborted(result);
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_finger_positions_ = current_finger_positions;

        FingerAngles target;
	target.Finger1 = goal->command.position;
	target.Finger2 = goal->command.position;
	target.Finger3 = goal->command.position; 
        arm_comm_.setFingerPositions(target);

        // Loop until the action completed, is preempted, or fails in some way.
        // timeout is left to the caller since the timeout may greatly depend on
        // the context of the movement.
        while (true)
        {
            ros::spinOnce();

            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                result.position = current_finger_positions.Finger1;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
                return;
            }
            else if (arm_comm_.isStopped())
            {
                result.position = current_finger_positions.Finger1;
                action_server_.setAborted(result);
                return;
            }

            arm_comm_.getFingerPositions(current_finger_positions);
            current_time = ros::Time::now();
            //feedback.fingers = current_finger_positions.constructFingersMsg();
            //action_server_.publishFeedback(feedback);

            if (target.isCloseToOther(current_finger_positions, tolerance_))
            {
                // Check if the action has succeeeded
                result.position = current_finger_positions.Finger1;
		result.reached_goal = true;
                action_server_.setSucceeded(result);
                return;
            }
            else if (!last_nonstall_finger_positions_.isCloseToOther(current_finger_positions, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_finger_positions_ = current_finger_positions;
            }
            else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
            {
                // Check if the full stall condition has been meet
                result.position = current_finger_positions.Finger1;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
                return;
            }

            ros::Rate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.position = current_finger_positions.Finger1;;
        ROS_ERROR_STREAM(e.what());
        action_server_.setAborted(result);
    }

}

}  // namespace jaco
