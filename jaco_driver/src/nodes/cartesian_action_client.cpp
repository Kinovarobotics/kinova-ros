#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jaco_msgs/ArmPoseAction.h>
#include <geometry_msgs/PoseStamped.h>

#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_comm.h"


int main (int argc, char** argv)
{
	ros::init(argc, argv, "pose_action_client");

	actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/jaco_arm_driver/arm_pose/arm_pose", true);
	
	ROS_INFO("Waiting for action server to start.");
	
	ac.waitForServer();
	
	ROS_INFO("Action server started, sending goal.");
	
	jaco_msgs::ArmPoseGoal goal;

	goal.pose.pose.position.x = -0.35;
	goal.pose.pose.position.y = -0.35;
	goal.pose.pose.position.z = 0.01;
	goal.pose.pose.orientation.x = 0.00;
	goal.pose.pose.orientation.y = 1.00;
	goal.pose.pose.orientation.z = 0.00;	
	goal.pose.pose.orientation.w = 0.00;	
    
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "jaco_link_base";
	
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
	
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
	  ROS_INFO("Action did not finish before the time out.");
	  
	//exit
	return 0;
}

