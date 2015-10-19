#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jaco_msgs/ArmJointAnglesAction.h>


#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_comm.h"

#define PI 3.1415926

int main (int argc, char** argv)
{
	ros::init(argc, argv, "angle_action_client");

	actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/jaco_arm_driver/joint_angles/arm_joint_angles", true);
	
	ROS_INFO("Waiting for action server to start.");
	
	ac.waitForServer();
	
	ROS_INFO("Action server started, sending goal.");
	
	jaco_msgs::ArmJointAnglesGoal goal;

	goal.angles.joint1 = ( 307.720581055 - 180.0 ) / 180.0 * PI;
	goal.angles.joint2 = ( 241.5 - 270.0 ) / 180.0 * PI;
	goal.angles.joint3 = ( 90.2205886841 - 90.0 ) / 180.0 * PI;
	goal.angles.joint4 = ( 350.659088135 - 180.0 ) / 180.0 * PI;
	goal.angles.joint5 = ( 34.4318199158 - 180.0 ) / 180.0 * PI;
	goal.angles.joint6 = ( 208.636367798 - 270 ) / 180.0 * PI;	

	
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
	
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

