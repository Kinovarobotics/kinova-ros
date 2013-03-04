//============================================================================
// Name        : Jaco.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/**
 * @file jaco_arm_control.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <jaco_arm/jaco_arm_controller.h>

using namespace std;
using namespace jaco_arm;

JacoArm::JacoArm() {
	ROS_INFO("Initiating Library");
	Jacolib_Init();
	ROS_INFO("Initiating API");

	int api_result = 0; //stores result from the API

	api_result = (Jaco_InitAPI)();

	if (api_result != 1) {
		ROS_FATAL("Could not initialize arm");
		ROS_FATAL("Jaco_InitAPI returned: %d", api_result);
		ros::shutdown();

	} else {
		ROS_INFO("API Initialized Successfully!");
	}
}

void JacoArm::GoToPosition(const geometry_msgs::PoseStampedConstPtr& arm_pose) {
	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position));

	double x,y,z;
	tf::Quaternion q;
	tf::quaternionMsgToTF(arm_pose->pose.orientation,q);

	tf::Matrix3x3 bt_q(q);

	bt_q.getEulerYPR(z,y,x);

	ROS_INFO("X = %f", arm_pose->pose.position.x);
	ROS_INFO("Y = %f", arm_pose->pose.position.y);
	ROS_INFO("Z = %f", arm_pose->pose.position.z);

	ROS_INFO("Theta X = %f", x);
	ROS_INFO("Theta Y = %f", y);
	ROS_INFO("Theta Z = %f", z);







	Jaco_StopControlAPI();

	Jaco_StartControlAPI();

	Jaco_Position.Position.Type = CARTESIAN_POSITION;

	Jaco_Position.Position.CartesianPosition.X =
			(float) arm_pose->pose.position.x;
	Jaco_Position.Position.CartesianPosition.Y =
			(float) arm_pose->pose.position.y;
	Jaco_Position.Position.CartesianPosition.Z =
			(float) arm_pose->pose.position.z;




	Jaco_Position.Position.CartesianPosition.ThetaX =
			(float) x;
	Jaco_Position.Position.CartesianPosition.ThetaY =
			(float) y;
	Jaco_Position.Position.CartesianPosition.ThetaZ =
			(float) z;

	Jaco_SendAdvanceTrajectory(Jaco_Position);

	//Jaco_StopControlAPI();

}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string JacoPosition("JacoPosition"); ///String containing the topic name for cartesian commands

	if (argc < 1) {
		ROS_INFO( "Usage: jaco_arm_controller cartesian_info_topic");
		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(JacoPosition, JacoPosition))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Jaco Position Topic <%s>!", JacoPosition.c_str(), JacoPosition.c_str());
	}

	//Print out received topics
	ROS_DEBUG("Got Jaco Position Topic Name: <%s>", JacoPosition.c_str());

	ROS_INFO("Starting Up Jaco Arm Controller...");

	//create the arm object
	JacoArm jaco;

	ros::Subscriber sub = nh.subscribe(JacoPosition, 1, &JacoArm::GoToPosition,
			&jaco);

	ros::spin();

}

