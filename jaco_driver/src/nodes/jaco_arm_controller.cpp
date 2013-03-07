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
#include <jaco_driver/jaco_arm_controller.h>

using namespace std;
using namespace jaco_arm;

JacoArm::JacoArm(ros::NodeHandle nh, std::string ArmPose) {
	ROS_INFO("Initiating Library");
	Jacolib_Init();
	ROS_INFO("Initiating API");

	int api_result = 0; //stores result from the API

	api_result = (Jaco_InitAPI)();

	if (api_result != 1) {
		ROS_FATAL("Could not initialize arm");
		ROS_FATAL("Jaco_InitAPI returned: %d", api_result);
#ifndef DEBUG_WITHOUT_ARM
		ros::shutdown();
#endif
	} else {
		ROS_INFO("API Initialized Successfully!");

	}

	tf::Transform transform;
	tf::Quaternion rotation_q(0, 0, 0, 0);
	tf::Vector3 translation_v(0, 0, 0);

	/**********************API Origin Offset**********************/
	/* API Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get api rotation matrix
	kinematics.Origin_Rotation(kinematics.aa(), rotation_q);

	/* Display Results */
	ROS_INFO(
			"API Origin Offset: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* API Translation */
	translation_v.setValue(0, 0, 0); //zero translation

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_base",
					"jaco_api_origin"));
	/*********************************************************/

	this->sub = nh.subscribe(ArmPose, 1, &JacoArm::GoToPosition, this);
	this->timer = nh.createTimer(ros::Duration(0.1), &JacoArm::TimerCallback,
			this);

}

void SetAngles(AngularInfo angles) {

}

void SetPosition(CartesianInfo position) {

}

void SetFingers(FingersPosition fingers) {

}

void GetAngles(AngularInfo &angles) {

}

void GetPosition(CartesianInfo &position) {

}

void GetFingers(FingersPosition &fingers) {

}

void JacoArm::GoToPosition(const geometry_msgs::PoseStampedConstPtr& arm_pose) {
	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	double x, y, z;
	tf::Quaternion q;
	tf::quaternionMsgToTF(arm_pose->pose.orientation, q);

	tf::Matrix3x3 bt_q(q);

	bt_q.getEulerYPR(z, y, x);

	ROS_INFO("X = %f", arm_pose->pose.position.x);
	ROS_INFO("Y = %f", arm_pose->pose.position.y);
	ROS_INFO("Z = %f", arm_pose->pose.position.z);

	ROS_INFO("Theta X = %f", x);
	ROS_INFO("Theta Y = %f", y);
	ROS_INFO("Theta Z = %f", z);

	Jaco_EraseAllTrajectories();
	Jaco_StopControlAPI();

	Jaco_StartControlAPI();

	Jaco_Position.Position.Type = CARTESIAN_POSITION;

	Jaco_Position.Position.CartesianPosition.X =
			(float) arm_pose->pose.position.x;
	Jaco_Position.Position.CartesianPosition.Y =
			(float) arm_pose->pose.position.y;
	Jaco_Position.Position.CartesianPosition.Z =
			(float) arm_pose->pose.position.z;

	Jaco_Position.Position.CartesianPosition.ThetaX = (float) x;
	Jaco_Position.Position.CartesianPosition.ThetaY = (float) y;
	Jaco_Position.Position.CartesianPosition.ThetaZ = (float) z;

	Jaco_SendBasicTrajectory(Jaco_Position);

}

void JacoArm::GoHome(void) {
	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	Jaco_EraseAllTrajectories();
	Jaco_StopControlAPI();

	Jaco_StartControlAPI();

	Jaco_Position.Position.Type = ANGULAR_POSITION;

	Jaco_Position.Position.Actuators.Actuator1 = 270.385651;
	Jaco_Position.Position.Actuators.Actuator2 = 150.203217;
	Jaco_Position.Position.Actuators.Actuator3 = 26.541765;
	Jaco_Position.Position.Actuators.Actuator4 = 267.597351;
	Jaco_Position.Position.Actuators.Actuator5 = 5.570505;
	Jaco_Position.Position.Actuators.Actuator6 = 99.634575;

	Jaco_SendBasicTrajectory(Jaco_Position);

//	ROS_INFO("X = %f", Jaco_Position.Position.CartesianPosition.X);
//	ROS_INFO("Y = %f", Jaco_Position.Position.CartesianPosition.Y);
//	ROS_INFO("Z = %f", Jaco_Position.Position.CartesianPosition.Z);
//
//	ROS_INFO("Theta X = %f", Jaco_Position.Position.CartesianPosition.ThetaX);
//	ROS_INFO("Theta Y = %f", Jaco_Position.Position.CartesianPosition.ThetaY);
//	ROS_INFO("Theta Z = %f", Jaco_Position.Position.CartesianPosition.ThetaZ);


}

void JacoArm::PrintPosition(void) {
	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	Jaco_GetCartesianPosition(Jaco_Position);

	ROS_INFO("X = %f", Jaco_Position.Coordinates.X);
	ROS_INFO("Y = %f", Jaco_Position.Coordinates.Y);
	ROS_INFO("Z = %f", Jaco_Position.Coordinates.Z);

	ROS_INFO("Theta X = %f", Jaco_Position.Coordinates.ThetaX);
	ROS_INFO("Theta Y = %f", Jaco_Position.Coordinates.ThetaY);
	ROS_INFO("Theta Z = %f", Jaco_Position.Coordinates.ThetaZ);

}

void JacoArm::PrintAngles(void) {
	AngularPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	Jaco_GetAngularPosition(Jaco_Position);

	ROS_INFO("Joint 1 = %f", Jaco_Position.Actuators.Actuator1);
	ROS_INFO("Joint 2 = %f", Jaco_Position.Actuators.Actuator2);
	ROS_INFO("Joint 3 = %f", Jaco_Position.Actuators.Actuator3);

	ROS_INFO("Joint 4 = %f", Jaco_Position.Actuators.Actuator4);
	ROS_INFO("Joint 5 = %f", Jaco_Position.Actuators.Actuator5);
	ROS_INFO("Joint 6 = %f", Jaco_Position.Actuators.Actuator6);

}

void JacoArm::CalculatePostion(void) {
	AngularPosition arm_angles;
	memset(&arm_angles, 0, sizeof(arm_angles)); //zero structure

#ifndef DEBUG_WITHOUT_ARM
	Jaco_GetAngularPosition(arm_angles); //Query arm for joint angles
#else if
			//Populate with dummy values
			arm_angles.Actuators.Actuator1 = 180;
			arm_angles.Actuators.Actuator2 = 180;
			arm_angles.Actuators.Actuator3 = 180;
			arm_angles.Actuators.Actuator4 = 180;
			arm_angles.Actuators.Actuator5 = 180;
			arm_angles.Actuators.Actuator6 = 180;
#endif

	ROS_INFO("Joint 1 = %f", arm_angles.Actuators.Actuator1);
	ROS_INFO("Joint 2 = %f", arm_angles.Actuators.Actuator2);
	ROS_INFO("Joint 3 = %f", arm_angles.Actuators.Actuator3);

	ROS_INFO("Joint 4 = %f", arm_angles.Actuators.Actuator4);
	ROS_INFO("Joint 5 = %f", arm_angles.Actuators.Actuator5);
	ROS_INFO("Joint 6 = %f", arm_angles.Actuators.Actuator6);

	tf::Transform transform;
	tf::Quaternion rotation_q(0, 0, 0, 0);
	tf::Vector3 translation_v(0, 0, 0);

	/**********************Joint_1**********************/
	/* Joint 1 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get joint 1 rotation matrix
	kinematics.J1_Rotation(
			kinematics.deg_to_rad(
					kinematics.Q1(arm_angles.Actuators.Actuator1)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 1 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 1 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 1 translation vector
	kinematics.J1_Translation(
			kinematics.deg_to_rad(
					kinematics.Q1(arm_angles.Actuators.Actuator1)),
			translation_v);
	/* Display Results */
	ROS_INFO(
			"Joint 1 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_api_origin",
					"jaco_joint_1"));
	/***************************************************/

	/**********************Joint_2**********************/
	/* Joint 2 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get joint 2 rotation matrix
	kinematics.J2_Rotation(
			kinematics.deg_to_rad(
					kinematics.Q2(arm_angles.Actuators.Actuator2)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 2 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 2 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 2 translation vector
	kinematics.J2_Translation(
			kinematics.deg_to_rad(
					kinematics.Q2(arm_angles.Actuators.Actuator2)),
			translation_v);
	/* Display Results */
	ROS_INFO(
			"Joint 2 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_1",
					"jaco_joint_2"));
	/***************************************************/

	/**********************Joint_3**********************/
	/* Joint 3 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get joint 3 rotation matrix
	kinematics.J3_Rotation(
			kinematics.deg_to_rad(kinematics.Q3(arm_angles.Actuators.Actuator3)),
					rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 3 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 3 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 3 translation vector
	kinematics.J3_Translation(
			kinematics.deg_to_rad(
					kinematics.Q3(arm_angles.Actuators.Actuator3)),
			translation_v);
	/* Display Results */
	ROS_INFO(
			"Joint 3 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_2",
					"jaco_joint_3"));
	/***************************************************/

	/**********************Joint_4**********************/
	/* Joint 4 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get joint 4 rotation matrix
	kinematics.J4_Rotation(
			kinematics.deg_to_rad(kinematics.Q4(arm_angles.Actuators.Actuator4)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 4 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 4 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 4 translation vector
	kinematics.J4_Translation(
			kinematics.deg_to_rad(kinematics.Q4(arm_angles.Actuators.Actuator4)),
			translation_v);
	/* Display Results */
	ROS_INFO(
			"Joint 4 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_3",
					"jaco_joint_4"));
	/***************************************************/

	/**********************Joint_5**********************/
	/* Joint 5 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get joint 5 rotation matrix
	kinematics.J5_Rotation(
			kinematics.deg_to_rad(kinematics.Q5(arm_angles.Actuators.Actuator5)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 5 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 5 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 5 translation vector
	kinematics.J5_Translation(
			kinematics.deg_to_rad(kinematics.Q5(arm_angles.Actuators.Actuator5)),
			translation_v);
	/* Display Results */
	ROS_INFO(
			"Joint 5 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_4",
					"jaco_joint_5"));
	/***************************************************/

	/**********************Joint_6**********************/
	/* Joint 6 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation

	//get joint 6 rotation matrix
	kinematics.J6_Rotation(
			kinematics.deg_to_rad(kinematics.Q6(arm_angles.Actuators.Actuator6)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 6 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 6 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 6 translation vector
	kinematics.J6_Translation(
			kinematics.deg_to_rad(kinematics.Q6(arm_angles.Actuators.Actuator6)),
			translation_v);
	/* Display Results */
	ROS_INFO(
			"Joint 6 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_5",
					"jaco_joint_6"));
	/***************************************************/
}

void JacoArm::TimerCallback(const ros::TimerEvent&) {
	this->CalculatePostion();
}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_controller");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string ArmPose("ArmPose"); ///String containing the topic name for cartesian commands

	if (argc < 1) {
		ROS_INFO( "Usage: jaco_arm_controller cartesian_info_topic");
		return 1;
	} else {
		//Grab the topic parameters, print warnings if using default values
		if (!param_nh.getParam(ArmPose, ArmPose))
			ROS_WARN(
					"Parameter <%s> Not Set. Using Default Jaco Position Topic <%s>!", ArmPose.c_str(), ArmPose.c_str());
	}

	//Print out received topics
	ROS_DEBUG("Got Jaco Position Topic Name: <%s>", ArmPose.c_str());

	ROS_INFO("Starting Up Jaco Arm Controller...");

	//create the arm object
	JacoArm jaco(nh, ArmPose);

	jaco.GoHome();

	ros::spin();

}

