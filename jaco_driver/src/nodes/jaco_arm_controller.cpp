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

	this->sub = nh.subscribe(ArmPose, 1, &JacoArm::PoseMSG_Sub, this);
	this->timer = nh.createTimer(ros::Duration(0.1), &JacoArm::TimerCallback,
			this);

}

void JacoArm::SetAngles(AngularInfo angles) {
	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	Jaco_EraseAllTrajectories();
	Jaco_StopControlAPI();

	Jaco_StartControlAPI();

	Jaco_Position.Position.Type = ANGULAR_POSITION;

	Jaco_Position.Position.Actuators.Actuator1 = angles.Actuator1;
	Jaco_Position.Position.Actuators.Actuator2 = angles.Actuator2;
	Jaco_Position.Position.Actuators.Actuator3 = angles.Actuator3;
	Jaco_Position.Position.Actuators.Actuator4 = angles.Actuator4;
	Jaco_Position.Position.Actuators.Actuator5 = angles.Actuator5;
	Jaco_Position.Position.Actuators.Actuator6 = angles.Actuator6;

	Jaco_SendBasicTrajectory(Jaco_Position);
}

void JacoArm::SetPosition(CartesianInfo position) {

	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	Jaco_EraseAllTrajectories();
	Jaco_StopControlAPI();

	Jaco_StartControlAPI();

	Jaco_Position.Position.Type = CARTESIAN_POSITION;

	Jaco_Position.Position.CartesianPosition.X = position.X;
	Jaco_Position.Position.CartesianPosition.Y = position.Y;
	Jaco_Position.Position.CartesianPosition.Z = position.Z;
	Jaco_Position.Position.CartesianPosition.ThetaX = position.ThetaX;
	Jaco_Position.Position.CartesianPosition.ThetaY = position.ThetaY;
	Jaco_Position.Position.CartesianPosition.ThetaZ = position.ThetaZ;

	Jaco_SendBasicTrajectory(Jaco_Position);

}

void JacoArm::SetFingers(FingersPosition fingers) {

	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	Jaco_EraseAllTrajectories();
	Jaco_StopControlAPI();

	Jaco_StartControlAPI();

	Jaco_Position.Position.HandMode = POSITION_MODE;

	Jaco_Position.Position.Fingers.Finger1 = fingers.Finger1;
	Jaco_Position.Position.Fingers.Finger2 = fingers.Finger2;
	Jaco_Position.Position.Fingers.Finger3 = fingers.Finger3;

	Jaco_SendBasicTrajectory(Jaco_Position);

}

void JacoArm::GetAngles(AngularInfo &angles) {
	AngularPosition Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	Jaco_GetAngularPosition(Jaco_Position);

	angles.Actuator1 = Jaco_Position.Actuators.Actuator1;
	angles.Actuator2 = Jaco_Position.Actuators.Actuator2;
	angles.Actuator3 = Jaco_Position.Actuators.Actuator3;
	angles.Actuator4 = Jaco_Position.Actuators.Actuator4;
	angles.Actuator5 = Jaco_Position.Actuators.Actuator5;
	angles.Actuator6 = Jaco_Position.Actuators.Actuator6;

}

void JacoArm::GetPosition(CartesianInfo &position) {
	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	Jaco_GetCartesianPosition(Jaco_Position);

	position.X = Jaco_Position.Coordinates.X;
	position.Y = Jaco_Position.Coordinates.Y;
	position.Z = Jaco_Position.Coordinates.Z;
	position.ThetaX = Jaco_Position.Coordinates.ThetaX;
	position.ThetaY = Jaco_Position.Coordinates.ThetaY;
	position.ThetaZ = Jaco_Position.Coordinates.ThetaZ;

}

void JacoArm::GetFingers(FingersPosition &fingers) {
	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	Jaco_GetCartesianPosition(Jaco_Position);

	fingers.Finger1 = Jaco_Position.Fingers.Finger1;
	fingers.Finger2 = Jaco_Position.Fingers.Finger2;
	fingers.Finger3 = Jaco_Position.Fingers.Finger3;

}

void JacoArm::PrintAngles(AngularInfo angles) {

	ROS_INFO("Jaco Arm Angles (Degrees)");
	ROS_INFO("Joint 1 = %f", angles.Actuator1);
	ROS_INFO("Joint 2 = %f", angles.Actuator2);
	ROS_INFO("Joint 3 = %f", angles.Actuator3);

	ROS_INFO("Joint 4 = %f", angles.Actuator4);
	ROS_INFO("Joint 5 = %f", angles.Actuator5);
	ROS_INFO("Joint 6 = %f", angles.Actuator6);

}

void JacoArm::PrintPosition(CartesianInfo position) {

	ROS_INFO("Jaco Arm Position (Meters)");
	ROS_INFO("X = %f", position.X);
	ROS_INFO("Y = %f", position.Y);
	ROS_INFO("Z = %f", position.Z);

	ROS_INFO("Jaco Arm Rotations (Radians)");
	ROS_INFO("Theta X = %f", position.ThetaX);
	ROS_INFO("Theta Y = %f", position.ThetaY);
	ROS_INFO("Theta Z = %f", position.ThetaZ);

}

void JacoArm::PrintFingers(FingersPosition fingers) {

	ROS_INFO("Jaco Arm Finger Positions");
	ROS_INFO("Finger 1 = %f", fingers.Finger1);
	ROS_INFO("Finger 2 = %f", fingers.Finger2);
	ROS_INFO("Finger 3 = %f", fingers.Finger3);

}

void JacoArm::PoseMSG_Sub(const geometry_msgs::PoseStampedConstPtr& arm_pose) {
	CartesianInfo Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	double x, y, z;
	tf::Quaternion q;
	tf::quaternionMsgToTF(arm_pose->pose.orientation, q);

	tf::Matrix3x3 bt_q(q);

	bt_q.getEulerYPR(z, y, x);

	Jaco_Position.X = (float) arm_pose->pose.position.x;
	Jaco_Position.Y = (float) arm_pose->pose.position.y;
	Jaco_Position.Z = (float) arm_pose->pose.position.z;

	Jaco_Position.ThetaX = (float) x;
	Jaco_Position.ThetaY = (float) y;
	Jaco_Position.ThetaZ = (float) z;

	this->PrintPosition(Jaco_Position);
	this->SetPosition(Jaco_Position);

}

void JacoArm::GoHome(void) {
	FingersPosition fingers_home = { 53, 53, 53 };
	AngularInfo joint_home = { 270.385651, 150.203217, 26.541765, 267.597351,
			5.570505, 99.634575 };

	this->SetFingers(fingers_home); //send fingers to home position
	this->SetAngles(joint_home); //send joints to home position
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
			kinematics.deg_to_rad(
					kinematics.Q3(arm_angles.Actuators.Actuator3)), rotation_q);

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
			kinematics.deg_to_rad(
					kinematics.Q4(arm_angles.Actuators.Actuator4)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 4 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 4 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 4 translation vector
	kinematics.J4_Translation(
			kinematics.deg_to_rad(
					kinematics.Q4(arm_angles.Actuators.Actuator4)),
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
			kinematics.deg_to_rad(
					kinematics.Q5(arm_angles.Actuators.Actuator5)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 5 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 5 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 5 translation vector
	kinematics.J5_Translation(
			kinematics.deg_to_rad(
					kinematics.Q5(arm_angles.Actuators.Actuator5)),
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
			kinematics.deg_to_rad(
					kinematics.Q6(arm_angles.Actuators.Actuator6)), rotation_q);

	/* Display Results */
	ROS_INFO(
			"Joint 6 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());

	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 6 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 6 translation vector
	kinematics.J6_Translation(
			kinematics.deg_to_rad(
					kinematics.Q6(arm_angles.Actuators.Actuator6)),
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
	this->CalculatePostion();	//Update TF Tree
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

