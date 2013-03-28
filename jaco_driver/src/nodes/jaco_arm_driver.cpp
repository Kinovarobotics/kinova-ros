//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

/**
 * @file jaco_arm_driver.cpp
 *
 * @date   Feb 20, 2013
 * @author parallels
 * @brief \todo
 */

//License File
#include <jaco_driver/jaco_arm_driver.h>

using namespace std;
using namespace jaco;

JacoArm::JacoArm(ros::NodeHandle nh, std::string ArmPose) {
	ROS_INFO("Initiating Library");
	API = new JacoAPI();
	ROS_INFO("Initiating API");

	int api_result = 0; //stores result from the API

	api_result = (API->InitAPI)();

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

	CalculatePostion();

	this->sub = nh.subscribe(ArmPose, 1, &JacoArm::PoseMSG_Sub, this);
	this->timer = nh.createTimer(ros::Duration(0.1), &JacoArm::TimerCallback,
			this);

}

void JacoArm::SetAngles(AngularInfo angles, int timeout, bool push) {
	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	if (push == true) {
		API->EraseAllTrajectories();
		API->StopControlAPI();
	}
	API->StartControlAPI();

	Jaco_Position.Position.Type = ANGULAR_POSITION;

	Jaco_Position.Position.Actuators.Actuator1 = angles.Actuator1;
	Jaco_Position.Position.Actuators.Actuator2 = angles.Actuator2;
	Jaco_Position.Position.Actuators.Actuator3 = angles.Actuator3;
	Jaco_Position.Position.Actuators.Actuator4 = angles.Actuator4;
	Jaco_Position.Position.Actuators.Actuator5 = angles.Actuator5;
	Jaco_Position.Position.Actuators.Actuator6 = angles.Actuator6;

	API->SendAdvanceTrajectory(Jaco_Position);

	//if we want to timeout
	if (timeout != 0) {
		double start_secs;
		double current_sec;

		//If ros is still runniing use rostime, else use system time
		if (ros::ok()) {
			start_secs = ros::Time::now().toSec();
			current_sec = ros::Time::now().toSec();
		} else {
			start_secs = (double) time(NULL);
			current_sec = (double) time(NULL);
		}

		AngularPosition cur_angles; //holds the current angles of the arm

		const float Angle_Range = 2; //dead zone for angles (degrees)

		bool Joint_1_Reached = false;
		bool Joint_2_Reached = false;
		bool Joint_3_Reached = false;
		bool Joint_4_Reached = false;
		bool Joint_5_Reached = false;
		bool Joint_6_Reached = false;

		//while we have not timed out
		while ((current_sec - start_secs) < timeout) {

			//If ros is still runniing use rostime, else use system time
			if (ros::ok()) {
				current_sec = ros::Time::now().toSec();
			} else {
				current_sec = (double) time(NULL);
			}

			API->GetAngularPosition(cur_angles); //update current arm angles

			//Check if Joint 1 is in range
			if (((cur_angles.Actuators.Actuator1)
					<= Jaco_Position.Position.Actuators.Actuator1 + Angle_Range)
					&& (cur_angles.Actuators.Actuator1)
							>= (Jaco_Position.Position.Actuators.Actuator1
									- Angle_Range)) {
				Joint_1_Reached = true;
			}

			//Check if Joint 2 is in range
			if (((cur_angles.Actuators.Actuator2)
					<= Jaco_Position.Position.Actuators.Actuator2 + Angle_Range)
					&& (cur_angles.Actuators.Actuator2)
							>= (Jaco_Position.Position.Actuators.Actuator2
									- Angle_Range)) {
				Joint_2_Reached = true;
			}

			//Check if Joint 3 is in range
			if (((cur_angles.Actuators.Actuator3)
					<= Jaco_Position.Position.Actuators.Actuator3 + Angle_Range)
					&& (cur_angles.Actuators.Actuator3)
							>= (Jaco_Position.Position.Actuators.Actuator3
									- Angle_Range)) {
				Joint_3_Reached = true;
			}

			//Check if Joint 4 is in range
			if (((cur_angles.Actuators.Actuator4)
					<= Jaco_Position.Position.Actuators.Actuator4 + Angle_Range)
					&& (cur_angles.Actuators.Actuator4)
							>= (Jaco_Position.Position.Actuators.Actuator4
									- Angle_Range)) {
				Joint_4_Reached = true;
			}

			//Check if Joint 5 is in range
			if (((cur_angles.Actuators.Actuator5)
					<= Jaco_Position.Position.Actuators.Actuator5 + Angle_Range)
					&& (cur_angles.Actuators.Actuator5)
							>= (Jaco_Position.Position.Actuators.Actuator5
									- Angle_Range)) {
				Joint_5_Reached = true;
			}

			//Check if Joint 6 is in range
			if (((cur_angles.Actuators.Actuator6)
					<= Jaco_Position.Position.Actuators.Actuator6 + Angle_Range)
					&& (cur_angles.Actuators.Actuator6)
							>= (Jaco_Position.Position.Actuators.Actuator6
									- Angle_Range)) {
				Joint_6_Reached = true;
			}

			//If all the joints reached their destination then break out of timeout loop
			if (Joint_1_Reached == true && Joint_2_Reached == true
					&& Joint_3_Reached == true && Joint_4_Reached == true
					&& Joint_5_Reached == true && Joint_6_Reached == true) {
				break;
			}
		}
	}
}

void JacoArm::SetPosition(CartesianInfo position, int timeout, bool push) {

	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	if (push == true) {

		API->EraseAllTrajectories();
		API->StopControlAPI();
	}
	API->StartControlAPI();

	Jaco_Position.Position.Type = CARTESIAN_POSITION;

	Jaco_Position.Position.CartesianPosition.X = position.X;
	Jaco_Position.Position.CartesianPosition.Y = position.Y;
	Jaco_Position.Position.CartesianPosition.Z = position.Z;
	Jaco_Position.Position.CartesianPosition.ThetaX = position.ThetaX;
	Jaco_Position.Position.CartesianPosition.ThetaY = position.ThetaY;
	Jaco_Position.Position.CartesianPosition.ThetaZ = position.ThetaZ;

	API->SendAdvanceTrajectory(Jaco_Position);

	//if we want to timeout
	if (timeout != 0) {
		double start_secs;
		double current_sec;

		//If ros is still runniing use rostime, else use system time
		if (ros::ok()) {
			start_secs = ros::Time::now().toSec();
			current_sec = ros::Time::now().toSec();
		} else {
			start_secs = (double) time(NULL);
			current_sec = (double) time(NULL);
		}

		CartesianPosition cur_position; //holds the current position of the arm

		const float Postion_Range = 5; //dead zone for position
		const float Rotation_Range = 5; //dead zone for rotation

		bool Position_X_Reached = false;
		bool Position_Y_Reached = false;
		bool Position_Z_Reached = false;
		bool Position_TX_Reached = false;
		bool Position_TY_Reached = false;
		bool Position_TZ_Reached = false;

		//while we have not timed out
		while ((current_sec - start_secs) < timeout) {

			//If ros is still runniing use rostime, else use system time
			if (ros::ok()) {
				current_sec = ros::Time::now().toSec();
			} else {
				current_sec = (double) time(NULL);
			}

			API->GetCartesianPosition(cur_position); //update current arm position

			//Check if X is in range
			if (((cur_position.Coordinates.X)
					<= Jaco_Position.Position.CartesianPosition.X
							+ Postion_Range)
					&& (cur_position.Coordinates.X)
							>= (Jaco_Position.Position.CartesianPosition.X
									- Postion_Range)) {
				Position_X_Reached = true;
			}

			//Check if Y is in range
			if (((cur_position.Coordinates.Y)
					<= Jaco_Position.Position.CartesianPosition.Y
							+ Postion_Range)
					&& (cur_position.Coordinates.Y)
							>= (Jaco_Position.Position.CartesianPosition.Y
									- Postion_Range)) {
				Position_Y_Reached = true;
			}

			//Check if Z is in range
			if (((cur_position.Coordinates.Z)
					<= Jaco_Position.Position.CartesianPosition.Z
							+ Postion_Range)
					&& (cur_position.Coordinates.Z)
							>= (Jaco_Position.Position.CartesianPosition.Z
									- Postion_Range)) {
				Position_Z_Reached = true;
			}

			//Check if ThetaX is in range
			if (((cur_position.Coordinates.ThetaX)
					<= Jaco_Position.Position.CartesianPosition.ThetaX
							+ Rotation_Range)
					&& (cur_position.Coordinates.ThetaX)
							>= (Jaco_Position.Position.CartesianPosition.ThetaX
									- Rotation_Range)) {
				Position_TX_Reached = true;
			}

			//Check if ThetaY is in range
			if (((cur_position.Coordinates.ThetaY)
					<= Jaco_Position.Position.CartesianPosition.ThetaY
							+ Rotation_Range)
					&& (cur_position.Coordinates.ThetaY)
							>= (Jaco_Position.Position.CartesianPosition.ThetaY
									- Rotation_Range)) {
				Position_TY_Reached = true;
			}

			//Check if ThetaZ is in range
			if (((cur_position.Coordinates.ThetaZ)
					<= Jaco_Position.Position.CartesianPosition.ThetaZ
							+ Rotation_Range)
					&& (cur_position.Coordinates.ThetaZ)
							>= (Jaco_Position.Position.CartesianPosition.ThetaZ
									- Rotation_Range)) {
				Position_TZ_Reached = true;
			}

			//If the arm reaches it's destination then break out of timeout loop
			if (Position_X_Reached == true && Position_Y_Reached == true
					&& Position_Z_Reached == true && Position_TX_Reached == true
					&& Position_TY_Reached == true
					&& Position_TZ_Reached == true) {
				break;
			}
		}
	}
}

void JacoArm::SetFingers(FingersPosition fingers, int timeout, bool push) {

	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	if (push == true) {
		API->EraseAllTrajectories();

		API->StopControlAPI();
	}

	API->StartControlAPI();

	Jaco_Position.Position.HandMode = POSITION_MODE;

	Jaco_Position.Position.Fingers.Finger1 = fingers.Finger1;
	Jaco_Position.Position.Fingers.Finger2 = fingers.Finger2;
	Jaco_Position.Position.Fingers.Finger3 = fingers.Finger3;

	API->SendAdvanceTrajectory(Jaco_Position);

	//if we want to timeout
	if (timeout != 0) {
		double start_secs;
		double current_sec;

		//If ros is still runniing use rostime, else use system time
		if (ros::ok()) {
			start_secs = ros::Time::now().toSec();
			current_sec = ros::Time::now().toSec();
		} else {
			start_secs = (double) time(NULL);
			current_sec = (double) time(NULL);
		}

		FingersPosition cur_fingers; //holds the current position of the fingers
		const float finger_range = 5; //dead zone for fingers
		bool Finger_1_Reached = false;
		bool Finger_2_Reached = false;
		bool Finger_3_Reached = false;

		//while we have not timed out
		while ((current_sec - start_secs) < timeout) {

			//If ros is still runniing use rostime, else use system time
			if (ros::ok()) {
				current_sec = ros::Time::now().toSec();
			} else {
				current_sec = (double) time(NULL);
			}

			GetFingers(cur_fingers); //update current finger position

			//Check if finger is in range
			if (((cur_fingers.Finger1)
					<= Jaco_Position.Position.Fingers.Finger1 + finger_range)
					&& (cur_fingers.Finger1)
							>= (Jaco_Position.Position.Fingers.Finger1
									- finger_range)) {
				Finger_1_Reached = true;
			}

			//Check if finger is in range
			if (((cur_fingers.Finger2)
					<= Jaco_Position.Position.Fingers.Finger2 + finger_range)
					&& (cur_fingers.Finger2)
							>= (Jaco_Position.Position.Fingers.Finger2
									- finger_range)) {
				Finger_2_Reached = true;
			}

			//Check if finger is in range
			if (((cur_fingers.Finger3)
					<= Jaco_Position.Position.Fingers.Finger3 + finger_range)
					&& (cur_fingers.Finger3)
							>= (Jaco_Position.Position.Fingers.Finger3
									- finger_range)) {
				Finger_3_Reached = true;
			}

			//If all the fingers reached their destination then break out of timeout loop
			if (Finger_1_Reached == true && Finger_2_Reached == true
					&& Finger_3_Reached == true) {
				break;
			}
		}
	}
}

void JacoArm::GetAngles(AngularInfo &angles) {
	AngularPosition Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	API->GetAngularPosition(Jaco_Position);

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
	API->GetCartesianPosition(Jaco_Position);

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
	API->GetCartesianPosition(Jaco_Position);

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

	geometry_msgs::PoseStamped api_pose;
	ROS_INFO("Raw MSG");
	ROS_INFO("X = %f", arm_pose->pose.position.x);
	ROS_INFO("Y = %f", arm_pose->pose.position.y);
	ROS_INFO("Z = %f", arm_pose->pose.position.z);

	ROS_INFO("RX = %f", arm_pose->pose.orientation.x);
	ROS_INFO("RY = %f", arm_pose->pose.orientation.y);
	ROS_INFO("RZ = %f", arm_pose->pose.orientation.z);
	ROS_INFO("RW = %f", arm_pose->pose.orientation.w);


	listener.waitForTransform("/jaco_api_origin", arm_pose->header.frame_id, arm_pose->header.stamp, ros::Duration(3.0) );

	listener.transformPose("/jaco_api_origin", *arm_pose, api_pose);

	ROS_INFO("Transformed MSG");
	ROS_INFO("X = %f", api_pose.pose.position.x);
	ROS_INFO("Y = %f", api_pose.pose.position.y);
	ROS_INFO("Z = %f", api_pose.pose.position.z);

	ROS_INFO("RX = %f", api_pose.pose.orientation.x);
	ROS_INFO("RY = %f", api_pose.pose.orientation.y);
	ROS_INFO("RZ = %f", api_pose.pose.orientation.z);
	ROS_INFO("RW = %f", api_pose.pose.orientation.w);

	double x, y, z;
	tf::Quaternion q;
	tf::quaternionMsgToTF(api_pose.pose.orientation, q);

	tf::Matrix3x3 bt_q(q);

	bt_q.getEulerYPR(z, y, x);

	Jaco_Position.X = (float) api_pose.pose.position.x;
	Jaco_Position.Y = (float) api_pose.pose.position.y;
	Jaco_Position.Z = (float) api_pose.pose.position.z;

	Jaco_Position.ThetaX = (float) x;
	Jaco_Position.ThetaY = (float) y;
	Jaco_Position.ThetaZ = (float) z;

	this->PrintPosition(Jaco_Position);
	this->SetPosition(Jaco_Position);

}

void JacoArm::GoHome(void) {
	FingersPosition fingers_home = { 40, 40, 40 };
	AngularInfo joint_home = { 270.385651, 150.203217, 24, 267.597351, 5.570505,
			99.634575 };

	this->SetFingers(fingers_home, 5); //send fingers to home position
	this->SetAngles(joint_home, 5); //send joints to home position
}

void JacoArm::CalculatePostion(void) {
	AngularPosition arm_angles;
	memset(&arm_angles, 0, sizeof(arm_angles)); //zero structure

#ifndef DEBUG_WITHOUT_ARM
	API->GetAngularPosition(arm_angles); //Query arm for joint angles
#else
			//Populate with dummy values
			arm_angles.Actuators.Actuator1 = 30;
			arm_angles.Actuators.Actuator2 = 30;
			arm_angles.Actuators.Actuator3 = 0;
			arm_angles.Actuators.Actuator4 = 0;
			arm_angles.Actuators.Actuator5 = 0;
			arm_angles.Actuators.Actuator6 = 0;
#endif

	//ROS_INFO("Joint 1 = %f", arm_angles.Actuators.Actuator1);
	//ROS_INFO("Joint 2 = %f", arm_angles.Actuators.Actuator2);
	//ROS_INFO("Joint 3 = %f", arm_angles.Actuators.Actuator3);

	//ROS_INFO("Joint 4 = %f", arm_angles.Actuators.Actuator4);
	//ROS_INFO("Joint 5 = %f", arm_angles.Actuators.Actuator5);
	//ROS_INFO("Joint 6 = %f", arm_angles.Actuators.Actuator6);


	//Update the forward Kinematics
	kinematics.UpdateForward(
			kinematics.deg_to_rad(arm_angles.Actuators.Actuator1),
			kinematics.deg_to_rad(arm_angles.Actuators.Actuator2),
			kinematics.deg_to_rad(arm_angles.Actuators.Actuator3),
			kinematics.deg_to_rad(arm_angles.Actuators.Actuator4),
			kinematics.deg_to_rad(arm_angles.Actuators.Actuator5),
			kinematics.deg_to_rad(arm_angles.Actuators.Actuator6));

}
void JacoArm::TimerCallback(const ros::TimerEvent&) {
	this->CalculatePostion();	//Update TF Tree
}

int main(int argc, char **argv) {

	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_driver");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	std::string ArmPose("ArmPose"); ///String containing the topic name for cartesian commands

	if (argc < 1) {
		ROS_INFO( "Usage: jaco_arm_driver cartesian_info_topic");
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

	ros::spin();
	jaco.GoHome();
}

