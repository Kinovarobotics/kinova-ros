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

JacoArm::JacoArm(ros::NodeHandle nh, ros::NodeHandle param_nh)
{

	std::string arm_pose_topic("arm_pose_topic"); ///String containing the topic name for Cartesian commands
	std::string joint_velocity_topic("joint_velocity_topic"); ///String containing the topic name for JointVelocity
	std::string joint_angles_topic("joint_angles_topic"); ///String containing the topic name for JointAngles
	std::string cartesian_velocity_topic("cartesian_velocity_topic"); ///String containing the topic name for CartesianVelocity
	std::string tool_position_topic("tool_position_topic"); ///String containing the topic name for ToolPosition
	std::string set_finger_position_topic("set_finger_position_topic"); ///String containing the topic name for SetFingerPosition
	std::string finger_position_topic("finger_position_topic"); ///String containing the topic name for FingerPosition
	std::string software_pause_topic("software_pause_topic"); ///String containing the topic name for SoftwarePause
	std::string aero_state("aero/supervisor/state"); ///String containing the topic name for aero_state
	std::string zero_arm_req("zero_arm_req"); ///String containing the topic name for zero_arm_req
	std::string zero_arm_stat("zero_arm_stat"); ///String containing the topic name for zero_arm_stat

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(arm_pose_topic, arm_pose_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Jaco Position Topic <%s>!", arm_pose_topic.c_str(),
				arm_pose_topic.c_str());
	if (!param_nh.getParam(joint_velocity_topic, joint_velocity_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Joint Velocity Topic <%s>!",
				joint_velocity_topic.c_str(), joint_velocity_topic.c_str());
	if (!param_nh.getParam(joint_angles_topic, joint_angles_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Joint Angles Topic <%s>!", joint_angles_topic.c_str(),
				joint_angles_topic.c_str());
	if (!param_nh.getParam(cartesian_velocity_topic, cartesian_velocity_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Cartesian Velocity Topic <%s>!",
				cartesian_velocity_topic.c_str(), cartesian_velocity_topic.c_str());
	if (!param_nh.getParam(tool_position_topic, tool_position_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Tool Position Topic <%s>!",
				tool_position_topic.c_str(), tool_position_topic.c_str());
	if (!param_nh.getParam(set_finger_position_topic, set_finger_position_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Set Finger Position Topic <%s>!",
				set_finger_position_topic.c_str(), set_finger_position_topic.c_str());
	if (!param_nh.getParam(finger_position_topic, finger_position_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Finger Position Topic <%s>!",
				finger_position_topic.c_str(), finger_position_topic.c_str());
	if (!param_nh.getParam(software_pause_topic, software_pause_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Software Pause Topic <%s>!",
				software_pause_topic.c_str(), software_pause_topic.c_str());
	if (!param_nh.getParam(aero_state, aero_state))
		ROS_WARN("Parameter <%s> Not Set. Using Default Aero State Topic <%s>!", aero_state.c_str(),
				aero_state.c_str());
	if (!param_nh.getParam(zero_arm_req, zero_arm_req))
		ROS_WARN("Parameter <%s> Not Set. Using Default zero_arm_req Topic <%s>!", zero_arm_req.c_str(),
				zero_arm_req.c_str());
	if (!param_nh.getParam(zero_arm_stat, zero_arm_stat))
		ROS_WARN("Parameter <%s> Not Set. Using Default zero_arm_stat Topic <%s>!", zero_arm_stat.c_str(),
				zero_arm_stat.c_str());

//Print out received topics
	ROS_DEBUG("Got Jaco Position Topic Name: <%s>", arm_pose_topic.c_str());
	ROS_DEBUG("Got Joint Velocity Topic Name: <%s>", joint_velocity_topic.c_str());
	ROS_DEBUG("Got Joint Angles Topic Name: <%s>", joint_angles_topic.c_str());
	ROS_DEBUG("Got Cartesian Velocity Topic Name: <%s>", cartesian_velocity_topic.c_str());
	ROS_DEBUG("Got Tool Position Topic Name: <%s>", tool_position_topic.c_str());
	ROS_DEBUG("Got Set Finger Position Topic Name: <%s>", set_finger_position_topic.c_str());
	ROS_DEBUG("Got Finger Position Topic Name: <%s>", finger_position_topic.c_str());
	ROS_DEBUG("Got SoftwarePause Topic Name: <%s>", software_pause_topic.c_str());

	ROS_INFO("Starting Up Jaco Arm Controller...");

	this->software_pause = false;
	previous_state = 0;

	this->SoftwarePause_sub = nh.subscribe(software_pause_topic, 1, &JacoArm::SoftwarePauseMSG, this);

	/* Connecting to Jaco Arm */
	ROS_INFO("Initiating Library");
	API = new JacoAPI();
	ROS_INFO("Initiating API");
	last_update_time = ros::Time::now();
	update_time = ros::Duration(5.0);
	int api_result = 0; //stores result from the API
	ros::Duration(5.0).sleep();

	api_result = (API->InitAPI)();

	if (api_result != 1)
	{
		/* Failed to contact arm */
		ROS_FATAL("Could not initialize arm");
		ROS_FATAL("Jaco_InitAPI returned: %d", api_result);
#ifndef DEBUG_WITHOUT_ARM
		ros::shutdown();
#endif
	} else
	{
		ROS_INFO("API Initialized Successfully!");
	}
	ros::Duration(2.0).sleep();

	/* Set Default Configuration */

	API->RestoreFactoryDefault();


	ClientConfigurations configuration;
	GetConfig(configuration);
	PrintConfig(configuration);
	//configuration.RetractedPositionCount = 0;
//	configuration.MaxForce = 5;

	//SetConfig(configuration);
//		 while (!HomeState() && ros::ok())
//		{API->EraseAllTrajectories();
//		API->StopControlAPI();
//
//			ROS_INFO("Off");
//
//			memset(&Jaco_Angles, 0, sizeof(Jaco_Angles)); //zero structure
//			API->GetAngularPosition(Jaco_Angles);
//
//			if ((Jaco_Angles.Actuators.Actuator1 <= (Old_Jaco_Angles.Actuators.Actuator1 + 1))
//					&& (Jaco_Angles.Actuators.Actuator1 >= (Old_Jaco_Angles.Actuators.Actuator1 - 1))
//					&& (Jaco_Angles.Actuators.Actuator2 <= (Old_Jaco_Angles.Actuators.Actuator2 + 1))
//					&& (Jaco_Angles.Actuators.Actuator2 >= (Old_Jaco_Angles.Actuators.Actuator2 - 1))
//					&& (Jaco_Angles.Actuators.Actuator3 <= (Old_Jaco_Angles.Actuators.Actuator3 + 1))
//					&& (Jaco_Angles.Actuators.Actuator3 >= (Old_Jaco_Angles.Actuators.Actuator3 - 1))
//					&& (Jaco_Angles.Actuators.Actuator4 <= (Old_Jaco_Angles.Actuators.Actuator4 + 1))
//					&& (Jaco_Angles.Actuators.Actuator4 >= (Old_Jaco_Angles.Actuators.Actuator4 - 1))
//					&& (Jaco_Angles.Actuators.Actuator5 <= (Old_Jaco_Angles.Actuators.Actuator5 + 1))
//					&& (Jaco_Angles.Actuators.Actuator5 >= (Old_Jaco_Angles.Actuators.Actuator5 - 1))
//					&& (Jaco_Angles.Actuators.Actuator6 <= (Old_Jaco_Angles.Actuators.Actuator6 + 1))
//					&& (Jaco_Angles.Actuators.Actuator6 >= (Old_Jaco_Angles.Actuators.Actuator6 - 1)))
//			{
//				API->StartControlAPI();
//
//				home_command.ButtonValue[0] = 1;
//				API->SendJoystickCommand(home_command);
//				ROS_INFO("On");
////				ros::Duration(1.0).sleep();
////				home_command.ButtonValue[0] = 0;
////				API->SendJoystickCommand(home_command);
////				ROS_INFO("Off");
//
//			}
//
//			home_command.ButtonValue[2] = 1;
//			API->SendJoystickCommand(home_command);
//			ROS_INFO("On");
//			ros::Duration(15.0).sleep();
//			home_command.ButtonValue[2] = 0;
//			API->SendJoystickCommand(home_command);
//			ROS_INFO("Off");
//
//			if (!HomeState())
//			{
//				API->StartControlAPI();
//
//				home_command.ButtonValue[2] = 1;
//				API->SendJoystickCommand(home_command);
//				ROS_INFO("On");
//				ros::Duration(15.0).sleep();
//				home_command.ButtonValue[2] = 0;
//				API->SendJoystickCommand(home_command);
//				ROS_INFO("Off");
//			}
//
//		}
//
//		for (int i = 0; i < 16; i++)
//				{
//			for(int x=0;x<1;x++)
//			{
//			ROS_INFO("Trying %d...",i);
//					home_command.ButtonValue[i] = 1;
//										API->SendJoystickCommand(home_command);
//										ROS_INFO("On");
//
//					ros::Duration(2.0).sleep();
//					home_command.ButtonValue[i] = 0;
//					API->SendJoystickCommand(home_command);
//					ROS_INFO("Off");
//
//					ros::Duration(2.0).sleep();
//			}
//				}
//	tf::Transform transform;
//	tf::Quaternion rotation_q(0, 0, 0, 0);
//	tf::Vector3 translation_v(0, 0, 0);
//	//API->EraseAllTrajectories();
//
//	API->StartControlAPI();
//
//	ros::Duration(2.0).sleep();
//
//	if (!HomeState())
//	{
//		ros::Duration(2.0).sleep();
//
//		ROS_INFO("Homing Arm...");
//
//		/* Homing the Arm */
//		double start_secs;
//		double current_sec;
//
//		//If ros is still running use rostime, else use system time
//		if (ros::ok())
//		{
//			start_secs = ros::Time::now().toSec();
//			current_sec = ros::Time::now().toSec();
//		} else
//		{
//			start_secs = (double) time(NULL);
//			current_sec = (double) time(NULL);
//		}
//		JoystickCommand home_command;
//		memset(&home_command, 0, sizeof(home_command)); //zero structure
//		for (int i = 0; i < 16; i++)
//		{
//			home_command.ButtonValue[i] = 0;
//		}
//
//		home_command.InclineForwardBackward = 0;
//		home_command.InclineLeftRight = 0;
//		home_command.MoveForwardBackward = 0;
//		home_command.MoveLeftRight = 0;
//		home_command.PushPull = 0;
//		home_command.Rotate = 0;
//
//		home_command.ButtonValue[2] = 1;
//		home_command.ButtonValue[3] = 1;
//
//					API->SendJoystickCommand(home_command);
//					ROS_INFO("Sending Command");
//
//			//while we have not timed out
//			while ((current_sec - start_secs) < 20)
//			{
//				//If ros is still running use rostime, else use system time
//				if (ros::ok())
//				{
//					current_sec = ros::Time::now().toSec();
//				} else
//				{
//					current_sec = (double) time(NULL);
//				}
//
//				if (software_pause != true)
//				{
//					//ros::Duration(1.0).sleep();
//
//
//
//					ros::Duration(1.0).sleep();
//
//
//				} else
//				{
////TODO Pause
//				}
//				ros::spinOnce();
//
//			}
//
//			API->StopControlAPI();
//
//			home_command.ButtonValue[3] = 0;
//
//			home_command.ButtonValue[2] = 0;
//			API->SendJoystickCommand(home_command);
//
//	} else
//	{
//		ROS_INFO("Arm already in Home State.");
//	}
//	ROS_INFO("Homing Fingers...");
//
//	/* Homing the Fingers */
//
//	ros::Duration(10.0).sleep();
////		/* Homing the Arm */
//			double start_secs;
//			double current_sec;
//
//			//If ros is still running use rostime, else use system time
//			if (ros::ok())
//			{
//				start_secs = ros::Time::now().toSec();
//				current_sec = ros::Time::now().toSec();
//			} else
//			{
//				start_secs = (double) time(NULL);
//				current_sec = (double) time(NULL);
//			}
//			JoystickCommand finger_command;
//
//			//while we have not timed out
//			while ((current_sec - start_secs) < 5)
//			{
//				//If ros is still running use rostime, else use system time
//				if (ros::ok())
//				{
//					current_sec = ros::Time::now().toSec();
//				} else
//				{
//					current_sec = (double) time(NULL);
//				}
//
//				if (software_pause != true)
//				{
//					finger_command.ButtonValue[7] = 1;
//					finger_command.MoveLeftRight = 5;
//					API->SendJoystickCommand(finger_command);
//				} else
//				{
//	//TODO Pause
//				}
//				ros::spinOnce();
//
//			}
//			finger_command.ButtonValue[7] = 0;
//			API->SendJoystickCommand(finger_command);
	ZeroArm();

	/* Storing arm in home position */

	this->GoHome();
	this->aero_state_sub = nh.subscribe(aero_state, 1, &JacoArm::AeroStateMSG, this);

	/* Set up Publishers */
	this->JointAngles_pub = nh.advertise<jaco_driver::joint_angles>(joint_angles_topic, 2);
	this->ToolPosition_pub = nh.advertise<geometry_msgs::PoseStamped>(tool_position_topic, 2);
	this->FingerPosition_pub = nh.advertise<jaco_driver::finger_position>(finger_position_topic, 2);
	this->ZeroArm_pub = nh.advertise<jaco_driver::zero_arm>(zero_arm_stat, 1, true);

	/* Set up Subscribers*/
	this->ArmPose_sub = nh.subscribe(arm_pose_topic, 1, &JacoArm::PoseMSG_Sub, this);
	this->JointVelocity_sub = nh.subscribe(joint_velocity_topic, 1, &JacoArm::VelocityMSG, this);
	this->CartesianVelocity_sub = nh.subscribe(cartesian_velocity_topic, 1, &JacoArm::CartesianVelocityMSG,
			this);
	this->SetFingerPosition_sub = nh.subscribe(set_finger_position_topic, 1, &JacoArm::SetFingerPositionMSG,
			this);

	this->status_timer = nh.createTimer(ros::Duration(0.05), &JacoArm::StatusTimer, this);

	this->joint_vel_timer = nh.createTimer(ros::Duration(0.01), &JacoArm::JointVelTimer, this);
	joint_vel_timer.stop();
	joint_vel_timer_flag = false;
	this->cartesian_vel_timer = nh.createTimer(ros::Duration(0.01), &JacoArm::CartesianVelTimer, this);
	cartesian_vel_timer.stop();
	cartesian_vel_timer_flag = false;

	this->ZeroArm_sub = nh.subscribe(zero_arm_req, 1, &JacoArm::ZeroArmMSG, this);

	BroadCastAngles();
	ROS_INFO("Arm ready to use.");

	TrajectoryPoint Jaco_Velocity;

	memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure
	API->StartControlAPI();
	Jaco_Velocity.Position.Type = CARTESIAN_VELOCITY;

	API->SendAdvanceTrajectory(Jaco_Velocity);
}

void JacoArm::AeroStateMSG(const aero_srr_msgs::AeroStateConstPtr& aero_state)
{
	ROS_INFO("State = %d", aero_state->state);
	switch (aero_state->state)
	{

		case aero_srr_msgs::AeroState::PICKUP:

			break;
		default:

			if (previous_state == aero_srr_msgs::AeroState::PICKUP)
			{
				this->GoHome();

			}
			break;

	}
	previous_state = aero_state->state;

}

bool JacoArm::HomeState(void)
{
	AngularPosition Old_Jaco_Angles;
	AngularPosition Jaco_Angles;
	ROS_INFO("Checking Homing State...");
	memset(&Old_Jaco_Angles, 0, sizeof(Old_Jaco_Angles)); //zero structure
	memset(&Jaco_Angles, 0, sizeof(Jaco_Angles)); //zero structure
	API->GetAngularPosition(Jaco_Angles);

	Old_Jaco_Angles.Actuators.Actuator1 = Jaco_Angles.Actuators.Actuator1;
	Old_Jaco_Angles.Actuators.Actuator2 = Jaco_Angles.Actuators.Actuator2;
	Old_Jaco_Angles.Actuators.Actuator3 = Jaco_Angles.Actuators.Actuator3;
	Old_Jaco_Angles.Actuators.Actuator4 = Jaco_Angles.Actuators.Actuator4;
	Old_Jaco_Angles.Actuators.Actuator5 = Jaco_Angles.Actuators.Actuator5;
	Old_Jaco_Angles.Actuators.Actuator6 = Jaco_Angles.Actuators.Actuator6;

	Jaco_Angles.Actuators.Actuator3 += 5;

	SetAngles(Jaco_Angles.Actuators, 5);
	ros::Duration(1.0).sleep();

	memset(&Jaco_Angles, 0, sizeof(Jaco_Angles)); //zero structure
	API->GetAngularPosition(Jaco_Angles);
	ros::Duration(1.0).sleep();

	SetAngles(Old_Jaco_Angles.Actuators, 5);
	ros::Duration(1.0).sleep();
	if ((Jaco_Angles.Actuators.Actuator3 > (Old_Jaco_Angles.Actuators.Actuator3 + 1))
			|| (Jaco_Angles.Actuators.Actuator3 < (Old_Jaco_Angles.Actuators.Actuator3 - 1)))
	{
		return true;
	} else
	{
		return false;
	}
}

void JacoArm::SetConfig(ClientConfigurations config)
{
	API->SetClientConfigurations(config);
}

void JacoArm::ZeroArm(void)
{

	API->StartControlAPI();

	JoystickCommand home_command;
	memset(&home_command, 0, sizeof(home_command)); //zero structure

	home_command.ButtonValue[2] = 1;
	API->SendJoystickCommand(home_command);
	ros::Duration(20.0).sleep();
	home_command.ButtonValue[2] = 0;
	API->SendJoystickCommand(home_command);
	FingersPosition fingers_home;

	fingers_home.Finger1 = 0;
	fingers_home.Finger2 = 0;
	fingers_home.Finger3 = 0;

	this->SetFingers(fingers_home, 5); //send fingers to home position
	ros::Duration(5.0).sleep();
	fingers_home.Finger1 = 40;
	fingers_home.Finger2 = 40;
	fingers_home.Finger3 = 40;

	this->SetFingers(fingers_home, 5); //send fingers to home position
	ros::Duration(5.0).sleep();
}

void JacoArm::SetAngles(AngularInfo angles, int timeout, bool push)
{
	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Position;

		memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
		if (push == true)
		{
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
		if (timeout != 0)
		{
			double start_secs;
			double current_sec;

			//If ros is still running use rostime, else use system time
			if (ros::ok())
			{
				start_secs = ros::Time::now().toSec();
				current_sec = ros::Time::now().toSec();
			} else
			{
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
			while ((current_sec - start_secs) < timeout)
			{

				//If ros is still running use rostime, else use system time
				if (ros::ok())
				{
					current_sec = ros::Time::now().toSec();
				} else
				{
					current_sec = (double) time(NULL);
				}

				API->GetAngularPosition(cur_angles); //update current arm angles

				//Check if Joint 1 is in range
				if (((cur_angles.Actuators.Actuator1)
						<= Jaco_Position.Position.Actuators.Actuator1 + Angle_Range)
						&& (cur_angles.Actuators.Actuator1)
								>= (Jaco_Position.Position.Actuators.Actuator1 - Angle_Range))
				{
					Joint_1_Reached = true;
				}

				//Check if Joint 2 is in range
				if (((cur_angles.Actuators.Actuator2)
						<= Jaco_Position.Position.Actuators.Actuator2 + Angle_Range)
						&& (cur_angles.Actuators.Actuator2)
								>= (Jaco_Position.Position.Actuators.Actuator2 - Angle_Range))
				{
					Joint_2_Reached = true;
				}

				//Check if Joint 3 is in range
				if (((cur_angles.Actuators.Actuator3)
						<= Jaco_Position.Position.Actuators.Actuator3 + Angle_Range)
						&& (cur_angles.Actuators.Actuator3)
								>= (Jaco_Position.Position.Actuators.Actuator3 - Angle_Range))
				{
					Joint_3_Reached = true;
				}

				//Check if Joint 4 is in range
				if (((cur_angles.Actuators.Actuator4)
						<= Jaco_Position.Position.Actuators.Actuator4 + Angle_Range)
						&& (cur_angles.Actuators.Actuator4)
								>= (Jaco_Position.Position.Actuators.Actuator4 - Angle_Range))
				{
					Joint_4_Reached = true;
				}

				//Check if Joint 5 is in range
				if (((cur_angles.Actuators.Actuator5)
						<= Jaco_Position.Position.Actuators.Actuator5 + Angle_Range)
						&& (cur_angles.Actuators.Actuator5)
								>= (Jaco_Position.Position.Actuators.Actuator5 - Angle_Range))
				{
					Joint_5_Reached = true;
				}

				//Check if Joint 6 is in range
				if (((cur_angles.Actuators.Actuator6)
						<= Jaco_Position.Position.Actuators.Actuator6 + Angle_Range)
						&& (cur_angles.Actuators.Actuator6)
								>= (Jaco_Position.Position.Actuators.Actuator6 - Angle_Range))
				{
					Joint_6_Reached = true;
				}

				//If all the joints reached their destination then break out of timeout loop
				if (Joint_1_Reached == true && Joint_2_Reached == true && Joint_3_Reached == true
						&& Joint_4_Reached == true && Joint_5_Reached == true && Joint_6_Reached == true)
				{
					break;
				}
			}

		}
	}
}

void JacoArm::SetPosition(CartesianInfo position, int timeout, bool push)
{
	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Position;

		memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

		if (push == true)
		{

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
		if (timeout != 0)
		{
			double start_secs;
			double current_sec;

			//If ros is still running use rostime, else use system time
			if (ros::ok())
			{
				start_secs = ros::Time::now().toSec();
				current_sec = ros::Time::now().toSec();
			} else
			{
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
			while ((current_sec - start_secs) < timeout)
			{

				//If ros is still runniing use rostime, else use system time
				if (ros::ok())
				{
					current_sec = ros::Time::now().toSec();
				} else
				{
					current_sec = (double) time(NULL);
				}

				API->GetCartesianPosition(cur_position); //update current arm position

				//Check if X is in range
				if (((cur_position.Coordinates.X)
						<= Jaco_Position.Position.CartesianPosition.X + Postion_Range)
						&& (cur_position.Coordinates.X)
								>= (Jaco_Position.Position.CartesianPosition.X - Postion_Range))
				{
					Position_X_Reached = true;
				}

				//Check if Y is in range
				if (((cur_position.Coordinates.Y)
						<= Jaco_Position.Position.CartesianPosition.Y + Postion_Range)
						&& (cur_position.Coordinates.Y)
								>= (Jaco_Position.Position.CartesianPosition.Y - Postion_Range))
				{
					Position_Y_Reached = true;
				}

				//Check if Z is in range
				if (((cur_position.Coordinates.Z)
						<= Jaco_Position.Position.CartesianPosition.Z + Postion_Range)
						&& (cur_position.Coordinates.Z)
								>= (Jaco_Position.Position.CartesianPosition.Z - Postion_Range))
				{
					Position_Z_Reached = true;
				}

				//Check if ThetaX is in range
				if (((cur_position.Coordinates.ThetaX)
						<= Jaco_Position.Position.CartesianPosition.ThetaX + Rotation_Range)
						&& (cur_position.Coordinates.ThetaX)
								>= (Jaco_Position.Position.CartesianPosition.ThetaX - Rotation_Range))
				{
					Position_TX_Reached = true;
				}

				//Check if ThetaY is in range
				if (((cur_position.Coordinates.ThetaY)
						<= Jaco_Position.Position.CartesianPosition.ThetaY + Rotation_Range)
						&& (cur_position.Coordinates.ThetaY)
								>= (Jaco_Position.Position.CartesianPosition.ThetaY - Rotation_Range))
				{
					Position_TY_Reached = true;
				}

				//Check if ThetaZ is in range
				if (((cur_position.Coordinates.ThetaZ)
						<= Jaco_Position.Position.CartesianPosition.ThetaZ + Rotation_Range)
						&& (cur_position.Coordinates.ThetaZ)
								>= (Jaco_Position.Position.CartesianPosition.ThetaZ - Rotation_Range))
				{
					Position_TZ_Reached = true;
				}

				//If the arm reaches it's destination then break out of timeout loop
				if (Position_X_Reached == true && Position_Y_Reached == true && Position_Z_Reached == true
						&& Position_TX_Reached == true && Position_TY_Reached == true
						&& Position_TZ_Reached == true)
				{
					break;
				}
			}
		}
	}
}

void JacoArm::SetFingers(FingersPosition fingers, int timeout, bool push)
{
	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Position;

		memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

		ros::Duration(4.0).sleep();

		if (push == true)
		{
			API->EraseAllTrajectories();

			API->StopControlAPI();
		}

		API->StartControlAPI();

		Jaco_Position.Position.HandMode = POSITION_MODE;

		Jaco_Position.Position.Fingers.Finger1 = fingers.Finger1;
		Jaco_Position.Position.Fingers.Finger2 = fingers.Finger2;
		Jaco_Position.Position.Fingers.Finger3 = fingers.Finger3;

		API->SendAdvanceTrajectory(Jaco_Position);
		ROS_INFO("Sending Fingers");
//if we want to timeout
		if (timeout != 0)
		{
			double start_secs;
			double current_sec;

			//If ros is still runniing use rostime, else use system time
			if (ros::ok())
			{
				start_secs = ros::Time::now().toSec();
				current_sec = ros::Time::now().toSec();
			} else
			{
				start_secs = (double) time(NULL);
				current_sec = (double) time(NULL);
			}

			FingersPosition cur_fingers; //holds the current position of the fingers
			const float finger_range = 5; //dead zone for fingers
			bool Finger_1_Reached = false;
			bool Finger_2_Reached = false;
			bool Finger_3_Reached = false;

			//while we have not timed out
			while ((current_sec - start_secs) < timeout)
			{

				//If ros is still runniing use rostime, else use system time
				if (ros::ok())
				{
					current_sec = ros::Time::now().toSec();
				} else
				{
					current_sec = (double) time(NULL);
				}

				GetFingers(cur_fingers); //update current finger position

				//Check if finger is in range
				if (((cur_fingers.Finger1) <= Jaco_Position.Position.Fingers.Finger1 + finger_range)
						&& (cur_fingers.Finger1) >= (Jaco_Position.Position.Fingers.Finger1 - finger_range))
				{
					Finger_1_Reached = true;
				}

				//Check if finger is in range
				if (((cur_fingers.Finger2) <= Jaco_Position.Position.Fingers.Finger2 + finger_range)
						&& (cur_fingers.Finger2) >= (Jaco_Position.Position.Fingers.Finger2 - finger_range))
				{
					Finger_2_Reached = true;
				}

				//Check if finger is in range
				if (((cur_fingers.Finger3) <= Jaco_Position.Position.Fingers.Finger3 + finger_range)
						&& (cur_fingers.Finger3) >= (Jaco_Position.Position.Fingers.Finger3 - finger_range))
				{
					Finger_3_Reached = true;
				}

				//If all the fingers reached their destination then break out of timeout loop
				if (Finger_1_Reached == true && Finger_2_Reached == true && Finger_3_Reached == true)
				{
					break;
				}
			}
		}
	}
}

void JacoArm::SetCartesianVelocities(CartesianInfo velocities)
{
	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Velocity;

		memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure
		API->StartControlAPI();
		Jaco_Velocity.Position.Type = CARTESIAN_VELOCITY;

		Jaco_Velocity.Position.CartesianPosition.X = velocities.X;
		Jaco_Velocity.Position.CartesianPosition.Y = velocities.Y;
		Jaco_Velocity.Position.CartesianPosition.Z = velocities.Z;
		Jaco_Velocity.Position.CartesianPosition.ThetaX = velocities.ThetaX;
		Jaco_Velocity.Position.CartesianPosition.ThetaY = velocities.ThetaY;
		Jaco_Velocity.Position.CartesianPosition.ThetaZ = velocities.ThetaZ;

		API->SendAdvanceTrajectory(Jaco_Velocity);
	}
	else
	{
		API->EraseAllTrajectories();

	}
}

void JacoArm::SetVelocities(AngularInfo joint_vel)
{
	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Velocity;

		memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure
		API->StartControlAPI();
		Jaco_Velocity.Position.Type = ANGULAR_VELOCITY;

		Jaco_Velocity.Position.Actuators.Actuator1 = joint_vel.Actuator1;
		Jaco_Velocity.Position.Actuators.Actuator2 = joint_vel.Actuator2;
		Jaco_Velocity.Position.Actuators.Actuator3 = joint_vel.Actuator3;
		Jaco_Velocity.Position.Actuators.Actuator4 = joint_vel.Actuator4;
		Jaco_Velocity.Position.Actuators.Actuator5 = joint_vel.Actuator5;
		Jaco_Velocity.Position.Actuators.Actuator6 = joint_vel.Actuator6;

		API->SendAdvanceTrajectory(Jaco_Velocity);
	}
}

void JacoArm::GetAngles(AngularInfo &angles)
{
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

void JacoArm::GetConfig(ClientConfigurations &config)
{
	memset(&config, 0, sizeof(config)); //zero structure
	API->GetClientConfigurations(config);
}

void JacoArm::GetPosition(CartesianInfo &position)
{
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

void JacoArm::GetFingers(FingersPosition &fingers)
{
	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	API->GetCartesianPosition(Jaco_Position);

	fingers.Finger1 = Jaco_Position.Fingers.Finger1;
	fingers.Finger2 = Jaco_Position.Fingers.Finger2;
	fingers.Finger3 = Jaco_Position.Fingers.Finger3;

}

void JacoArm::PrintConfig(ClientConfigurations config)
{

	ROS_INFO("Jaco Config");
	ROS_INFO("ClientID = %s", config.ClientID);
	ROS_INFO("ClientName = %s", config.ClientName);
	ROS_INFO("Organization = %s", config.Organization);
	ROS_INFO("Serial = %s", config.Serial);
	ROS_INFO("Model = %s", config.Model);
	ROS_INFO("MaxLinearSpeed = %f", config.MaxLinearSpeed);
	ROS_INFO("MaxAngularSpeed = %f", config.MaxAngularSpeed);
	ROS_INFO("MaxLinearAcceleration = %f", config.MaxLinearAcceleration);
	ROS_INFO("MaxForce = %f", config.MaxForce);
	ROS_INFO("Sensibility = %f", config.Sensibility);
	ROS_INFO("DrinkingHeight = %f", config.DrinkingHeight);
	ROS_INFO("ComplexRetractActive = %d", config.ComplexRetractActive);
	ROS_INFO("RetractedPositionAngle = %f", config.RetractedPositionAngle);
	ROS_INFO("RetractedPositionCount = %d", config.RetractedPositionCount);
	ROS_INFO("DrinkingDistance = %f", config.DrinkingDistance);
	ROS_INFO("Fingers2and3Inverted = %d", config.Fingers2and3Inverted);
	ROS_INFO("DrinkingLenght = %f", config.DrinkingLenght);
	ROS_INFO("DeletePreProgrammedPositionsAtRetract = %d", config.DeletePreProgrammedPositionsAtRetract);
	ROS_INFO("EnableFlashErrorLog = %d", config.EnableFlashErrorLog);
	ROS_INFO("EnableFlashPositionLog = %d", config.EnableFlashPositionLog);

}
void JacoArm::PrintAngles(AngularInfo angles)
{

	ROS_INFO("Jaco Arm Angles (Degrees)");
	ROS_INFO("Joint 1 = %f", angles.Actuator1);
	ROS_INFO("Joint 2 = %f", angles.Actuator2);
	ROS_INFO("Joint 3 = %f", angles.Actuator3);

	ROS_INFO("Joint 4 = %f", angles.Actuator4);
	ROS_INFO("Joint 5 = %f", angles.Actuator5);
	ROS_INFO("Joint 6 = %f", angles.Actuator6);

}

void JacoArm::PrintPosition(CartesianInfo position)
{

	ROS_INFO("Jaco Arm Position (Meters)");
	ROS_INFO("X = %f", position.X);
	ROS_INFO("Y = %f", position.Y);
	ROS_INFO("Z = %f", position.Z);

	ROS_INFO("Jaco Arm Rotations (Radians)");
	ROS_INFO("Theta X = %f", position.ThetaX);
	ROS_INFO("Theta Y = %f", position.ThetaY);
	ROS_INFO("Theta Z = %f", position.ThetaZ);

}

void JacoArm::PrintFingers(FingersPosition fingers)
{

	ROS_INFO("Jaco Arm Finger Positions");
	ROS_INFO("Finger 1 = %f", fingers.Finger1);
	ROS_INFO("Finger 2 = %f", fingers.Finger2);
	ROS_INFO("Finger 3 = %f", fingers.Finger3);

}

void JacoArm::PoseMSG_Sub(const geometry_msgs::PoseStampedConstPtr& arm_pose)
{
	CartesianInfo Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	if (software_pause == false)
	{
		geometry_msgs::PoseStamped api_pose;
		ROS_INFO("Raw MSG");
		ROS_INFO("X = %f", arm_pose->pose.position.x);
		ROS_INFO("Y = %f", arm_pose->pose.position.y);
		ROS_INFO("Z = %f", arm_pose->pose.position.z);

		ROS_INFO("RX = %f", arm_pose->pose.orientation.x);
		ROS_INFO("RY = %f", arm_pose->pose.orientation.y);
		ROS_INFO("RZ = %f", arm_pose->pose.orientation.z);
		ROS_INFO("RW = %f", arm_pose->pose.orientation.w);

		while (ros::ok()
				&& !listener.canTransform("/jaco_api_origin", arm_pose->header.frame_id,
						arm_pose->header.stamp))
		{
			ros::spinOnce();
		}

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

		if (ros::Time::now() - last_update_time > update_time)
		{
			this->PrintPosition(Jaco_Position);

			last_update_time = ros::Time::now();
			this->SetPosition(Jaco_Position);
		}
	}

}

void JacoArm::SetFingerPositionMSG(const jaco_driver::finger_positionConstPtr& finger_pos)
{
	if (software_pause == false)
	{
		FingersPosition Finger_Position;
		memset(&Finger_Position, 0, sizeof(Finger_Position)); //zero structure

		Finger_Position.Finger1 = finger_pos->Finger_1;
		Finger_Position.Finger2 = finger_pos->Finger_2;
		Finger_Position.Finger3 = finger_pos->Finger_3;
		this->SetFingers(Finger_Position);
	}
}

void JacoArm::VelocityMSG(const jaco_driver::joint_velocityConstPtr& joint_vel)
{
	if (software_pause == false)
	{
		joint_velocities.Actuator1 = joint_vel->Velocity_J1;
		joint_velocities.Actuator2 = joint_vel->Velocity_J2;
		joint_velocities.Actuator3 = joint_vel->Velocity_J3;
		joint_velocities.Actuator4 = joint_vel->Velocity_J4;
		joint_velocities.Actuator5 = joint_vel->Velocity_J5;
		joint_velocities.Actuator6 = joint_vel->Velocity_J6;
		last_joint_update = ros::Time().now();

		if (joint_vel_timer_flag == false)
		{
			joint_vel_timer.start();
			joint_vel_timer_flag = true;
		}
	}

}

void JacoArm::ZeroArmMSG(const jaco_driver::zero_armConstPtr& zero_req)
{

	jaco_driver::zero_arm zero_stat;
	if (zero_req->zero == true)
	{
		this->ZeroArm();
		zero_stat.zero = true;
	}
	ZeroArm_pub.publish(zero_stat);

}
void JacoArm::SoftwarePauseMSG(const robot_base_msgs::SoftwareStopConstPtr& software_pause)
{

//TODO add software pause
	this->software_pause = software_pause->stop;
ROS_INFO("PAUSE");
	if (software_pause->stop == true)
	{
		ROS_INFO("STOP");

		API->EraseAllTrajectories();

	} else
	{		ROS_INFO("START");

		API->StartControlAPI();
	}

}

void JacoArm::CartesianVelocityMSG(const geometry_msgs::TwistStampedConstPtr& cartesian_vel)
{
	if (software_pause == false)
	{
		cartesian_velocities.X = cartesian_vel->twist.linear.x;
		cartesian_velocities.Y = cartesian_vel->twist.linear.y;
		cartesian_velocities.Z = cartesian_vel->twist.linear.z;
		cartesian_velocities.ThetaX = cartesian_vel->twist.angular.x;
		cartesian_velocities.ThetaY = cartesian_vel->twist.angular.y;
		cartesian_velocities.ThetaZ = cartesian_vel->twist.angular.z;

		last_cartesian_update = ros::Time().now();

		if (cartesian_vel_timer_flag == false)
		{
			cartesian_vel_timer.start();
			cartesian_vel_timer_flag = true;
		}
	}
}
void JacoArm::CartesianVelTimer(const ros::TimerEvent&)
{
	this->SetCartesianVelocities(cartesian_velocities);
	if ((ros::Time().now().toSec() - last_cartesian_update.toSec()) > 1)
	{
		cartesian_vel_timer.stop();
		cartesian_vel_timer_flag = false;
	}
}

void JacoArm::JointVelTimer(const ros::TimerEvent&)
{

	this->SetVelocities(joint_velocities);
	if ((ros::Time().now().toSec() - last_joint_update.toSec()) > 1)
	{
		joint_vel_timer.stop();
		joint_vel_timer_flag = false;
	}
}

void JacoArm::GoHome(void)
{

//	FingersPosition fingers_home = { 40, 40, 40 };
	AngularInfo joint_home;

	joint_home.Actuator1 = 176.0;
	joint_home.Actuator2 = 180.0;
	joint_home.Actuator3 = 107.0;
	joint_home.Actuator4 = 459.0;
	joint_home.Actuator5 = 102.0;
	joint_home.Actuator6 = 106.0;

//	this->SetFingers(fingers_home, 5); //send fingers to home position
	this->SetAngles(joint_home, 10); //send joints to home position

	joint_home.Actuator1 = 176.0;
	joint_home.Actuator2 = 111.0;
	joint_home.Actuator3 = 107.0;
	joint_home.Actuator4 = 459.0;
	joint_home.Actuator5 = 102.0;
	joint_home.Actuator6 = 106.0;

//	this->SetFingers(fingers_home, 5); //send fingers to home position
	this->SetAngles(joint_home, 10); //send joints to home position

	API->SetCartesianControl();
}

void JacoArm::BroadCastAngles(void)
{
	AngularPosition arm_angles;
	jaco_driver::joint_angles current_angles;

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

//Broadcast joint angles

	current_angles.Angle_J1 = arm_angles.Actuators.Actuator1;
	current_angles.Angle_J2 = arm_angles.Actuators.Actuator2;
	current_angles.Angle_J3 = arm_angles.Actuators.Actuator3;
	current_angles.Angle_J4 = arm_angles.Actuators.Actuator4;
	current_angles.Angle_J5 = arm_angles.Actuators.Actuator5;
	current_angles.Angle_J6 = arm_angles.Actuators.Actuator6;

	JointAngles_pub.publish(current_angles);
}

void JacoArm::BroadCastPosition(void)
{
	CartesianPosition position;
	geometry_msgs::PoseStamped current_position;
//	ROS_INFO("prior x = %f, y = %f, z = %f", position.Coordinates.X, position.Coordinates.Y,
//			position.Coordinates.Z);

	memset(&position, 0, sizeof(position)); //zero structure

	API->GetCartesianPosition(position); //Query arm for position
//
//	ROS_INFO("x = %f, y = %f, z = %f", position.Coordinates.X, position.Coordinates.Y,
//			position.Coordinates.Z);
//	ROS_INFO("finger 1 = %f, finger 2 = %f, finger 3 = %f,", position.Fingers.Finger1,
//			position.Fingers.Finger2, position.Fingers.Finger3);

	current_position.header.frame_id = "/jaco_api_origin";
	current_position.header.stamp = ros::Time().now();

//Broadcast position

	current_position.pose.position.x = position.Coordinates.X;
	current_position.pose.position.y = position.Coordinates.Y;
	current_position.pose.position.z = position.Coordinates.Z;

	tf::Quaternion position_quaternion;

	position_quaternion.setRPY(position.Coordinates.ThetaX, position.Coordinates.ThetaY,
			position.Coordinates.ThetaZ);

	tf::quaternionTFToMsg(position_quaternion, current_position.pose.orientation);

	/* The following code is for testing */
	/*remove this */
//
//	try
//	{
//
//		tf_listener.waitForTransform("/arm_mount", current_position.header.frame_id,
//				current_position.header.stamp, ros::Duration(1.0));
//
//		geometry_msgs::PoseStamped current_position_mount;
//
//		tf_listener.transformPose("/arm_mount", current_position, current_position_mount);
//		ROS_INFO("Position in %s",current_position_mount.header.frame_id.c_str());
//		ROS_INFO("x=%f",current_position_mount.pose.position.x);
//		ROS_INFO("y=%f",current_position_mount.pose.position.y);
//		ROS_INFO("z=%f",current_position_mount.pose.position.z);
//
//
//		tf::Quaternion q;
//		tf::quaternionMsgToTF(current_position_mount.pose.orientation, q);
//
//		tf::Matrix3x3 bt_q(q);
//
//		double yaw,pitch,roll;
//		bt_q.getRPY(roll,pitch,yaw);
//
//
//		ROS_INFO("roll=%f",roll);
//		ROS_INFO("pitch=%f",pitch);
//		ROS_INFO("yaw=%f",yaw);
//
//
//	} catch (std::exception& e)
//	{
//		ROS_ERROR_STREAM_THROTTLE(1, e.what());
//	}
	/* to here */

	ToolPosition_pub.publish(current_position);
}

void JacoArm::BroadCastFingerPosition(void)
{
	CartesianPosition Jaco_Position;
	jaco_driver::finger_position finger_position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	API->GetCartesianPosition(Jaco_Position);

	finger_position.Finger_1 = Jaco_Position.Fingers.Finger1;
	finger_position.Finger_2 = Jaco_Position.Fingers.Finger2;
	finger_position.Finger_3 = Jaco_Position.Fingers.Finger3;

	FingerPosition_pub.publish(finger_position);
}

void JacoArm::StatusTimer(const ros::TimerEvent&)
{
	this->BroadCastAngles();
	this->BroadCastPosition();
	this->BroadCastFingerPosition();
}

int main(int argc, char **argv)
{

	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_driver");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

//create the arm object
	JacoArm jaco(nh, param_nh);

	ros::spin();
//	jaco.GoHome();
}

