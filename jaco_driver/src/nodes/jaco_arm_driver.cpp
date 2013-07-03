//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================


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
	std::string joint_state("joint_state"); ///String containing the topic name for JointState
	std::string set_joint_angle_topic("set_joint_angle_topic"); ///String containing topic name for SetJoint

	//Grab the topic parameters, print warnings if using default values
	if (!param_nh.getParam(arm_pose_topic, arm_pose_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Arm Position Topic <%s>!", arm_pose_topic.c_str(),
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
	if (!param_nh.getParam(joint_state, joint_state))
		ROS_WARN("Parameter <%s> Not Set. Using Default Joint State Topic <%s>!", joint_state.c_str(),
				joint_state.c_str());
	if (!param_nh.getParam(set_joint_angle_topic, set_joint_angle_topic))
		ROS_WARN("Parameter <%s> Not Set. Using Default Set Joint Angle Topic <%s>!", set_joint_angle_topic.c_str(),
				set_joint_angle_topic.c_str());

	//Print out received topics
	ROS_DEBUG("Got Arm Position Topic Name: <%s>", arm_pose_topic.c_str());
	ROS_DEBUG("Got Joint Velocity Topic Name: <%s>", joint_velocity_topic.c_str());
	ROS_DEBUG("Got Joint Angles Topic Name: <%s>", joint_angles_topic.c_str());
	ROS_DEBUG("Got Cartesian Velocity Topic Name: <%s>", cartesian_velocity_topic.c_str());
	ROS_DEBUG("Got Tool Position Topic Name: <%s>", tool_position_topic.c_str());
	ROS_DEBUG("Got Set Finger Position Topic Name: <%s>", set_finger_position_topic.c_str());
	ROS_DEBUG("Got Finger Position Topic Name: <%s>", finger_position_topic.c_str());
	ROS_DEBUG("Got Joint State Topic Name: <%s>", joint_state.c_str());
	ROS_DEBUG("Got Set Joint Angle Topic Name: <%s>", set_joint_angle_topic.c_str());

	ROS_INFO("Starting Up Jaco Arm Controller...");

	this->software_pause = false;
	previous_state = 0;

	/* Connecting to Jaco Arm */
	ROS_INFO("Initiating Library");
	API = new JacoAPI();
	ROS_INFO("Initiating API");
	last_update_time = ros::Time::now();
	update_time = ros::Duration(5.0);

	int api_result = 0; //stores result from the API
	ros::Duration(5.0).sleep();

	api_result = (API->InitAPI());

	/* 
	A common result that may be returned is "1014", which means communications
	could not be established with the arm.  This often means the arm is not turned on, 
	or the InitAPI command was initiated before the arm had fully booted up.
	*/

	if (api_result != 1)
	{
		/* Failed to contact arm */
		ROS_FATAL("Could not initialize arm");
		ROS_FATAL("Jaco_InitAPI returned: %d", api_result);
		#ifndef DEBUG_WITHOUT_ARM
		ros::shutdown();
		#endif
	} 
	else
	{
		ROS_INFO("API Initialized Successfully!");
	}
	ros::Duration(2.0).sleep();

	/* Set up Services */
	stop_service = nh.advertiseService("stop", &JacoArm::StopSRV, this);
	start_service = nh.advertiseService("start", &JacoArm::StartSRV, this);
	homing_service = nh.advertiseService("home_arm", &JacoArm::HomeArmSRV, this);

	/* Set Default Configuration */

	// API->RestoreFactoryDefault(); // uncomment comment ONLY if you want to lose your settings on each launch.

	ClientConfigurations configuration;
	GetConfig(configuration);
	PrintConfig(configuration);

	ROS_INFO("Initializing the Arm");

	ZeroArm();

	/* Storing arm in home position */

	// this->GoHome();  // Useful if you want to move the arm to an alternate "home" position after the default home.

	/* Set up Publishers */
	this->JointAngles_pub = nh.advertise<jaco_driver::JointAngles>(joint_angles_topic, 2);
	this->JointState_pub = nh.advertise<sensor_msgs::JointState>(joint_state, 2);
	this->ToolPosition_pub = nh.advertise<geometry_msgs::PoseStamped>(tool_position_topic, 2);
	this->FingerPosition_pub = nh.advertise<jaco_driver::FingerPosition>(finger_position_topic, 2);

	/* Set up Subscribers*/
	this->ArmPose_sub = nh.subscribe(arm_pose_topic, 1, &JacoArm::PoseMSG_Sub, this);
	this->JointVelocity_sub = nh.subscribe(joint_velocity_topic, 1, &JacoArm::VelocityMSG, this);
	this->CartesianVelocity_sub = nh.subscribe(cartesian_velocity_topic, 1, &JacoArm::CartesianVelocityMSG, this);
	this->SetFingerPosition_sub = nh.subscribe(set_finger_position_topic, 1, &JacoArm::SetFingerPositionMSG, this);
	this->SetJoint_sub = nh.subscribe(set_joint_angle_topic, 1, &JacoArm::SetJointAnglesMSG, this);

	this->status_timer = nh.createTimer(ros::Duration(0.05), &JacoArm::StatusTimer, this);

	this->joint_vel_timer = nh.createTimer(ros::Duration(0.01), &JacoArm::JointVelTimer, this);
	joint_vel_timer.stop();
	joint_vel_timer_flag = false;
	this->cartesian_vel_timer = nh.createTimer(ros::Duration(0.01), &JacoArm::CartesianVelTimer, this);
	cartesian_vel_timer.stop();
	cartesian_vel_timer_flag = false;

	BroadCastAngles();
	ROS_INFO("The Arm is ready to use.");

	TrajectoryPoint Jaco_Velocity;

	memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure

	API->StartControlAPI();
	Jaco_Velocity.Position.Type = CARTESIAN_VELOCITY;

	API->SendAdvanceTrajectory(Jaco_Velocity);
}

void JacoArm::WaitForHome(int timeout)
{
	double start_secs;
	double current_sec;

	//If ros is still running use rostime, else use system time
	if (ros::ok())
	{
		start_secs = ros::Time::now().toSec();
		current_sec = ros::Time::now().toSec();
	} 
	else
	{
		start_secs = (double) time(NULL);
		current_sec = (double) time(NULL);
	}

	//while we have not timed out
	while ((current_sec - start_secs) < timeout)
	{
		ros::Duration(0.5).sleep();
		
		//If ros is still running use rostime, else use system time
		if (ros::ok())
		{
			current_sec = ros::Time::now().toSec();
		} 
		else
		{
			current_sec = (double) time(NULL);
		}

		/* Check each joint angle to determine whether it has stopped moving */
		if (HomeState())
		{
			ros::Duration(1.0).sleep();  // Grants a bit more time for the arm to "settle"
			return;
		}
	}

	ROS_WARN("Timed out waiting for arm to return \"home\"");
}

bool JacoArm::HomeState(void)
{
/*
Determines whether the arm has returned to its "Home" state by checking the current
joint angles, then comparing them to the known "Home" joint angles.
*/
	const AngularInfo home_position = {
		282.8,
		154.4,
		43.6,
		230.7,
		83.0,
		78.1 };
	const float tolerance = 1.0; //dead zone for angles (degrees)

	AngularPosition cur_angles; //holds the current angles of the arm
	memset(&cur_angles, 0, sizeof(cur_angles));
	API->GetAngularPosition(cur_angles); //update current arm angles

	return CompareAngles(home_position, cur_angles.Actuators, tolerance);
}

void JacoArm::SetConfig(ClientConfigurations config)
{

/*
Obtains the current arm configuration, which are stored on the arm itself. Many
of these configurations may be set using the Windows interface.
*/

	API->SetClientConfigurations(config);
}

void JacoArm::ZeroArm(void)
{

/*
Sends the arm to the "home" position.  The code replicates the function of the "home" button on the
user controller by "pressing" the home button long enough for the arm to return to the home position.

Future API changes may expose a flag that indicates when the arm has arrived in the home position,
thus allowing the homing procedure to take less time than the "hard-wired" 20 seconds.

Fingers are homed by manually opening them fully, then returning them to a half-open position.

Note that if the arm is already in the "home" position, this routine will cause it to move into the
"retract" position.
*/

	if (HomeState())
	{
		ROS_INFO("Arm is already in \"home\" position");
		return;
	}

	API->StartControlAPI();

	JoystickCommand home_command;
	memset(&home_command, 0, sizeof(home_command)); //zero structure

	home_command.ButtonValue[2] = 1;
	API->SendJoystickCommand(home_command);
	WaitForHome(25);
	home_command.ButtonValue[2] = 0;
	API->SendJoystickCommand(home_command);

	FingersPosition fingers_home;

	// Set the fingers fully "open." This is required to initialize the fingers.
	fingers_home.Finger1 = 0;
	fingers_home.Finger2 = 0;
	fingers_home.Finger3 = 0;
	this->SetFingers(fingers_home, 5);
	ros::Duration(1.0).sleep();

	// Set the fingers to "half-open"
	fingers_home.Finger1 = 40;
	fingers_home.Finger2 = 40;
	fingers_home.Finger3 = 40;
	this->SetFingers(fingers_home, 5);
	//ros::Duration(2.0).sleep();
}


bool JacoArm::HomeArmSRV(jaco_driver::HomeArm::Request &req, jaco_driver::HomeArm::Response &res)
{
	ZeroArm();
	res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";

	return true;
}


void JacoArm::SetAngles(AngularInfo angles, int timeout, bool push)
{

/*
Sends a joint angle command to the Jaco arm, and waits until the arm has stopped
moving before releasing control of the API.
*/

	timeout = 30;

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
		API->SetAngularControl();
		
		Jaco_Position.LimitationsActive = false;
		Jaco_Position.Position.Delay = 0.0;
		Jaco_Position.Position.Type = ANGULAR_POSITION;

		Jaco_Position.Position.Actuators = angles;

		API->SendAdvanceTrajectory(Jaco_Position);

		if (timeout != 0)
		{
			double start_secs;
			double current_sec;

			//If ros is still running use rostime, else use system time
			if (ros::ok())
			{
				start_secs = ros::Time::now().toSec();
				current_sec = ros::Time::now().toSec();
			} 
			else
			{
				start_secs = (double) time(NULL);
				current_sec = (double) time(NULL);
			}

			AngularPosition cur_angles; //holds the current angles of the arm
			AngularPosition past_angles;
			memset(&cur_angles, 0, sizeof(cur_angles));

			API->GetAngularPosition(past_angles); // Load the starting position into past_angles

			const float tolerance = 0.01; //dead zone for angles (degrees)

			//ros::Duration(1.0).sleep();

			//while we have not timed out
			while ((current_sec - start_secs) < timeout)
			{
				ros::Duration(0.5).sleep();
				
				//If ros is still running use rostime, else use system time
				if (ros::ok())
				{
					current_sec = ros::Time::now().toSec();
				} 
				else
				{
					current_sec = (double) time(NULL);
				}

				API->GetAngularPosition(cur_angles); //update current arm angles

				/* Check each joint angle to determine whether it has stopped moving */
				if (CompareAngles(past_angles.Actuators, cur_angles.Actuators, tolerance))
				{
					ROS_DEBUG("Angular Control Complete.");
					ros::Duration(1.0).sleep();  // Grants a bit more time for the arm to "settle"
					API->EraseAllTrajectories();  // Clear any remaining trajectories
					//API->StopControlAPI();
					break;
				}
				else
				{
					past_angles.Actuators = cur_angles.Actuators;
				}
			}
		}
	}
}

void JacoArm::SetPosition(CartesianInfo position, int timeout, bool push)
{

/*
Sends a cartesian coordinate trajectory to the Jaco arm, and waits until the arm has stopped
moving before releasing control of the API.
*/

	timeout = 30;

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
		API->SetCartesianControl();

		Jaco_Position.LimitationsActive = false;
		Jaco_Position.Position.Delay = 0.0;
		Jaco_Position.Position.Type = CARTESIAN_POSITION;

		Jaco_Position.Position.CartesianPosition = position;
		Jaco_Position.Position.CartesianPosition.ThetaZ += 0.0001; // A workaround for a bug in the Kinova API

		API->SendBasicTrajectory(Jaco_Position);

		if (timeout != 0)
		{
			double start_secs;
			double current_sec;

			//If ros is still running use rostime, else use system time
			if (ros::ok())
			{
				start_secs = ros::Time::now().toSec();
				current_sec = ros::Time::now().toSec();
			} 
			else
			{
				start_secs = (double) time(NULL);
				current_sec = (double) time(NULL);
			}

			CartesianPosition cur_position;		//holds the current position of the arm
			CartesianPosition past_position;	//holds the past position of the arm
			memset(&cur_position, 0, sizeof(cur_position));

			API->GetCartesianPosition(past_position); //pre-load the past arm position

			const float tolerance = 0.001; 	//dead zone for position

			//ros::Duration(1.0).sleep();

			//while we have not timed out
			while ((current_sec - start_secs) < timeout)
			{

				ros::Duration(0.5).sleep();

				//If ros is still runniing use rostime, else use system time
				if (ros::ok())
				{
					current_sec = ros::Time::now().toSec();
				} 
				else
				{
					current_sec = (double) time(NULL);
				}

				API->GetCartesianPosition(cur_position); //update current arm position

				if (ComparePositions(past_position.Coordinates, cur_position.Coordinates, tolerance))
				{
					ROS_INFO("Cartesian Control Complete.");
					ros::Duration(1.0).sleep();  // Grants a bit more time for the arm to "settle"
					API->EraseAllTrajectories();  // Clear any remaining trajectories				
					//API->StopControlAPI();
					break;
				}
				else
				{
					past_position.Coordinates = cur_position.Coordinates;
				}
			}
		}
	}	
}

void JacoArm::SetFingers(FingersPosition fingers, int timeout, bool push)
{

/*
Sets the finger positions
*/

	timeout = 30;

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

		Jaco_Position.Position.Fingers = fingers;

		API->SendAdvanceTrajectory(Jaco_Position);
		ROS_DEBUG("Sending Fingers");

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
			} 
			else
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
				} 
				else
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

/*
Set the velocity of the angles using cartesian input.
*/

	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Velocity;

		memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure

		API->StartControlAPI();
		Jaco_Velocity.Position.Type = CARTESIAN_VELOCITY;

		// confusingly, velocity is passed in the position struct
		Jaco_Velocity.Position.CartesianPosition = velocities;

		API->SendAdvanceTrajectory(Jaco_Velocity);
	}
	else
	{
		API->EraseAllTrajectories();
	}
}

void JacoArm::SetVelocities(AngularInfo joint_vel)
{

/*
Set the velocity of the angles using angular input.
*/

	if (software_pause == false)
	{
		TrajectoryPoint Jaco_Velocity;

		memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure

		API->StartControlAPI();
		Jaco_Velocity.Position.Type = ANGULAR_VELOCITY;

		// confusingly, velocity is passed in the position struct
		Jaco_Velocity.Position.Actuators = joint_vel;

		API->SendAdvanceTrajectory(Jaco_Velocity);
	}
}

void JacoArm::GetAngles(AngularInfo &angles)
{

/*
API call to obtain the current angular position of all the joints.
*/

	AngularPosition Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	API->GetAngularPosition(Jaco_Position);

	angles = Jaco_Position.Actuators;
}

void JacoArm::GetConfig(ClientConfigurations &config)
{

/*
API call to obtain the current client configuration.
*/

	memset(&config, 0, sizeof(config)); //zero structure
	API->GetClientConfigurations(config);
}

void JacoArm::GetPosition(CartesianInfo &position)
{

/*
API call to obtain the current cartesian position of the arm.
*/

	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	API->GetCartesianPosition(Jaco_Position);

	position = Jaco_Position.Coordinates;
}

void JacoArm::GetFingers(FingersPosition &fingers)
{

/*
API call to obtain the current finger positions.
*/

	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	API->GetCartesianPosition(Jaco_Position);

	fingers = Jaco_Position.Fingers;
}

void JacoArm::PrintConfig(ClientConfigurations config)
{

/*
Dumps the client configuration onto the screen.  
*/

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
	ROS_INFO("DrinkingLength = %f", config.DrinkingLenght);
	ROS_INFO("DeletePreProgrammedPositionsAtRetract = %d", config.DeletePreProgrammedPositionsAtRetract);
	ROS_INFO("EnableFlashErrorLog = %d", config.EnableFlashErrorLog);
	ROS_INFO("EnableFlashPositionLog = %d", config.EnableFlashPositionLog);

}
void JacoArm::PrintAngles(AngularInfo angles)
{

/*
Dumps the current joint angles onto the screen.  
*/

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

/*
Dumps the current cartesian positions onto the screen.  
*/

	ROS_DEBUG("Jaco Arm Position (Meters)");
	ROS_DEBUG("X = %f", position.X);
	ROS_DEBUG("Y = %f", position.Y);
	ROS_DEBUG("Z = %f", position.Z);

	ROS_DEBUG("Jaco Arm Rotations (Radians)");
	ROS_DEBUG("Theta X = %f", position.ThetaX);
	ROS_DEBUG("Theta Y = %f", position.ThetaY);
	ROS_DEBUG("Theta Z = %f", position.ThetaZ);

}

void JacoArm::PrintFingers(FingersPosition fingers)
{

/*
Dumps the current finger positions onto the screen.  
*/

	ROS_DEBUG("Jaco Arm Finger Positions");
	ROS_DEBUG("Finger 1 = %f", fingers.Finger1);
	ROS_DEBUG("Finger 2 = %f", fingers.Finger2);
	ROS_DEBUG("Finger 3 = %f", fingers.Finger3);

}

void JacoArm::PoseMSG_Sub(const geometry_msgs::PoseStampedConstPtr& arm_pose)
{

/*
Displays the cartesian coordinates of the arm before and after a transform.
*/

	CartesianInfo Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	if (software_pause == false)
	{
		geometry_msgs::PoseStamped api_pose;
		ROS_DEBUG("Raw MSG");
		ROS_DEBUG("X = %f", arm_pose->pose.position.x);
		ROS_DEBUG("Y = %f", arm_pose->pose.position.y);
		ROS_DEBUG("Z = %f", arm_pose->pose.position.z);

		ROS_DEBUG("RX = %f", arm_pose->pose.orientation.x);
		ROS_DEBUG("RY = %f", arm_pose->pose.orientation.y);
		ROS_DEBUG("RZ = %f", arm_pose->pose.orientation.z);
		ROS_DEBUG("RW = %f", arm_pose->pose.orientation.w);

		while (ros::ok()
				&& !listener.canTransform("/jaco_api_origin", arm_pose->header.frame_id,
						arm_pose->header.stamp))
		{
			ros::spinOnce();
		}

		listener.transformPose("/jaco_api_origin", *arm_pose, api_pose);

		ROS_DEBUG("Transformed MSG");
		ROS_DEBUG("X = %f", api_pose.pose.position.x);
		ROS_DEBUG("Y = %f", api_pose.pose.position.y);
		ROS_DEBUG("Z = %f", api_pose.pose.position.z);

		ROS_DEBUG("RX = %f", api_pose.pose.orientation.x);
		ROS_DEBUG("RY = %f", api_pose.pose.orientation.y);
		ROS_DEBUG("RZ = %f", api_pose.pose.orientation.z);
		ROS_DEBUG("RW = %f", api_pose.pose.orientation.w);

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


void JacoArm::SetFingerPositionMSG(const jaco_driver::FingerPositionConstPtr& finger_pos)
{

/*
Receives ROS command messages and relays them to SetFingers.
*/

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


void JacoArm::SetJointAnglesMSG(const jaco_driver::JointAnglesConstPtr& angles)
{

/*
Receives ROS command messages and relays them to SetAngles.
*/

	if (software_pause == false)
	{
		AngularInfo Joint_Position;
		memset(&Joint_Position, 0, sizeof(Joint_Position)); //zero structure

		Joint_Position.Actuator1 = angles->Angle_J1;
		Joint_Position.Actuator2 = angles->Angle_J2;
		Joint_Position.Actuator3 = angles->Angle_J3;
		Joint_Position.Actuator4 = angles->Angle_J4;
		Joint_Position.Actuator5 = angles->Angle_J5;
		Joint_Position.Actuator6 = angles->Angle_J6;

		this->SetAngles(Joint_Position);
	}
}


void JacoArm::VelocityMSG(const jaco_driver::JointVelocityConstPtr& joint_vel)
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

bool JacoArm::StopSRV(jaco_driver::Stop::Request &req, jaco_driver::Stop::Response &res)
{
/*
A service that will instantly stop the arm.
*/

	software_pause = true;

	API->StartControlAPI();

	JoystickCommand home_command;
	memset(&home_command, 0, sizeof(home_command)); //zero structure

	home_command.ButtonValue[2] = 1;
	API->SendJoystickCommand(home_command);
	ros::Duration(0.05).sleep();
	home_command.ButtonValue[2] = 0;
	API->SendJoystickCommand(home_command);

	API->EraseAllTrajectories();

	res.stop_result = "JACO ARM HAS BEEN STOPPED";
	ROS_DEBUG("JACO ARM STOP REQUEST");

	return true;
}

bool JacoArm::StartSRV(jaco_driver::Start::Request &req, jaco_driver::Start::Response &res)
{
/*
A service that re-enables control of the arm.
*/

	software_pause = false;

	API->StartControlAPI();

	res.start_result = "JACO ARM CONTROL HAS BEEN ENABLED";
	ROS_DEBUG("JACO ARM START REQUEST");

	return true;
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

/*
Contains coordinates for an alternate "Home" position, to which the arm may move after
reaching the default home position. GoHome() function must be enabled in the initialization
routine for this to work.
*/

	AngularInfo joint_home;

	joint_home.Actuator1 = 176.0;
	joint_home.Actuator2 = 111.0;
	joint_home.Actuator3 = 107.0;
	joint_home.Actuator4 = 459.0;
	joint_home.Actuator5 = 102.0;
	joint_home.Actuator6 = 106.0;

	this->SetAngles(joint_home, 10); //send joints to home position

	API->SetCartesianControl();
}

void JacoArm::BroadCastAngles(void)
{

/*
Publishes the current joint angles.

Joint angles are published in both their raw state as obtained from the arm (JointAngles),
and transformed & converted to radians (joint_state) as per the Jaco Kinematics PDF.

JointState will eventually also publish the velocity and effort for each joint, when
this data is made available by the C++ API.  Currenty velocity and effort are reported
as being zero (0.0) for all joints.
*/

	// Populate an array of joint names.  arm_0_joint is the base, arm_5_joint is the wrist.
	const char* nameArgs[] = {"arm_0_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "Finger_1", "Finger_2", "Finger_3"};
	std::vector<std::string> JointName(nameArgs, nameArgs+9);

	AngularPosition arm_angles;

	jaco_driver::JointAngles current_angles;

	sensor_msgs::JointState joint_state;
	joint_state.name = JointName;

	// Define array sizes for the joint_state topic
        joint_state.position.resize(9);
	joint_state.velocity.resize(9);
	joint_state.effort.resize(9);

	memset(&arm_angles, 0, sizeof(arm_angles)); //zero structure

	//Query arm for current joint angles
	API->GetAngularPosition(arm_angles); 

	// Raw joint angles
	current_angles.Angle_J1 = arm_angles.Actuators.Actuator1;
	current_angles.Angle_J2 = arm_angles.Actuators.Actuator2;
	current_angles.Angle_J3 = arm_angles.Actuators.Actuator3;
	current_angles.Angle_J4 = arm_angles.Actuators.Actuator4;
	current_angles.Angle_J5 = arm_angles.Actuators.Actuator5;
	current_angles.Angle_J6 = arm_angles.Actuators.Actuator6;

	// Transform from Kinova DH algorithm to physical angles in radians, then place into vector array

	joint_state.position[0] = (180.0 - arm_angles.Actuators.Actuator1) / (180.0 / PI);
	joint_state.position[1] = (arm_angles.Actuators.Actuator2 - 270.0) / (180.0 / PI);
	joint_state.position[2] = (90.0 - arm_angles.Actuators.Actuator3) / (180.0 / PI);
	joint_state.position[3] = (180.0 - arm_angles.Actuators.Actuator4) / (180.0 / PI);
	joint_state.position[4] = (180.0 - arm_angles.Actuators.Actuator5) / (180.0 / PI);
	joint_state.position[5] = (260.0 - arm_angles.Actuators.Actuator6) / (180.0 / PI);
	joint_state.position[6] = (180.0 - arm_angles.Fingers.Finger1) / (180.0 / PI);
	joint_state.position[7] = (180.0 - arm_angles.Fingers.Finger2) / (180.0 / PI);
	joint_state.position[8] = (180.0 - arm_angles.Fingers.Finger3) / (180.0 / PI);

	//Publish the joint state messages

	JointAngles_pub.publish(current_angles); // Publishes the raw joint angles in a custom message.

	JointState_pub.publish(joint_state);     // Publishes the transformed angles in a standard sensor_msgs format.

}

void JacoArm::BroadCastPosition(void)
{

/*
Publishes the current cartesian coordinates
*/

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

	ToolPosition_pub.publish(current_position);
}

void JacoArm::BroadCastFingerPosition(void)
{

/*
Publishes the current finger positions.
*/

	CartesianPosition Jaco_Position;
	jaco_driver::FingerPosition finger_position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure
	API->GetCartesianPosition(Jaco_Position);

	finger_position.Finger_1 = Jaco_Position.Fingers.Finger1;
	finger_position.Finger_2 = Jaco_Position.Fingers.Finger2;
	finger_position.Finger_3 = Jaco_Position.Fingers.Finger3;

	FingerPosition_pub.publish(finger_position);
}

bool JacoArm::ComparePositions(const CartesianInfo &first, const CartesianInfo &second, float tolerance)
{
	bool status = true;

	status = status && CompareValues(first.X, second.X, tolerance);
	status = status && CompareValues(first.Y, second.Y, tolerance);
	status = status && CompareValues(first.Z, second.Z, tolerance);
	status = status && CompareValues(first.ThetaX, second.ThetaX, tolerance);
	status = status && CompareValues(first.ThetaY, second.ThetaY, tolerance);
	status = status && CompareValues(first.ThetaZ, second.ThetaZ, tolerance);

	return status;
}

bool JacoArm::CompareAngles(const AngularInfo &first, const AngularInfo &second, float tolerance)
{
	bool status = true;

	status = status && CompareAngularValues(first.Actuator1, second.Actuator1, tolerance);
	status = status && CompareAngularValues(first.Actuator2, second.Actuator2, tolerance);
	status = status && CompareAngularValues(first.Actuator3, second.Actuator3, tolerance);
	status = status && CompareAngularValues(first.Actuator4, second.Actuator4, tolerance);
	status = status && CompareAngularValues(first.Actuator5, second.Actuator5, tolerance);
	status = status && CompareAngularValues(first.Actuator6, second.Actuator6, tolerance);

	return status;
}

bool JacoArm::CompareValues(float first, float second, float tolerance)
{
	return ((first <= second + tolerance) && (first >= second - tolerance));
}

bool JacoArm::CompareAngularValues(float first, float second, float tolerance)
{
	// joint angles may be off in increments of 360 degrees, normalize them
	while (first > 360.0)
		first -= 360.0;
	while (first < 0.0)
		first += 360.0;
	while (second > 360.0)
		second -= 360.0;
	while (second < 0.0)
		second += 360.0;

	return ((first <= second + tolerance) && (first >= second - tolerance));
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
}

