/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: jaco_comm.cpp
 *  Desc: Class for moving/querying jaco arm.
 *  Auth: Alex Bencz, Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#include <ros/ros.h>
#include "jaco_driver/jaco_comm.h"

#define PI 3.14159265358

namespace jaco
{

JacoComm::JacoComm() : software_stop(false)
{
	/* Connecting to Jaco Arm */
	ROS_INFO("Initiating Library");
	API = new JacoAPI();
	ROS_INFO("Initiating API");

	int api_result = 0; //stores result from the API
	ros::Duration(1.0).sleep();

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
}

JacoComm::~JacoComm()
{
	API->CloseAPI();
}

/*!
 * \brief Determines whether the arm has returned to its "Home" state. 
 * 
 * Checks the current joint angles, then compares them to the known "Home"
 * joint angles.
 */
bool JacoComm::HomeState(void)
{
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

/*!
 * \brief Send the arm to the "home" position.
 * 
 * The code replicates the function of the "home" button on the user controller
 * by "pressing" the home button long enough for the arm to return to the home
 * position.
 *
 * Fingers are homed by manually opening them fully, then returning them to a
 * half-open position.
 */
void JacoComm::HomeArm(void)
{
	if (Stopped())
	{
		return;
	}

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
}

/*!
 * \brief Initialize finger actuators.
 *
 * Move fingers to open position, then close part way to initialize limits.
 */
void JacoComm::InitializeFingers(void)
{
	FingersPosition fingers_home = {0, 0, 0};

	// Set the fingers fully "open." This is required to initialize the fingers.
	SetFingers(fingers_home, 5);
	ros::Duration(1.0).sleep();

	// Set the fingers to "half-open"
	fingers_home.Finger1 = 40;
	fingers_home.Finger2 = 40;
	fingers_home.Finger3 = 40;
	SetFingers(fingers_home, 5);
}

/*!
 * \brief Sends a joint angle command to the Jaco arm.
 * 
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void JacoComm::SetAngles(AngularInfo angles, int timeout, bool push)
{
	if (Stopped())
		return;

	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	if (push == true)
	{
		API->EraseAllTrajectories();
		API->StopControlAPI();
	}
	
	API->StartControlAPI();
	API->SetAngularControl();
	
	//Jaco_Position.LimitationsActive = false;
	Jaco_Position.Position.Delay = 0.0;
	Jaco_Position.Position.Type = ANGULAR_POSITION;

	// No transform performed, raw angles expected:
	//Jaco_Position.Position.Actuators = angles; 

	// This block converts the angles in DH-transformed radians back into degrees for the arm:
	Jaco_Position.Position.Actuators.Actuator1 = 180.0 - (angles.Actuator1 * (180.0 / PI));
	Jaco_Position.Position.Actuators.Actuator2 = (angles.Actuator2 * (180.0 / PI)) + 270.0;
	Jaco_Position.Position.Actuators.Actuator3 = 90.0 - (angles.Actuator3 * (180.0 / PI));
	Jaco_Position.Position.Actuators.Actuator4 = 180.0 - (angles.Actuator4 * (180.0 / PI));
	Jaco_Position.Position.Actuators.Actuator5 = 180.0 - (angles.Actuator5 * (180.0 / PI));
	Jaco_Position.Position.Actuators.Actuator6 = 260.0 - (angles.Actuator6 * (180.0 / PI));

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

		const float tolerance = 0.5; //dead zone for angles (degrees)

		//while we have not timed out
		while ((current_sec - start_secs) < timeout)
		{
			ros::Duration(0.5).sleep();
			
			//If ros is still running use rostime, else use system time
			if (ros::ok())
			{
				current_sec = ros::Time::now().toSec();
				ros::spinOnce();
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
				break;
			}
			else
			{
				past_angles.Actuators = cur_angles.Actuators;
			}
		}
	}
}

/*!
 * \brief Sends a cartesian coordinate trajectory to the Jaco arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void JacoComm::SetPosition(CartesianInfo position, int timeout, bool push)
{
	if (Stopped())
		return;

	TrajectoryPoint Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	if (push == true)
	{
		API->EraseAllTrajectories();
		API->StopControlAPI();
	}

	API->StartControlAPI();
	API->SetCartesianControl();

	//Jaco_Position.LimitationsActive = false;
	Jaco_Position.Position.Delay = 0.0;
	Jaco_Position.Position.Type = CARTESIAN_POSITION;

	Jaco_Position.Position.CartesianPosition = position;
	//Jaco_Position.Position.CartesianPosition.ThetaZ += 0.0001; // A workaround for a bug in the Kinova API

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

		//while we have not timed out
		while ((current_sec - start_secs) < timeout)
		{

			ros::Duration(0.5).sleep();

			//If ros is still runniing use rostime, else use system time
			if (ros::ok())
			{
				current_sec = ros::Time::now().toSec();
				ros::spinOnce();
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
				break;
			}
			else
			{
				past_position.Coordinates = cur_position.Coordinates;
			}
		}
	}
}

/*!
 * \brief Sets the finger positions
 */
void JacoComm::SetFingers(FingersPosition fingers, int timeout, bool push)
{
	if (Stopped())
		return;

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

/*!
 * \brief Set the velocity of the angles using angular input.
 */
void JacoComm::SetVelocities(AngularInfo joint_vel)
{
	if (Stopped())
		return;

	TrajectoryPoint Jaco_Velocity;

	memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure

	API->StartControlAPI();
	Jaco_Velocity.Position.Type = ANGULAR_VELOCITY;

	// confusingly, velocity is passed in the position struct
	Jaco_Velocity.Position.Actuators = joint_vel;

	API->SendAdvanceTrajectory(Jaco_Velocity);
}

/*!
 * \brief Set the velocity of the angles using cartesian input.
 */
void JacoComm::SetCartesianVelocities(CartesianInfo velocities)
{
	if (Stopped())
	{
		API->EraseAllTrajectories();
		return;
	}

	TrajectoryPoint Jaco_Velocity;

	memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure

	API->StartControlAPI();
	Jaco_Velocity.Position.Type = CARTESIAN_VELOCITY;

	// confusingly, velocity is passed in the position struct
	Jaco_Velocity.Position.CartesianPosition = velocities;

	API->SendAdvanceTrajectory(Jaco_Velocity);
}

/*!
 * \brief Obtains the current arm configuration.
 *
 * This is the configuration which are stored on the arm itself. Many of these
 * configurations may be set using the Windows interface.
 */
void JacoComm::SetConfig(ClientConfigurations config)
{
	API->SetClientConfigurations(config);
}

/*!
 * \brief API call to obtain the current angular position of all the joints.
 */
void JacoComm::GetAngles(AngularInfo &angles)
{
	AngularPosition Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	API->GetAngularPosition(Jaco_Position);

	angles = Jaco_Position.Actuators;
}

/*!
 * \brief API call to obtain the current cartesian position of the arm.
 */
void JacoComm::GetPosition(CartesianInfo &position)
{
	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	API->GetCartesianPosition(Jaco_Position);

	position = Jaco_Position.Coordinates;
}

/*!
 * \brief API call to obtain the current finger positions.
 */
void JacoComm::GetFingers(FingersPosition &fingers)
{
	CartesianPosition Jaco_Position;

	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	API->GetCartesianPosition(Jaco_Position);

	fingers = Jaco_Position.Fingers;
}

/*!
 * \brief API call to obtain the current client configuration.
 */
void JacoComm::GetConfig(ClientConfigurations &config)
{
	memset(&config, 0, sizeof(config)); //zero structure
	API->GetClientConfigurations(config);
}

/*!
 * \brief Dumps the current joint angles onto the screen.  
 */
void JacoComm::PrintAngles(AngularInfo angles)
{
	ROS_INFO("Jaco Arm Angles (Degrees)");
	ROS_INFO("Joint 1 = %f", angles.Actuator1);
	ROS_INFO("Joint 2 = %f", angles.Actuator2);
	ROS_INFO("Joint 3 = %f", angles.Actuator3);

	ROS_INFO("Joint 4 = %f", angles.Actuator4);
	ROS_INFO("Joint 5 = %f", angles.Actuator5);
	ROS_INFO("Joint 6 = %f", angles.Actuator6);
}

/*!
 * \brief Dumps the current cartesian positions onto the screen.  
 */
void JacoComm::PrintPosition(CartesianInfo position)
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

/*! 
 * \brief Dumps the current finger positions onto the screen.  
 */
void JacoComm::PrintFingers(FingersPosition fingers)
{
	ROS_INFO("Jaco Arm Finger Positions");
	ROS_INFO("Finger 1 = %f", fingers.Finger1);
	ROS_INFO("Finger 2 = %f", fingers.Finger2);
	ROS_INFO("Finger 3 = %f", fingers.Finger3);
}

/*!
 * \brief Dumps the client configuration onto the screen.  
 */
void JacoComm::PrintConfig(ClientConfigurations config)
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
	ROS_INFO("DrinkingLength = %f", config.DrinkingLenght);
	ROS_INFO("DeletePreProgrammedPositionsAtRetract = %d", config.DeletePreProgrammedPositionsAtRetract);
	ROS_INFO("EnableFlashErrorLog = %d", config.EnableFlashErrorLog);
	ROS_INFO("EnableFlashPositionLog = %d", config.EnableFlashPositionLog);
}

void JacoComm::Stop()
{
	software_stop = true;

	API->StartControlAPI();

	JoystickCommand home_command;
	memset(&home_command, 0, sizeof(home_command)); //zero structure

	home_command.ButtonValue[2] = 1;
	API->SendJoystickCommand(home_command);
	API->EraseAllTrajectories();
	ros::Duration(0.05).sleep();

	home_command.ButtonValue[2] = 0;
	API->SendJoystickCommand(home_command);
}

void JacoComm::Start()
{
	software_stop = false;

	API->StartControlAPI();

}

bool JacoComm::Stopped()
{
	return software_stop;
}

/*!
 * \brief Wait for the arm to reach the "home" position.
 *
 * \param timeout Timeout after which to give up waiting for arm to finish "homing".
 */
void JacoComm::WaitForHome(int timeout)
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
            ros::spinOnce();
		} 
		else
		{
			current_sec = (double) time(NULL);
		}

		if (HomeState())
		{
			ros::Duration(1.0).sleep();  // Grants a bit more time for the arm to "settle"
			return;
		}
	}

	ROS_WARN("Timed out waiting for arm to return \"home\"");
}


bool JacoComm::ComparePositions(const CartesianInfo &first, const CartesianInfo &second, float tolerance)
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

bool JacoComm::CompareAngles(const AngularInfo &first, const AngularInfo &second, float tolerance)
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

bool JacoComm::CompareValues(float first, float second, float tolerance)
{
	return ((first <= second + tolerance) && (first >= second - tolerance));
}

bool JacoComm::CompareAngularValues(float first, float second, float tolerance)
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

} // namespace jaco
