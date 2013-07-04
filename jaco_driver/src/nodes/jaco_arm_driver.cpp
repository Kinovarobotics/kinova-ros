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
	std::string arm_pose_topic, joint_velocity_topic, joint_angles_topic, cartesian_velocity_topic,
		tool_position_topic, set_finger_position_topic, finger_position_topic, joint_state_topic,
		set_joint_angle_topic;

	nh.param<std::string>("arm_pose_topic", arm_pose_topic, "arm_pose");
	nh.param<std::string>("joint_velocity_topic", joint_velocity_topic, "joint_velocity");
	nh.param<std::string>("joint_angles_topic", joint_angles_topic, "joint_angles");
	nh.param<std::string>("cartesian_velocity_topic", cartesian_velocity_topic, "cartesian_velocity");
	nh.param<std::string>("tool_position_topic", tool_position_topic, "tool_position");
	nh.param<std::string>("set_finger_position_topic", set_finger_position_topic, "set_finger_position");
	nh.param<std::string>("finger_position_topic", finger_position_topic, "finger_position");
	nh.param<std::string>("joint_state_topic", joint_state_topic, "joint_state");
	nh.param<std::string>("set_joint_angle_topic", set_joint_angle_topic, "set_joint_angle");

	//Print out received topics
	ROS_DEBUG("Got Arm Position Topic Name: <%s>", arm_pose_topic.c_str());
	ROS_DEBUG("Got Joint Velocity Topic Name: <%s>", joint_velocity_topic.c_str());
	ROS_DEBUG("Got Joint Angles Topic Name: <%s>", joint_angles_topic.c_str());
	ROS_DEBUG("Got Cartesian Velocity Topic Name: <%s>", cartesian_velocity_topic.c_str());
	ROS_DEBUG("Got Tool Position Topic Name: <%s>", tool_position_topic.c_str());
	ROS_DEBUG("Got Set Finger Position Topic Name: <%s>", set_finger_position_topic.c_str());
	ROS_DEBUG("Got Finger Position Topic Name: <%s>", finger_position_topic.c_str());
	ROS_DEBUG("Got Joint State Topic Name: <%s>", joint_state_topic.c_str());
	ROS_DEBUG("Got Set Joint Angle Topic Name: <%s>", set_joint_angle_topic.c_str());

	ROS_INFO("Starting Up Jaco Arm Controller...");

	previous_state = 0;

	/* Set up Services */
	stop_service = nh.advertiseService("stop", &JacoArm::StopSRV, this);
	start_service = nh.advertiseService("start", &JacoArm::StartSRV, this);
	homing_service = nh.advertiseService("home_arm", &JacoArm::HomeArmSRV, this);

	/* Set Default Configuration */

	// API->RestoreFactoryDefault(); // uncomment comment ONLY if you want to lose your settings on each launch.

	ClientConfigurations configuration;
	arm.GetConfig(configuration);
	arm.PrintConfig(configuration);

	ROS_INFO("Initializing the Arm");

	last_update_time = ros::Time::now();
	update_time = ros::Duration(5.0);

	arm.HomeArm();
	arm.InitializeFingers();

	/* Storing arm in home position */

	/* Set up Publishers */
	JointAngles_pub = nh.advertise<jaco_driver::JointAngles>(joint_angles_topic, 2);
	JointState_pub = nh.advertise<sensor_msgs::JointState>(joint_state_topic, 2);
	ToolPosition_pub = nh.advertise<geometry_msgs::PoseStamped>(tool_position_topic, 2);
	FingerPosition_pub = nh.advertise<jaco_driver::FingerPosition>(finger_position_topic, 2);

	/* Set up Subscribers*/
	ArmPose_sub = nh.subscribe(arm_pose_topic, 1, &JacoArm::PoseMSG_Sub, this);
	JointVelocity_sub = nh.subscribe(joint_velocity_topic, 1, &JacoArm::VelocityMSG, this);
	CartesianVelocity_sub = nh.subscribe(cartesian_velocity_topic, 1, &JacoArm::CartesianVelocityMSG, this);
	SetFingerPosition_sub = nh.subscribe(set_finger_position_topic, 1, &JacoArm::SetFingerPositionMSG, this);
	SetJoint_sub = nh.subscribe(set_joint_angle_topic, 1, &JacoArm::SetJointAnglesMSG, this);

	status_timer = nh.createTimer(ros::Duration(0.05), &JacoArm::StatusTimer, this);

	joint_vel_timer = nh.createTimer(ros::Duration(0.01), &JacoArm::JointVelTimer, this);
	joint_vel_timer.stop();
	joint_vel_timer_flag = false;
	cartesian_vel_timer = nh.createTimer(ros::Duration(0.01), &JacoArm::CartesianVelTimer, this);
	cartesian_vel_timer.stop();
	cartesian_vel_timer_flag = false;

	ROS_INFO("The Arm is ready to use.");

	TrajectoryPoint Jaco_Velocity;

	memset(&Jaco_Velocity, 0, sizeof(Jaco_Velocity)); //zero structure

	arm.SetCartesianVelocities(Jaco_Velocity.Position.CartesianPosition);
}

JacoArm::~JacoArm()
{
}

bool JacoArm::HomeArmSRV(jaco_driver::HomeArm::Request &req, jaco_driver::HomeArm::Response &res)
{
	arm.HomeArm();
	res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";

	return true;
}

/*!
 * \brief Displays the cartesian coordinates of the arm before and after a transform.
 */
void JacoArm::PoseMSG_Sub(const geometry_msgs::PoseStampedConstPtr& arm_pose)
{
	CartesianInfo Jaco_Position;
	memset(&Jaco_Position, 0, sizeof(Jaco_Position)); //zero structure

	if (!arm.Stopped())
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

		if (ros::ok()
				&& !listener.canTransform("/jaco_api_origin", arm_pose->header.frame_id,
						arm_pose->header.stamp))
		{
			ROS_ERROR("Could not get transfrom from /jaco_api_origin to %s, aborting cartesian movement", arm_pose->header.frame_id.c_str());
			return;
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
			last_update_time = ros::Time::now();
			arm.SetPosition(Jaco_Position);
		}
	}
}

/*!
 * \brief Receives ROS command messages and relays them to SetFingers.
 */
void JacoArm::SetFingerPositionMSG(const jaco_driver::FingerPositionConstPtr& finger_pos)
{
	if (!arm.Stopped())
	{
		FingersPosition Finger_Position;
		memset(&Finger_Position, 0, sizeof(Finger_Position)); //zero structure

		Finger_Position.Finger1 = finger_pos->Finger_1;
		Finger_Position.Finger2 = finger_pos->Finger_2;
		Finger_Position.Finger3 = finger_pos->Finger_3;

		arm.SetFingers(Finger_Position);
	}
}

/*!
 * \brief Receives ROS command messages and relays them to SetAngles.
 */
void JacoArm::SetJointAnglesMSG(const jaco_driver::JointAnglesConstPtr& angles)
{
	if (!arm.Stopped())
	{
		AngularInfo Joint_Position;
		memset(&Joint_Position, 0, sizeof(Joint_Position)); //zero structure

		Joint_Position.Actuator1 = angles->Angle_J1;
		Joint_Position.Actuator2 = angles->Angle_J2;
		Joint_Position.Actuator3 = angles->Angle_J3;
		Joint_Position.Actuator4 = angles->Angle_J4;
		Joint_Position.Actuator5 = angles->Angle_J5;
		Joint_Position.Actuator6 = angles->Angle_J6;

		arm.SetAngles(Joint_Position);
	}
}


void JacoArm::VelocityMSG(const jaco_driver::JointVelocityConstPtr& joint_vel)
{
	if (!arm.Stopped())
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

/*!
 * \brief Handler for "stop" service.
 *
 * Instantly stops the arm and prevents further movement until start service is
 * invoked.
 */
bool JacoArm::StopSRV(jaco_driver::Stop::Request &req, jaco_driver::Stop::Response &res)
{
	arm.Stop();
	res.stop_result = "JACO ARM HAS BEEN STOPPED";
	ROS_DEBUG("JACO ARM STOP REQUEST");

	return true;
}

/*!
 * \brief Handler for "start" service.
 *
 * Re-enables control of the arm after a stop.
 */
bool JacoArm::StartSRV(jaco_driver::Start::Request &req, jaco_driver::Start::Response &res)
{
	arm.Start();
	res.start_result = "JACO ARM CONTROL HAS BEEN ENABLED";
	ROS_DEBUG("JACO ARM START REQUEST");

	return true;
}


void JacoArm::CartesianVelocityMSG(const geometry_msgs::TwistStampedConstPtr& cartesian_vel)
{
	if (!arm.Stopped())
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
	arm.SetCartesianVelocities(cartesian_velocities);

	if ((ros::Time().now().toSec() - last_cartesian_update.toSec()) > 1)
	{
		cartesian_vel_timer.stop();
		cartesian_vel_timer_flag = false;
	}
}

void JacoArm::JointVelTimer(const ros::TimerEvent&)
{
	arm.SetVelocities(joint_velocities);

	if ((ros::Time().now().toSec() - last_joint_update.toSec()) > 1)
	{
		joint_vel_timer.stop();
		joint_vel_timer_flag = false;
	}
}

/*!
 * \brief Contains coordinates for an alternate "Home" position
 *
 * GoHome() function must be enabled in the initialization routine for this to
 * work.
 */
void JacoArm::GoHome(void)
{
/*
	AngularInfo joint_home;

	joint_home.Actuator1 = 176.0;
	joint_home.Actuator2 = 111.0;
	joint_home.Actuator3 = 107.0;
	joint_home.Actuator4 = 459.0;
	joint_home.Actuator5 = 102.0;
	joint_home.Actuator6 = 106.0;

	SetAngles(joint_home, 10); //send joints to home position

	API->SetCartesianControl();
*/
}

/*!
 * \brief Publishes the current joint angles.
 *
 * Joint angles are published in both their raw state as obtained from the arm
 * (JointAngles), and transformed & converted to radians (joint_state) as per
 * the Jaco Kinematics PDF.
 *
 * JointState will eventually also publish the velocity and effort for each
 * joint, when this data is made available by the C++ API.  Currenty velocity
 * and effort are reported as being zero (0.0) for all joints.
 */
void JacoArm::BroadCastAngles(void)
{
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
	arm.GetAngles(arm_angles.Actuators);

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

/*!
 * \brief Publishes the current cartesian coordinates
 */
void JacoArm::BroadCastPosition(void)
{
	CartesianPosition position;
	geometry_msgs::PoseStamped current_position;

	memset(&position, 0, sizeof(position)); //zero structure

	arm.GetPosition(position.Coordinates);

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
	arm.GetPosition(Jaco_Position.Coordinates);

	finger_position.Finger_1 = Jaco_Position.Fingers.Finger1;
	finger_position.Finger_2 = Jaco_Position.Fingers.Finger2;
	finger_position.Finger_3 = Jaco_Position.Fingers.Finger3;

	FingerPosition_pub.publish(finger_position);
}


void JacoArm::StatusTimer(const ros::TimerEvent&)
{
	BroadCastAngles();
	BroadCastPosition();
	BroadCastFingerPosition();
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

