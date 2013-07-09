//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================


#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"

float get_xmlrpc_value(XmlRpc::XmlRpcValue &value)
{
	if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
	{
		return static_cast<double>(value);
	}
	else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
	{
		return (float) static_cast<int>(value);
	}

	throw std::string("Parameter 'home_position' must contain only numerical values");
	return 0.0;
}

jaco::JacoAngles get_home_position(ros::NodeHandle &nh)
{
	std::string key;
	jaco::JacoAngles home; 

	//if (nh.searchParam("~home_position", key))
	if (ros::param::has("~home_position"))
	{
		XmlRpc::XmlRpcValue joints_list;
		ros::param::get("~home_position", joints_list);
		ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_ASSERT(joints_list.size() == 6);

		try
		{
			home.Actuator1 = get_xmlrpc_value(joints_list[0]);
			home.Actuator2 = get_xmlrpc_value(joints_list[1]);
			home.Actuator3 = get_xmlrpc_value(joints_list[2]);
			home.Actuator4 = get_xmlrpc_value(joints_list[3]);
			home.Actuator5 = get_xmlrpc_value(joints_list[4]);
			home.Actuator6 = get_xmlrpc_value(joints_list[5]);

			return home;
		}
		catch (std::string msg)
		{
			ROS_ERROR("%s", msg.c_str());
			// use default home from below
		}
	}

	// typical home position for a "right-handed" arm
	home.Actuator1 = 282.8;
	home.Actuator2 = 154.4;
	home.Actuator3 = 43.1;
	home.Actuator4 = 230.7;
	home.Actuator5 = 83.0;
	home.Actuator6 = 78.1;

	return home;
}

int main(int argc, char **argv)
{
	/* Set up ROS */
	ros::init(argc, argv, "jaco_arm_driver");
	ros::NodeHandle nh;

	jaco::JacoComm comm(get_home_position(nh));

	ROS_INFO("Initializing the Arm");

	comm.HomeArm();
	comm.InitializeFingers();

	//create the arm object
	jaco::JacoArm jaco(comm, nh);
	jaco::JacoPoseActionServer pose_server(comm, nh);
	jaco::JacoAnglesActionServer angles_server(comm, nh);
	jaco::JacoFingersActionServer fingers_server(comm, nh);

	ros::spin();
}

