/*
 * jaco_tf_updater.cpp

 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */
#include <jaco_driver/jaco_tf_updater.h>

using namespace jaco;

JacoTFTree::JacoTFTree(ros::NodeHandle nh, ros::NodeHandle param_nh)
{

	std::string joint_angles_topic;
	nh.param<std::string>("joint_angles_topic", joint_angles_topic, "joint_angles");

	//Print out received topics
	ROS_DEBUG("Got Joint Angles Topic Name: <%s>", joint_angles_topic.c_str());

	ROS_INFO("Starting Up Jaco TF Updater...");

	this->joint_angles_sub = nh.subscribe(joint_angles_topic, 1, &JacoTFTree::JointAnglesMSG, this);
	current_angles.Angle_J1 = 0;
	current_angles.Angle_J2 = 0;
	current_angles.Angle_J3 = 0;
	current_angles.Angle_J4 = 0;
	current_angles.Angle_J5 = 0;
	current_angles.Angle_J6 = 0;
	last_angle_update = ros::Time().now();
	this->tf_update_timer = nh.createTimer(ros::Duration(0.01), &JacoTFTree::TFUpdateTimer, this);
	tf_update_timer.stop();

}

void JacoTFTree::JointAnglesMSG(const jaco_driver::JointAnglesConstPtr& joint_angles)
{
	current_angles.Angle_J1 = joint_angles->Angle_J1;
	current_angles.Angle_J2 = joint_angles->Angle_J2;
	current_angles.Angle_J3 = joint_angles->Angle_J3;
	current_angles.Angle_J4 = joint_angles->Angle_J4;
	current_angles.Angle_J5 = joint_angles->Angle_J5;
	current_angles.Angle_J6 = joint_angles->Angle_J6;
	last_angle_update = ros::Time().now();
	tf_update_timer.start();
}

void JacoTFTree::CalculatePostion(void)
{

	//Update the forward Kinematics
	kinematics.UpdateForward(kinematics.deg_to_rad(current_angles.Angle_J1),
			kinematics.deg_to_rad(current_angles.Angle_J2), kinematics.deg_to_rad(current_angles.Angle_J3),
			kinematics.deg_to_rad(current_angles.Angle_J4), kinematics.deg_to_rad(current_angles.Angle_J5),
			kinematics.deg_to_rad(current_angles.Angle_J6));

}

void JacoTFTree::TFUpdateTimer(const ros::TimerEvent&)
{
	this->CalculatePostion();	//Update TF Tree

	if ((ros::Time().now().toSec() - last_angle_update.toSec()) > 1)
	{
		tf_update_timer.stop();
	}

}

int main(int argc, char **argv)
{

	/* Set up ROS */
	ros::init(argc, argv, "jaco_tf_updater");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");

	//create the arm object
	JacoTFTree JacoTF(nh, param_nh);

	ros::spin();
}
