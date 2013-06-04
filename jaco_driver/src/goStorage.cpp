/*
 * goStorage.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: bpwiselybabu
 */

#include <jaco_driver/goStorage.h>

goStorage::goStorage():nh_()
{
	incollect_=false;			//initialize as if you are not in collect;

	getRosParam();				//load all the ros params, basically the joint angles and the
								//the robot topic

	//subscribe to the robot topic
	robot_sub_ = nh_.subscribe(robot_topic_.c_str(), 2, &goStorage::systemCb, this);
	ROS_INFO("subscribing to: %s",robot_topic_.c_str());
	joint_pub_=nh_.advertise<jaco_driver::joint_angles>(joint_topic_.c_str(), 10);
	ROS_INFO("publishing to: %s",joint_topic_.c_str());
}
void goStorage::systemCb(const aero_srr_msgs::AeroStateConstPtr& status)
{
	ROS_INFO("In state: %d",status->state);
	//if the state of the robot is to look for home
	if((status->state==aero_srr_msgs::AeroState::COLLECT)||(status->state==aero_srr_msgs::AeroState::STARTUP))
	{
		ROS_INFO("flag set");
		incollect_=true;						//activate the beacon detector
	}
	if((incollect_)&&(status->state==aero_srr_msgs::AeroState::SEARCH))
	{
		incollect_=false;
		//publish the joints into the topic
		ROS_INFO("publishing the go to home joints");
		joint_pub_.publish(angles_);
	}
	if((incollect_)&&(status->state==aero_srr_msgs::AeroState::HOME))
	{
		incollect_=false;
		//publish the joints into the topic
		ROS_INFO("publishing the go to home joints");
		joint_pub_.publish(angles_);
	}
}
goStorage::~goStorage() {
	// TODO Auto-generated destructor stub
}
void goStorage::getRosParam()
{
	//Options to/exposed
	//robot_topic 		- the topic on which the robot state needs to be subscribed
	//joint_topic		- the joint states topic that you need to publish

	ros::NodeHandle pnh("~");							//handle to the local param list
	if(!pnh.getParam("state_topic", robot_topic_))				//initialize var from launch file
	{
		ROS_ERROR("state_topic not set in launch file");
	}
	if(!pnh.getParam("joint_topic", joint_topic_))				//initialize var from launch file
	{
		ROS_ERROR("joint_topic not set in launch file");
	}
	double angle;
	if(!pnh.getParam("joint1", angle))				//initialize var from launch file
	{
		ROS_WARN("joint1 not set using default of 0.0");
		angle=0.0;
	}
	angles_.Angle_J1=angle;
	if(!pnh.getParam("joint2", angle))				//initialize var from launch file
	{
		ROS_WARN("joint2 not set using default of 0.0");
		angle=0.0;
	}
	angles_.Angle_J2=angle;

	if(!pnh.getParam("joint3", angle))				//initialize var from launch file
	{
		ROS_WARN("joint3 not set using default of 0.0");
		angle=0.0;
	}
	angles_.Angle_J3=angle;

	if(!pnh.getParam("joint4", angle))				//initialize var from launch file
	{
		ROS_WARN("joint4 not set using default of 0.0");
		angle=0.0;
	}
	angles_.Angle_J4=angle;

	if(!pnh.getParam("joint5", angle))				//initialize var from launch file
	{
		ROS_WARN("joint5 not set using default of 0.0");
		angle=0.0;
	}
	angles_.Angle_J5=angle;
	if(!pnh.getParam("joint6", angle))				//initialize var from launch file
	{
		ROS_WARN("joint6 not set using default of 0.0");
		angle=0.0;
	}
	angles_.Angle_J6=angle;

}

