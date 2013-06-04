/*
 * goStorage.h
 *
 *  Created on: Jun 3, 2013
 *      Author: bpwiselybabu
 */

#ifndef GOSTORAGE_H_
#define GOSTORAGE_H_

/* Headers ROS */
#include <ros/ros.h>
/* Headers Aero */
#include <aero_srr_msgs/AeroState.h>
#include <jaco_driver/joint_angles.h>
#include <jaco_driver/finger_position.h>

class goStorage
{
	/*ROS */
	std::string 						robot_topic_;							//the topic that will send the robot state message
	std::string							joint_topic_;					//the topic on which to publish the joint states
	ros::NodeHandle						nh_;							//global node handle;
	bool								incollect_;							//indicates if the robot is in collect state
	ros::Subscriber 					robot_sub_;							//robot status
	ros::Publisher						joint_pub_;							//joints topic to publish in
	/*Joint states */
	jaco_driver::joint_angles 			angles_;


public:
	goStorage();
	/*
	 * Load all the ros params!
	 */
	void getRosParam();
	/*
	 * system callback to monitor the state change
	 */
	void systemCb(const aero_srr_msgs::AeroStateConstPtr& status);

	virtual ~goStorage();
};

#endif /* GOSTORAGE_H_ */
