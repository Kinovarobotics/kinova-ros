/*
 * jaco_tf_updater.cpp

 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#include <kinova_driver/jaco_tf_updater.h>


namespace kinova
{

JacoTFTree::JacoTFTree(ros::NodeHandle node_handle)
    : kinematics_(node_handle)
{
    joint_angles_subscriber_ = node_handle.subscribe("in/joint_angles", 1,
                                                     &JacoTFTree::jointAnglesMsgHandler, this);
    current_angles_.joint1 = 0;
    current_angles_.joint2 = 0;
    current_angles_.joint3 = 0;
    current_angles_.joint4 = 0;
    current_angles_.joint5 = 0;
    current_angles_.joint6 = 0;
    last_angle_update_ = ros::Time().now();
    tf_update_timer_ = node_handle.createTimer(ros::Duration(0.01),
                                               &JacoTFTree::tfUpdateHandler, this);
    tf_update_timer_.stop();
}


void JacoTFTree::jointAnglesMsgHandler(const kinova_msgs::JointAnglesConstPtr& joint_angles)
{
    current_angles_.joint1 = joint_angles->joint1;
    current_angles_.joint2 = joint_angles->joint2;
    current_angles_.joint3 = joint_angles->joint3;
    current_angles_.joint4 = joint_angles->joint4;
    current_angles_.joint5 = joint_angles->joint5;
    current_angles_.joint6 = joint_angles->joint6;
    last_angle_update_ = ros::Time().now();
    tf_update_timer_.start();
}


void JacoTFTree::calculatePostion(void)
{
    // Update the forward Kinematics
    kinematics_.updateForward(kinematics_.degToRad(current_angles_.joint1),
                              kinematics_.degToRad(current_angles_.joint2),
                              kinematics_.degToRad(current_angles_.joint3),
                              kinematics_.degToRad(current_angles_.joint4),
                              kinematics_.degToRad(current_angles_.joint5),
                              kinematics_.degToRad(current_angles_.joint6));
}


void JacoTFTree::tfUpdateHandler(const ros::TimerEvent&)
{
    this->calculatePostion();  // Update TF Tree

    if ((ros::Time().now().toSec() - last_angle_update_.toSec()) > 1)
    {
        tf_update_timer_.stop();
    }
}

}  // namespace kinova


int main(int argc, char **argv)
{
    /* Set up ROS */
    ros::init(argc, argv, "jaco_tf_updater");
    ros::NodeHandle nh("~");

    kinova::JacoTFTree JacoTF(nh);
    ros::spin();

    return 0;
}
