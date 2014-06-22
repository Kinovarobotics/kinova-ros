/*
 * jaco_tf_updater.cpp

 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */
#include <jaco_driver/jaco_tf_updater.h>

using namespace jaco;


JacoTFTree::JacoTFTree(ros::NodeHandle node_handle)
{
    ROS_INFO("Starting Up Jaco TF Updater...");

    this->joint_angles_sub = node_handle.subscribe("in/joint_angles", 1, &JacoTFTree::JointAnglesMSG, this);
    current_angles.joint1 = 0;
    current_angles.joint2 = 0;
    current_angles.joint3 = 0;
    current_angles.joint4 = 0;
    current_angles.joint5 = 0;
    current_angles.joint6 = 0;
    last_angle_update = ros::Time().now();
    this->tf_update_timer = node_handle.createTimer(ros::Duration(0.01), &JacoTFTree::TFUpdateTimer, this);
    tf_update_timer.stop();
}


void JacoTFTree::JointAnglesMSG(const jaco_msgs::JointAnglesConstPtr& joint_angles)
{
    current_angles.joint1 = joint_angles->joint1;
    current_angles.joint2 = joint_angles->joint2;
    current_angles.joint3 = joint_angles->joint3;
    current_angles.joint4 = joint_angles->joint4;
    current_angles.joint5 = joint_angles->joint5;
    current_angles.joint6 = joint_angles->joint6;
    last_angle_update = ros::Time().now();
    tf_update_timer.start();
}


void JacoTFTree::CalculatePostion(void)
{
    //Update the forward Kinematics
    kinematics.UpdateForward(kinematics.deg_to_rad(current_angles.joint1),
            kinematics.deg_to_rad(current_angles.joint2), kinematics.deg_to_rad(current_angles.joint3),
            kinematics.deg_to_rad(current_angles.joint4), kinematics.deg_to_rad(current_angles.joint5),
            kinematics.deg_to_rad(current_angles.joint6));

}


void JacoTFTree::TFUpdateTimer(const ros::TimerEvent&)
{
    this->CalculatePostion();  // Update TF Tree

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

    //create the arm object
    JacoTFTree JacoTF(nh);

    ros::spin();

    return 0;
}
