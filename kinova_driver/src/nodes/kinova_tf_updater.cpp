/*
 * kinova_tf_updater.cpp

 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#include <kinova_driver/kinova_tf_updater.h>
#include <kinova_driver/kinova_ros_types.h>

namespace kinova
{

KinovaTFTree::KinovaTFTree(ros::NodeHandle node_handle, std::string& kinova_robotType)
    : kinematics_(node_handle, kinova_robotType)
{
    joint_angles_subscriber_ = node_handle.subscribe("in/joint_angles", 1,
                                                     &KinovaTFTree::jointAnglesMsgHandler, this);
    current_angles_.joint1 = 0;
    current_angles_.joint2 = 0;
    current_angles_.joint3 = 0;
    current_angles_.joint4 = 0;
    current_angles_.joint5 = 0;
    current_angles_.joint6 = 0;
    current_angles_.joint7 = 0;
    last_angle_update_ = ros::Time().now();
    tf_update_timer_ = node_handle.createTimer(ros::Duration(0.01),
                                               &KinovaTFTree::tfUpdateHandler, this);
    tf_update_timer_.stop();
}


void KinovaTFTree::jointAnglesMsgHandler(const kinova_msgs::JointAnglesConstPtr& joint_angles)
{
    current_angles_.joint1 = joint_angles->joint1;
    current_angles_.joint2 = joint_angles->joint2;
    current_angles_.joint3 = joint_angles->joint3;
    current_angles_.joint4 = joint_angles->joint4;
    current_angles_.joint5 = joint_angles->joint5;
    current_angles_.joint6 = joint_angles->joint6;
    current_angles_.joint7 = joint_angles->joint7;
    last_angle_update_ = ros::Time().now();
    tf_update_timer_.start();
}


void KinovaTFTree::calculatePostion(void)
{
    // Update the forward Kinematics
    float Q[7] = {kinematics_.degToRad(current_angles_.joint1),
                 kinematics_.degToRad(current_angles_.joint2),
                 kinematics_.degToRad(current_angles_.joint3),
                 kinematics_.degToRad(current_angles_.joint4),
                 kinematics_.degToRad(current_angles_.joint5),
                 kinematics_.degToRad(current_angles_.joint6),
                 kinematics_.degToRad(current_angles_.joint7)};

    kinematics_.updateForward(Q);
}


void KinovaTFTree::tfUpdateHandler(const ros::TimerEvent&)
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
    ros::init(argc, argv, "kinova_tf_updater");
    ros::NodeHandle nh("~");

    std::string kinova_robotType = "";

    // Retrieve the (non-option) argument:
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robotType provided in the argument!" << std::endl;
        return -1;
    }
    else // there is an input...
    {
        kinova_robotType = argv[argc-1];
        ROS_INFO("kinova_robotType is %s.", kinova_robotType.c_str());
    }

    kinova::KinovaTFTree KinovaTF(nh, kinova_robotType);
    ros::spin();

    return 0;
}
