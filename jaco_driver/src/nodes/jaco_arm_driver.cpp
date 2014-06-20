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


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "jaco_arm_driver");
        ros::NodeHandle nh("~");

        while (ros::ok())
        {
            jaco::JacoComm comm(nh);
            jaco::JacoArm jaco(comm, nh);
            jaco::JacoPoseActionServer pose_server(comm, nh);
            jaco::JacoAnglesActionServer angles_server(comm, nh);
            jaco::JacoFingersActionServer fingers_server(comm, nh);

            ros::spin();
            ros::Duration(1.0).sleep();
        }
    } catch(const std::exception& e) {
        ROS_ERROR_STREAM("Exception:\n" << e.what());
    }
    return 0;
}
