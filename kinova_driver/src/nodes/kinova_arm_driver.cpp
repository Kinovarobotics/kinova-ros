//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_arm_pose_action.h"
#include "kinova_driver/kinova_arm_joints_action.h"
#include "kinova_driver/kinova_fingers_action.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinova_arm_driver");
    ros::NodeHandle nh("~");
    boost::recursive_mutex api_mutex;

    bool is_first_init = true;
    while (ros::ok())
    {
        try
        {
            kinova::KinovaComm comm(nh, api_mutex, is_first_init);
            kinova::KinovaArm jaco(comm, nh);
            kinova::KinovaPoseActionServer pose_server(comm, nh);
            kinova::KinovaAnglesActionServer angles_server(comm, nh);
            kinova::KinovaFingersActionServer fingers_server(comm, nh);

            ros::spin();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
            kinova::KinovaAPI api;
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            api.closeAPI();
            ros::Duration(1.0).sleep();
        }

        is_first_init = false;
    }
    return 0;
}
