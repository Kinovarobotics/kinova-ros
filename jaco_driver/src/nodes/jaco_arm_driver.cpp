//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================

#include "jaco_driver/jaco_api.h"
#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_comm.h"
#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaco_arm_driver");
    ros::NodeHandle nh("~");
    
    boost::recursive_mutex api_mutex;
    std::string api_lib; 
    nh.param<std::string>("api_lib", api_lib, "");

    bool is_first_init = true;
    while (ros::ok())
    {
        try
        {
            jaco::JacoComm comm(nh, api_mutex, is_first_init, api_lib);
            jaco::JacoIKSolver ik;
            //jaco::JacoQPSolver qp;
            //jaco::JacoMPCController mpc_controller(comm, ik, qp, nh);
            jaco::JacoKinematicController kinematic_controller(comm, ik, nh);
            jaco::JacoArm jaco(comm, nh, kinematic_controller);
            jaco::JacoPoseActionServer pose_server(comm, nh);
            jaco::JacoAnglesActionServer angles_server(comm, nh);
            jaco::JacoFingersActionServer fingers_server(comm, nh);

			
            ros::spin();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
            jaco::JacoAPI api(api_lib);
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            api.closeAPI();
            ros::Duration(1.0).sleep();
        }

        is_first_init = false;
    }
    return 0;
}
