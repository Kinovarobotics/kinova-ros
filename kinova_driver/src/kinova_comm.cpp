/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_comm.cpp
 *  Desc: Class for moving/querying kinova arm.
 *  Auth: Alex Bencz, Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#include <ros/ros.h>
#include "kinova_driver/kinova_comm.h"
#include <string>
#include <vector>
#include <tf/tf.h>
#include <arpa/inet.h>

namespace kinova
{

KinovaComm::KinovaComm(const ros::NodeHandle& node_handle,
                   boost::recursive_mutex &api_mutex,
                   const bool is_movement_on_start,
                   const std::string &kinova_robotType)
    : is_software_stop_(false), api_mutex_(api_mutex)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    int result = NO_ERROR_KINOVA;

    //initialize kinova api functions
    std::string api_type;
    node_handle.param<std::string>("connection_type", api_type, "USB");
    if (api_type == "USB")
      kinova_api_.initializeKinovaAPIFunctions(USB);
    else
      kinova_api_.initializeKinovaAPIFunctions(ETHERNET);


    //Set ethernet parameters
    EthernetCommConfig ethernet_settings;
    std::string local_IP,subnet_mask;
    int local_cmd_port,local_bcast_port;
    node_handle.getParam("ethernet/local_machine_IP", local_IP);
    node_handle.getParam("ethernet/subnet_mask", subnet_mask);
    node_handle.getParam("ethernet/local_cmd_port", local_cmd_port);
    node_handle.getParam("ethernet/local_broadcast_port", local_bcast_port);
    ethernet_settings.localCmdport = local_cmd_port;
    ethernet_settings.localBcastPort = local_bcast_port;
    ethernet_settings.localIpAddress = inet_addr(local_IP.c_str());
    ethernet_settings.subnetMask = inet_addr(subnet_mask.c_str());
    ethernet_settings.rxTimeOutInMs = 1000;
    ethernet_settings.robotIpAddress = inet_addr("192.168.100.11");
    ethernet_settings.robotPort = 55000;

    // Get the serial number parameter for the arm we wish to connect to
    std::string serial_number = "";
    node_handle.getParam("serial_number", serial_number);

    int api_version[API_VERSION_COUNT];
    result = kinova_api_.getAPIVersion(api_version);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Kinova API version", result);
    }

    ROS_INFO_STREAM("Initializing Kinova "<< api_type.c_str()
                    << " API (header version: " << COMMAND_LAYER_VERSION
                    << ", library version: " << api_version[0] << "."
                                             << api_version[1] << "." << api_version[2] << ")");

    if (api_type == "USB"){
      result = kinova_api_.initAPI();
    }
    else{
        result =kinova_api_.initEthernetAPI(ethernet_settings);
    }

    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not initialize Kinova API", result);
    }

    result = kinova_api_.refresDevicesList();

    result = NO_ERROR_KINOVA;
    int devices_count = kinova_api_.getDevices(devices_list_, result);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get devices list", result);
    }

    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get devices list count.", result);
    }

    bool found_arm = false;
    for (int device_i = 0; device_i < devices_count; device_i++)
    {
        // If no device is specified, just use the first available device
        if (serial_number == "" || serial_number == "not_set" ||
            std::strcmp(serial_number.c_str(), devices_list_[device_i].SerialNumber) == 0)
        {
            result = kinova_api_.setActiveDevice(devices_list_[device_i]);
            if (result != NO_ERROR_KINOVA)
            {
                throw KinovaCommException("Could not set the active device", result);
            }

            GeneralInformations general_info;
            result = kinova_api_.getGeneralInformations(general_info);
            if (result != NO_ERROR_KINOVA)
            {
                throw KinovaCommException("Could not get general information about the device", result);
            }

            ClientConfigurations configuration;
            getConfig(configuration);

            QuickStatus quick_status;
            getQuickStatus(quick_status);

            robot_type_ = quick_status.RobotType;

            ROS_INFO_STREAM("Found " << devices_count << " device(s), using device at index " << device_i
                            << " (model: " << configuration.Model
                            << ", serial number: " << devices_list_[device_i].SerialNumber
                            << ", code version: " << general_info.CodeVersion
                            << ", code revision: " << general_info.CodeRevision << ")");

            found_arm = true;
            break;
        }
    }

    if (!found_arm)
    {
        ROS_ERROR("Could not find the specified arm (serial: %s) among the %d attached devices",
                  serial_number.c_str(), devices_count);
        throw KinovaCommException("Could not find the specified arm", 0);
    }

    //find the number of joints and fingers of the arm using robotType passed from arm node
    num_joints_ = kinova_robotType[3]-'0';
    num_fingers_ = kinova_robotType[5]-'0';

    // On a cold boot the arm may not respond to commands from the API right away.
    // This kick-starts the Control API so that it's ready to go.
    startAPI();
    stopAPI();
    startAPI();

    //Set robot to use manual COM parameters
    bool use_estimated_COM;
    node_handle.param("torque_parameters/use_estimated_COM_parameters",
                          use_estimated_COM,true);
    if (use_estimated_COM == true)
        kinova_api_.setGravityType(OPTIMAL);
    else
        kinova_api_.setGravityType(MANUAL_INPUT);

    //Set torque safety factor to 1
    kinova_api_.setTorqueSafetyFactor(1);


    // Set the angular velocity of each of the joints to zero
    TrajectoryPoint kinova_velocity;
    memset(&kinova_velocity, 0, sizeof(kinova_velocity));
    setCartesianVelocities(kinova_velocity.Position.CartesianPosition);

    if (is_movement_on_start)
    {
        initFingers();
    }
    else
    {
        ROS_WARN("Movement on connection to the arm has been suppressed on initialization. You may "
                 "have to home the arm (through the home service) before movement is possible");
    }
}


KinovaComm::~KinovaComm()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    kinova_api_.closeAPI();
}


/**
 * @brief This function tells the robotical arm that from now on, the API will control the robotical arm. It must been call before sending trajectories or joystick command.
 */
void KinovaComm::startAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    if (is_software_stop_)
    {
        is_software_stop_ = false;
        kinova_api_.stopControlAPI();
        ros::Duration(0.05).sleep();
    }

    int result = kinova_api_.startControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not start the control API", result);
    }
}


/**
 * @brief This function tells the robotical arm the from now on the API is not controlling the robotical arm.
 */
void KinovaComm::stopAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    is_software_stop_ = true;

    int result = kinova_api_.stopControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not stop the control API", result);
    }

    result = kinova_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not erase all trajectories", result);
    }
}


/**
 * @brief check if API lost the control of robot.
 * @return true if stopAPI() was called.
 */
bool KinovaComm::isStopped()
{
    return is_software_stop_;
}


/**
 * @brief This function activates the reactive force control for admittance control. Admittance control may be applied to joint or Cartesian depending to the control mode.
 * @warning You can only use this function if your robotic device has torque sensors on it. Also, the robotic device must be in a standard vertical position.
 */
void KinovaComm::startForceControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.startForceControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not start force control.", result);
    }
}


/**
 * @brief This function stops the admittance control. Admittance control may be applied to joint or Cartesian depending to the control mode.
 */
void KinovaComm::stopForceControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.stopForceControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not stop force control.", result);
    }
}


/**
 * @brief get robotType
 * Index for robot type:
 *  JACOV1_ASSISTIVE = 0,
 *  MICO_6DOF_SERVICE = 1,
 *  MICO_4DOF_SERVICE = 2,
 * 	JACOV2_6DOF_SERVICE = 3,
 * 	JACOV2_4DOF_SERVICE = 4,
 * 	MICO_6DOF_ASSISTIVE = 5,
 * 	JACOV2_6DOF_ASSISTIVE = 6,
 * @return index of robot type
 */
int KinovaComm::robotType() const
{
    return robot_type_;
}


/**
 * @brief This function gets information regarding some status flag of the robotical arm.
 * @param quick_status This structure holds various informations but mostly it is flag status, such as robotype, retractType, forceControlStatus,
 */
void KinovaComm::getQuickStatus(QuickStatus &quick_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&quick_status, 0, sizeof(quick_status));  // zero structure
    int result = kinova_api_.getQuickStatus(quick_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get quick status", result);
    }
}


/**
 * @brief This function set the client configurations of the robotical arm. The configuration data is stored on the arm itself.
 * @param config config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
 */
void KinovaComm::setConfig(const ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the client configuration", result);
    }
}


/**
 * @brief obtain the current client configuration.
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
 */
void KinovaComm::getConfig(ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&config, 0, sizeof(config));  // zero structure

    int result = kinova_api_.getClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get client configuration", result);
    }
}


/**
 * @brief Dumps the client configuration onto the screen.
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
 */
void KinovaComm::printConfig(const ClientConfigurations &config)
{
    ROS_INFO_STREAM("Arm configuration:\n"
                    "\tClientID: " << config.ClientID <<
                    "\n\tClientName: " << config.ClientName <<
                    "\n\tOrganization: " << config.Organization <<
                    "\n\tSerial:" << config.Serial <<
                    "\n\tModel: " << config.Model <<
                    "\n\tMaxForce: " << config.MaxForce <<
                    "\n\tSensibility: " <<  config.Sensibility <<
                    "\n\tDrinkingHeight: " << config.DrinkingHeight <<
                    "\n\tComplexRetractActive: " << config.ComplexRetractActive <<
                    "\n\tRetractedPositionAngle: " << config.RetractedPositionAngle <<
                    "\n\tRetractedPositionCount: " << config.RetractedPositionCount <<
                    "\n\tDrinkingDistance: " << config.DrinkingDistance <<
                    "\n\tFingers2and3Inverted: " << config.Fingers2and3Inverted <<
                    "\n\tDrinkingLength: " << config.DrinkingLenght <<
                    "\n\tDeletePreProgrammedPositionsAtRetract: " <<
                    config.DeletePreProgrammedPositionsAtRetract <<
                    "\n\tEnableFlashErrorLog: " << config.EnableFlashErrorLog <<
                    "\n\tEnableFlashPositionLog: " << config.EnableFlashPositionLog);
}


/**
 * @brief get current control type
 * The control in ROS is independent to Joystick control type. For example: even set robot in joint control though api in ROS, robot may still controlled in joint level by joystick.
 * @param controlType Cartesian control[0] or joint control[1]
 */
void KinovaComm::getControlType(int &controlType)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&controlType, 0, sizeof(controlType)); //zero structure
    int result = kinova_api_.getControlType(controlType);
    if (result!=NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get control type", result);
    }
}


/**
 * @brief get almost all information of the robotical arm.
 * @param general_info includes: power statistics, sensor infos, robot position and command, etc.
 */
void KinovaComm::getGeneralInformations(GeneralInformations &general_info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&general_info, 0, sizeof(general_info));
    int result = kinova_api_.getGeneralInformations(general_info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get general information about the device", result);
    }
}


/**
 * @brief This function returns information about the robotical arm's sensors. (Voltage, Total current, Temperature, acceleration)
 * @param The structure containing the sensor's informations
 */
void KinovaComm::getSensorsInfo(SensorsInfo &sensor_Info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&sensor_Info, 0, sizeof(sensor_Info));
    int result = kinova_api_.getSensorsInfo(sensor_Info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get sensors information", result);
    }
}


/**
 * @brief This function gets information regarding all forces.
 * @param force_Info A struct containing the information about the forces. Joint torque and end-effector wrench in Nm and N.
 */
void KinovaComm::getForcesInfo(ForcesInfo &force_Info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&force_Info, 0, sizeof(force_Info));
    int result = kinova_api_.getForcesInfo(force_Info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get force information", result);
    }
}


/**
 * @brief This function gets informations about the robotical arm's gripper. Some information may be missing, it is still in development.
 * @param gripper_status A struct containing the information of the gripper. Most information of each fingers, including model, motion, force, limits, etc.
 */
void KinovaComm::getGripperStatus(Gripper &gripper_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&gripper_status, 0, sizeof(gripper_status));
    int result = kinova_api_.getGripperStatus(gripper_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the gripper status", result);
    }
}




/**
 * @brief This function sets the robotical arm in angular control mode.
 * If robot is not in motion, change control model to Angular control
 */
void KinovaComm::setAngularControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    TrajectoryFIFO trajectory_fifo;
    memset(&trajectory_fifo, 0, sizeof(trajectory_fifo));
    getGlobalTrajectoryInfo(trajectory_fifo);
    if(trajectory_fifo.TrajectoryCount > 0)
    {
        ROS_WARN("Current tranjectory count is %d, Please wait the trajectory to finish to swich to Angular control.", trajectory_fifo.TrajectoryCount);
        return;
    }
    int result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set angular control", result);
    }
}


/**
 * @brief This function get the angular command of all actuators.
 * @param An AngularPosition struct containing the values. Units are degrees. angular_command {AngularInfo, FingersPosition}
 * @return
 *        - @ref NO_ERROR_KINOVA if operation is a success
 *        - @ref ERROR_API_NOT_INITIALIZED if the API has not been initialized. To initialize it, call the InitAPI() function.
 */
void KinovaComm::getAngularCommand(AngularPosition &angular_command)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&angular_command, 0, sizeof(angular_command));
    int result = kinova_api_.getAngularCommand(angular_command);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular command", result);
    }
}


/**
 * @brief This function returns the angular position of the robotical arm's end effector.
 * @param angles A structure that contains the position of each actuator. Unit in degrees.
 */
void KinovaComm::getJointAngles(KinovaAngles &angles)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_angles;
    memset(&kinova_angles, 0, sizeof(kinova_angles));  // zero structure

    int result = kinova_api_.getAngularPosition(kinova_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular position", result);
    }
    angles = KinovaAngles(kinova_angles.Actuators);
}


/**
 * @brief Sends a joint angle command to the Kinova arm.
 * This function sends trajectory point(Angular) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * @param angles target joint angle to set, type float, unit in degree
 * @param timeout default value 0.0, not used.
 * @param push default true, errase all trajectory before request motion..
 */
void KinovaComm::setJointAngles(const KinovaAngles &angles, double speedJoint123, double speedJoint4567, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_WARN_STREAM("In class [" << typeid(*this).name() << "], function ["<< __FUNCTION__ << "]: The angles could not be set because the arm is stopped" << std::endl);
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_joint;
    kinova_joint.InitStruct();
    memset(&kinova_joint, 0, sizeof(kinova_joint));  // zero structure

    if (push)
    {
        result = kinova_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw KinovaCommException("Could not erase trajectories", result);
        }
    }

    //startAPI();

    result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set angular control", result);
    }

    kinova_joint.Position.Delay = 0.0;
    kinova_joint.Position.Type = ANGULAR_POSITION;
    kinova_joint.Position.Actuators = angles;
    kinova_joint.Limitations.speedParameter1 = speedJoint123;
    kinova_joint.Limitations.speedParameter2 = speedJoint4567;
    kinova_joint.LimitationsActive = 1;

    result = kinova_api_.sendAdvanceTrajectory(kinova_joint);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint angle trajectory", result);
    }

}


/**
 * @brief This function get the velocity of each actuator.
 * @param vels A kinovaAngles structure contains joint velocity of each actuator. Unit in degrees/second.
 */
void KinovaComm::getJointVelocities(KinovaAngles &vels)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_vels;
    memset(&kinova_vels, 0, sizeof(kinova_vels));  // zero structure

    int result = kinova_api_.getAngularVelocity(kinova_vels);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular velocity", result);
    }

    vels = KinovaAngles(kinova_vels.Actuators);

    //velocities reported back by firmware seem to be half of actual value
    vels.Actuator1 = vels.Actuator1*2;
    vels.Actuator2 = vels.Actuator2*2;
    vels.Actuator3 = vels.Actuator3*2;
    vels.Actuator4 = vels.Actuator4*2;
    vels.Actuator5 = vels.Actuator5*2;
    vels.Actuator6 = vels.Actuator6*2;
    vels.Actuator7 = vels.Actuator7*2;
}


/**
 * @brief This function controls robot with joint velocity.
 * This function sends trajectory point(ANGULAR_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * @param joint_vel joint velocity in degree/second
 */
void KinovaComm::setJointVelocities(const AngularInfo &joint_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The velocities could not be set because the arm is stopped");
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = ANGULAR_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.Actuators = joint_vel;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint velocity trajectory", result);
    }
}

void KinovaComm::setJointTorques(float joint_torque[])
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The joint torques could not be set because the arm is stopped");
        return;
    }

    //memset(&joint_torque, 0, sizeof(joint_torque));  // zero structure

    //startAPI();
    //ROS_INFO("Torque %f %f %f %f %f %f %f ", joint_torque[0],joint_torque[1],joint_torque[2],
     //       joint_torque[3],joint_torque[4],joint_torque[5],joint_torque[6]);
    int result = kinova_api_.sendAngularTorqueCommand(joint_torque);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send joint torques", result);
    }
}

int KinovaComm::sendCartesianForceCommand(float force_cmd[COMMAND_SIZE])
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    if (isStopped())
    {
        ROS_INFO("The force cmd could not be set because the arm is stopped");
        return 0;
    }

    //memset(&joint_torque, 0, sizeof(joint_torque));  // zero structure

    //startAPI();
    //ROS_INFO("Force %f %f %f %f %f %f", force_cmd[0],force_cmd[1],force_cmd[2],
     //       force_cmd[3],force_cmd[4],force_cmd[5]);
    int result = kinova_api_.sendCartesianForceCommand(force_cmd);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send force cmd", result);
    }
    return result;
}



/**
 * @brief This function get the accelerometer values of each actuator. It does not directly refer to the angular acceleration.
 * @param joint_acc An AngularAcceleration struct containing the values. Units are in G
 */
void KinovaComm::getJointAccelerations(AngularAcceleration &joint_acc)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&joint_acc, 0, sizeof(joint_acc));
    int result = kinova_api_.getAngularAcceleration(joint_acc);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get joint acceleration.", result);
    }
}


/**
 * @brief This function returns the torque of each actuator.
 * @param tqs A structure that contains the torque of each actuator. Unit is Newton * meter
 */
void KinovaComm::getJointTorques(KinovaAngles &tqs)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_tqs;
    memset(&kinova_tqs, 0, sizeof(kinova_tqs));  // zero structure

    int result = kinova_api_.getAngularForce(kinova_tqs);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the joint torques", result);
    }

    tqs = KinovaAngles(kinova_tqs.Actuators);
}

void KinovaComm::getGravityCompensatedTorques(KinovaAngles &tqs)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_tqs;
    memset(&kinova_tqs, 0, sizeof(kinova_tqs));  // zero structure

    int result = kinova_api_.getAngularForceGravityFree(kinova_tqs);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the joint torques", result);
    }

    tqs = KinovaAngles(kinova_tqs.Actuators);
}


/**
 * @brief This function returns the current that each actuator consume on the main supply.
 * @param anguler_current A structure that contains the current of each actuator and finger. Unit in A.
 */
void KinovaComm::getJointCurrent(AngularPosition &anguler_current)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&anguler_current, 0, sizeof(anguler_current));
    int result = kinova_api_.getAngularCurrent(anguler_current);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the current in joints", result);
    }
}

/**
  *@brief Set zero torque for all joints
 */
void KinovaComm::setZeroTorque()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int actuator_address[] = {16,17,18,19,20,21,25};
    int result;
    for (int i=0;i<num_joints_;i++)
    {
        result = kinova_api_.setTorqueZero(actuator_address[i]);
    }
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set zero torques", result);
    }
    ROS_WARN("Torques for all joints set to zero");
}


/**
 * @brief This function set the angular torque's maximum and minimum values.
 * @param min A struct that contains all angular minimum values. (Unit: N * m)
 * @param max 6 A struct that contains all angular max values.     (Unit: N * m)
 */
void KinovaComm::setJointTorqueMinMax(AngularInfo &min, AngularInfo &max)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ROS_INFO("Setting min torues - %f %f %f %f %f %f %f", min.Actuator1,
              min.Actuator2,min.Actuator3,min.Actuator4,min.Actuator5,
              min.Actuator6,min.Actuator7);
    ROS_INFO("Setting max torues - %f %f %f %f %f %f %f", max.Actuator1,
              max.Actuator2,max.Actuator3,max.Actuator4,max.Actuator5,
              max.Actuator6,max.Actuator7);

    int result = kinova_api_.setAngularTorqueMinMax(min, max);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the limits for joint torques", result);
    }
}

/**
 * @brief setPayload
 * @param payload Array - Mass, COMx, COMy, COMz
 */
void KinovaComm::setPayload(std::vector<float> payload)
{
    float payload_[4];
    std::copy(payload.begin(), payload.end(), payload_);
    ROS_INFO("Payload set to - %f %f %f %f", payload_[0],payload_[1],
            payload_[2],payload_[3]);
    int result = kinova_api_.setGravityPayload(payload_);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the gravity payload", result);
    }
}

/**
 * @brief Safety factor defines a velocity threshold at which torque control switches to position control
 * @param factor between 0 and 1
 */
void KinovaComm::setToquesControlSafetyFactor(float factor)
{
    ROS_INFO("Setting torque safety factor to %f", factor);
    int result = kinova_api_.setTorqueSafetyFactor(factor);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set torque safety factor", result);
    }
}

/** @brief Sets COM and COMxyz for all links
  * @arg command[42] - {m1,m2..m7,x1,x2,..x7,y1,y2,...,y7,z1,z2,...z7}
//! */
void KinovaComm::setRobotCOMParam(GRAVITY_TYPE type,std::vector<float> params)
{
    float com_parameters[GRAVITY_PARAM_SIZE];
    memset(&com_parameters, 0, sizeof(com_parameters));
    std::ostringstream com_params;
    com_params<<"Setting COM parameters to ";
    for (int i=0; i<params.size(); i++)
    {
        com_parameters[i] = params[i];
        com_params<<params[i]<<", ";
    }
    ROS_INFO_STREAM(com_params.str());
    int result;
    if (type == MANUAL_INPUT)
        result = kinova_api_.setGravityManualInputParam(com_parameters);
    else
        result = kinova_api_.setGravityOptimalZParam(com_parameters);
    if (result != NO_ERROR_KINOVA && result!=2005)
    {
        throw KinovaCommException("Could not set the COM parameters", result);
    }
}

/**
* @brief This function is used to run a sequence to estimate the optimal gravity parameters when the robot is
* standing (Z).

The arm must be in Trajectory-Position mode before to launch the procedure.

Before using this procedure, you should make sure that the torque sensors are well calibrated. This procedure is
explained in the user guide and in the Advanced Specification Guide.

When the program is launched, the robot will execute a trajectory. The user must remain alert and turn off the
robot if something wrong occurs (for example if the robot collides with an object). When the program ends, it will
output the parameters in the console and in a text file named “ParametersOptimal_Z.txt” in the program folder.
These parameters can then be sent as input to the function SetOptimalZParam().
*
* @param type The robot type
* @param OptimalzParam The result of the sequence
*/
int KinovaComm::runCOMParameterEstimation(ROBOT_TYPE type)
{
    float COMparams[GRAVITY_PARAM_SIZE];
    memset(&COMparams[0],0,sizeof(COMparams));
    int result;
    if(type == SPHERICAL_7DOF_SERVICE)
    {
        ROS_INFO("Running 7 dof robot COM estimation sequence");
        result = kinova_api_.runGravityZEstimationSequence7DOF(type,COMparams);
    }
    else
    {
        double params[OPTIMAL_Z_PARAM_SIZE];
        ROS_INFO("Running COM estimation sequence");
        result = kinova_api_.runGravityZEstimationSequence(type,params);
        for (int i=0;i<OPTIMAL_Z_PARAM_SIZE;i++)
            COMparams[i] = (float)params[i];
    }
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not launch COM parameter estimation sequence", result);
    }
    result = kinova_api_.setGravityOptimalZParam(COMparams);
    if (result != NO_ERROR_KINOVA && result!=2005)
    {
        throw KinovaCommException("Could not set COM Parameters", result);
    }
}


/**
 * @brief Dumps the current joint angles onto the screen.
 * @param angles A structure contains six joint angles. Unit in degrees
 */
void KinovaComm::printAngles(const KinovaAngles &angles)
{
    ROS_INFO("Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f, J7: %f \n",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6,
             angles.Actuator7);

    ROS_INFO("Joint angles (rad) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f, J7: %f \n",
             angles.Actuator1/180.0*M_PI, angles.Actuator2/180.0*M_PI, angles.Actuator3/180.0*M_PI,
             angles.Actuator4/180.0*M_PI, angles.Actuator5/180.0*M_PI, angles.Actuator6/180.0*M_PI,
             angles.Actuator7/180.0*M_PI);
}




/**
 * @brief This function sets the robotical arm in cartesian control mode if this is possible.
 * If robot is not in motion, change control model to Cartesian control
 */
void KinovaComm::setCartesianControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    TrajectoryFIFO trajectory_fifo;
    memset(&trajectory_fifo, 0, sizeof(trajectory_fifo));
    getGlobalTrajectoryInfo(trajectory_fifo);
    if(trajectory_fifo.TrajectoryCount > 0)
    {
        ROS_WARN("Current tranjectory count is %d, Please wait the trajectory to finish to swich to Cartesian control.", trajectory_fifo.TrajectoryCount);
        return;
    }
    int result = kinova_api_.setCartesianControl();
    ROS_WARN("%d", result);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set Cartesian control", result);
    }
}


/**
 * @brief This function get the cartesian command of the end effector. The Cartesian orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
 * @param cartesian_command An CartesianPosition struct containing the values of end-effector and fingers.
 *
 * @htmlonly
 *
 * <table border="0" cellspacing="10">
 * <tr>
 * <th>Member</th>
 * <th>Unit</th>
 * </tr>
 * <tr><td width="50">X</td><td>meter</td></tr>
 * <tr><td>Y</td><td>meter</td></tsr>
 * <tr><td>Z</td><td>meter</td></tr>
 * <tr><td>Theta X</td><td>RAD</td></tr>
 * <tr><td>Theta Y</td><td>RAD</td></tr>
 * <tr><td>Theta Z</td><td>RAD</td></tr>
 * <tr><td>Finger 1</td><td>No unit</td></tr>
 * <tr><td>Finger 2</td><td>No unit</td></tr>
 * <tr><td>Finger 3</td><td>No unit</td></tr>
 * </table>
 *
 * @endhtmlonly
 */
void KinovaComm::getCartesianCommand(CartesianPosition &cartesian_command)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&cartesian_command, 0, sizeof(cartesian_command));
    int result = kinova_api_.getCartesianCommand(cartesian_command);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Cartesian command", result);
    }
}


/**
 * @brief This function returns the cartesian position of the robotical arm's end effector.
 * In KinovaPose, orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
 * @param position pose in [X,Y,Z,ThetaX,ThetaY,ThetaZ] form, Units in meters and radians.
 */
void KinovaComm::getCartesianPosition(KinovaPose &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_position;
    memset(&kinova_cartesian_position, 0, sizeof(kinova_cartesian_position));  // zero structure

    int result = kinova_api_.getCartesianPosition(kinova_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Cartesian position", result);
    }

//    ROS_INFO_STREAM_ONCE("Cartesian pose in [X,Y,Z, ThetaX, ThetaY, ThetaZ] is : " << kinova_cartesian_position.Coordinates.X << ", "
//                    << kinova_cartesian_position.Coordinates.Y << ", "
//                    << kinova_cartesian_position.Coordinates.Z << ", "
//                    << kinova_cartesian_position.Coordinates.ThetaX << ", "
//                    << kinova_cartesian_position.Coordinates.ThetaY << ", "
//                    << kinova_cartesian_position.Coordinates.ThetaZ << std::endl);

    position = KinovaPose(kinova_cartesian_position.Coordinates);
}


/**
 * @brief Sends a cartesian coordinate trajectory to the Kinova arm.
 * This function sends trajectory point(Cartesian) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendBasicTrajectory() is called in api to complete the motion.
 * In KinovaPose, orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
 * @param pose target pose of robot [X,Y,Z, ThetaX, ThetaY, ThetaZ], unit in meter and radians.
 * @param timeout default 0.0, not used.
 * @param push default false, does not erase previous trajectory point before new motion. If you want to erase all trajectory before request motion, set to true..
 */
void KinovaComm::setCartesianPosition(const KinovaPose &pose, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_WARN_STREAM("In class [" << typeid(*this).name() << "], function ["<< __FUNCTION__ << "]: The pose could not be set because the arm is stopped" << std::endl);
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_pose;
    kinova_pose.InitStruct();
    memset(&kinova_pose, 0, sizeof(kinova_pose));  // zero structure

    if (push)
    {
        result = kinova_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw KinovaCommException("Could not erase trajectories", result);
        }
    }

    //startAPI();

    result = kinova_api_.setCartesianControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set Cartesian control", result);
    }

    kinova_pose.Position.Delay = 0.0;
    kinova_pose.Position.Type = CARTESIAN_POSITION;
//    kinova_pose.Position.HandMode = HAND_NOMOVEMENT;
    kinova_pose.Position.CartesianPosition = pose;

    result = kinova_api_.sendBasicTrajectory(kinova_pose);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send basic trajectory", result);
    }
}


/**
 * @brief Linear and angular velocity control in Cartesian space
 * This function sends trajectory point(CARTESIAN_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
 * @param velocities unit are meter/second for linear velocity and radians/second for "Omega".
 */
void KinovaComm::setCartesianVelocities(const CartesianInfo &velocities)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The cartesian velocities could not be set because the arm is stopped");
        kinova_api_.eraseAllTrajectories();
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.CartesianPosition = velocities;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced Cartesian velocity trajectory", result);
    }
}

/**
 * @brief Linear and angular velocity control in Cartesian space
 * This function sends trajectory point(CARTESIAN_VELOCITY) that will be added in the robotical arm's FIFO. Waits until the arm has stopped moving before releasing control of the API. sendAdvanceTrajectory() is called in api to complete the motion.
 * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
 * @param velocities unit are meter/second for linear velocity and radians/second for "Omega".
 * @param fingers finger positions to reach at the same time as moving, in closure percentage
 */
void KinovaComm::setCartesianVelocitiesWithFingers(const CartesianInfo &velocities, const FingerAngles& fingers)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The cartesian velocities could not be set because the arm is stopped");
        kinova_api_.eraseAllTrajectories();
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.CartesianPosition = velocities;

    // Fill fingers
    kinova_velocity.Position.Fingers = fingers;
    kinova_velocity.Position.HandMode = POSITION_MODE;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced Cartesian velocity trajectory", result);
    }
}


/**
 * @brief This function returns the max translation(X, Y and Z) velocity of the robot's end effector in ClientConfigurations
 * @return MaxTranslationVelocity Unit in meter per second
 */
float KinovaComm::getMaxTranslationVelocity()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    return configuration.MaxTranslationVelocity;
}


/**
 * @brief This function set the max translation(X, Y and Z) velocity of the robot's end effector in ClientConfigurations
 * @param max_trans_vel Unit in meter per second
 */
void KinovaComm::setMaxTranslationVelocity(const float &max_trans_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    usleep(100000);
    configuration.MaxTranslationVelocity = max_trans_vel;
    setConfig(configuration);
}


/**
 * @brief This function get max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
 * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
 * @return Unit in rad/second
 */
float KinovaComm::getMaxOrientationVelocity()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    return configuration.MaxOrientationVelocity;
}


/**
 * @brief This function set max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
 * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
 * @param max_orient_vel Unit in rad/second
 */
void KinovaComm::setMaxOrientationVelocity(const float &max_orient_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    usleep(100000);
    configuration.MaxOrientationVelocity = max_orient_vel;
    setConfig(configuration);
}


/**
 * @brief This function returns the cartesian wrench at the robotical arm's end effector.
 * @param cart_force A structure that contains the wrench vector at the end effector. Unit in N and N * m.
 */
void KinovaComm::getCartesianForce(KinovaPose &cart_force)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_force;
    memset(&kinova_cartesian_force, 0, sizeof(kinova_cartesian_force));  // zero structure

    int result = kinova_api_.getCartesianForce(kinova_cartesian_force);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Cartesian force", result);
    }

    cart_force = KinovaPose(kinova_cartesian_force.Coordinates);
}


/**
 * @brief This function set the Cartesian force's maximum and minimum values.
 * @param min A struct that contains all Cartesian minimum values. (Translation unit: N     Orientation unit: N * m)
 * @param max A struct that contains all Cartesian maximum values. (Translation unit: N     Orientation unit: N * m)
 */
void KinovaComm::setCartesianForceMinMax(const CartesianInfo &min, const CartesianInfo& max)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setCartesianForceMinMax(min, max);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set cartesian min/max force.", result);
    }
}


/**
 * @brief This function set the Cartesian inertia and damping value.
 * @param inertia A struct that contains all Cartesian inertia values. (Translation unit: Kg,  Orientation unit: Kg * m^2)
 * @param damping A struct that contains all Cartesian damping values. (Translation unit: (N * s) / m,   Orientation unit: (N * s) / RAD)
 */
void KinovaComm::setCartesianInertiaDamping(const CartesianInfo &inertia, const CartesianInfo& damping)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setCartesianInertiaDamping(inertia, damping);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set cartesian inertia and damping", result);
    }
}


/**
 * @brief Dumps the current cartesian pose onto the screen.
 * In KinovaPose, orientation is expressed in Euler-XYZ convention (Rot=Rx*Ry*Rz). However, in ROS by default using Euler-ZYX. tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
 * @param position in [X,Y,Z,ThetaX,ThetaY,ThetaZ], where orientation is using Euler-ZYX convention.
 */
void KinovaComm::printPosition(const KinovaPose &position)
{
    ROS_INFO("Arm position\n"
             "\tposition (m) -- x: %f, y: %f z: %f\n"
             "\trotation (rad) -- theta_x: %f, theta_y: %f, theta_z: %f",
             position.X, position.Y, position.Z,
             position.ThetaX, position.ThetaY, position.ThetaZ);
}


/**
 * @brief This function extract the UserPosition from trajectory.
 * @param user_position contains POSITION_TYPE(Angular/Cartesian position/velocity, etc), CartesianInfo and AngularInfo, finger positions etc.
 */
void KinovaComm::getUserCommand(UserPosition &user_position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&user_position, 0, sizeof(user_position));
    TrajectoryPoint trajecory_point;
    int result = kinova_api_.getActualTrajectoryInfo(trajecory_point);
    if(result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get trajecory information", result);
    }
    user_position = trajecory_point.Position;
}


/**
 * @brief This function returns informations about the trajectories FIFO stored inside the robotical arm. Detail of trajectory point is not stored in trajectoryFIFO.
 * @param trajectoryFIFO The structure containing the FIFO's informations: {TrajectoryCount; UsedPercentage; MaxSize}.
 */
void KinovaComm::getGlobalTrajectoryInfo(TrajectoryFIFO &trajectoryFIFO)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&trajectoryFIFO, 0, sizeof(trajectoryFIFO));
    int result = kinova_api_.getGlobalTrajectoryInfo(trajectoryFIFO);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get trjectoryFIFO.", result);
    }
}


/**
 * @brief This function erases all the trajectories inside the robotical arm's FIFO. All trajectory will be cleared including angular, cartesian and fingers.
 */
void KinovaComm::eraseAllTrajectories()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not errase all trajectories.", result);
    }
}




/**
 * @brief This function get number of fingers. number of fingers determined by robotType. 3 fingers for robotType(0,3,4,6) and 2 fingers for robotType(1,2,5)
 * @return returns number of fingers.
 */
int KinovaComm::numFingers() const
{
    return num_fingers_;
}


/**
 * @brief This function obtain the joint position of fingers.
 * @param fingers in degrees, range from 0 to 6800
 */
void KinovaComm::getFingerPositions(FingerAngles &fingers)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_position;
    memset(&kinova_cartesian_position, 0, sizeof(kinova_cartesian_position));  // zero structure

    int result = kinova_api_.getCartesianPosition(kinova_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get Cartesian finger position", result);
    }

    if (num_fingers_ == 2)
    {
        kinova_cartesian_position.Fingers.Finger3 = 0.0;
    }

    fingers = FingerAngles(kinova_cartesian_position.Fingers);
}


/**
 * @brief This function sets the finger positions
 * The new finger position, combined with current joint values are constructed as a trajectory point. sendAdvancedTrajectory() is called in api to complete the motion.
 * @param fingers in degrees from 0 to about 6800
 * @param timeout timeout default 0.0, not used.
 * @param push default true, errase all trajectory before request motion.
 */
void KinovaComm::setFingerPositions(const FingerAngles &fingers, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The fingers could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    int control_type;
    result=kinova_api_.getControlType(control_type); // are we currently in angular or Cartesian mode? Response	0 = Cartesian control type, 1 = Angular control type.


    //initialize the trajectory point. same initialization for an angular or Cartesian point
    TrajectoryPoint kinova_point;
    kinova_point.InitStruct();
    memset(&kinova_point, 0, sizeof(kinova_point));  // zero structure

    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the current control type", result);
    }
    else
    {
	if (push)
    	{
        	result = kinova_api_.eraseAllTrajectories();
        	if (result != NO_ERROR_KINOVA)
        	{
           		throw KinovaCommException("Could not erase trajectories", result);
        	}
    	}
	// Initialize Cartesian control of the fingers
	kinova_point.Position.HandMode = POSITION_MODE;
	kinova_point.Position.Fingers = fingers;
	kinova_point.Position.Delay = 0.0;
	kinova_point.LimitationsActive = 0;
	if(control_type==0) //Cartesian
	{
		kinova_point.Position.Type = CARTESIAN_POSITION;
		CartesianPosition pose;
                memset(&pose, 0, sizeof(pose));  // zero structure   
		result = kinova_api_.getCartesianCommand(pose);
    		if (result != NO_ERROR_KINOVA)
    		{
        		throw KinovaCommException("Could not get the Cartesian position", result);
    		}    
		kinova_point.Position.CartesianPosition=pose.Coordinates;
	}
        else if(control_type==1) //angular
	{	
		kinova_point.Position.Type = ANGULAR_POSITION;	
		AngularPosition joint_angles;
    		memset(&joint_angles, 0, sizeof(joint_angles));  // zero structure    
		result = kinova_api_.getAngularCommand(joint_angles);
    		if (result != NO_ERROR_KINOVA)
    		{
        		throw KinovaCommException("Could not get the angular position", result);
    		}    
		kinova_point.Position.Actuators = joint_angles.Actuators;
	}
	else
	{ 
		throw KinovaCommException("Wrong control type", result);
	}  
    }
     

    // getAngularPosition will cause arm drop
    // result = kinova_api_.getAngularPosition(joint_angles);
       
    result = kinova_api_.sendBasicTrajectory(kinova_point);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced finger trajectory", result);
    }
}


/**
 * @brief Dumps the current finger agnles onto the screen.
 * @param fingers Unit in degrees 0 to 6800
 */
void KinovaComm::printFingers(const FingersPosition &fingers)
{
    ROS_INFO("Finger joint value -- F1: %f, F2: %f, F3: %f",
             fingers.Finger1, fingers.Finger2, fingers.Finger3);
}


/**
 * @brief This function move the arm to the "home" position.
 * The code replicates the function of the "home" button on the user controller by "pressing" the home button long enough for the arm to return to the home position.
 * @warning The home position is the default home, rather than user defined home.
 */
void KinovaComm::homeArm(void)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("Arm is stopped, cannot home");
        return;
    }
    else if (isHomed())
    {
        ROS_INFO("Arm is already in \"home\" position");
        return;
    }

    stopAPI();
    ros::Duration(1.0).sleep();
    startAPI();

    ROS_INFO("Homing the arm");
    kinova_api_.moveHome();

    /*JoystickCommand mycommand;
    mycommand.InitStruct();
    // In api mapping(observing with Jacosoft), home button is ButtonValue[2].
    mycommand.ButtonValue[2] = 1;
    for(int i = 0; i<2000; i++)
    {
        kinova_api_.sendJoystickCommand(mycommand);
        usleep(5000);

        // if (myhome.isCloseToOther(KinovaAngles(currentAngles.Actuators), angle_tolerance))
        if(isHomed())
        {
            ROS_INFO("Arm is in \"home\" position");
            // release home button.
            mycommand.ButtonValue[2] = 0;
            kinova_api_.sendJoystickCommand(mycommand);
            return;
        }
    }

    mycommand.ButtonValue[2] = 0;
    kinova_api_.sendJoystickCommand(mycommand);
    ROS_WARN("Homing arm timer out! If the arm is not in home position yet, please re-run home arm.");*/

}


/**
 * @brief Determines whether the arm has returned to its "Home" state. Checks the current joint angles, then compares them to the known "Home" joint angles.
 * @return true is robot is already in predefined "Home"configuration.
 * @warning The home position is the default home, rather than user defined home.
 */
bool KinovaComm::isHomed(void)
{
    QuickStatus quick_status;
    getQuickStatus(quick_status);

    if (quick_status.RetractType == RETRACT_TYPE_READY_STANDBY)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief This function initializes the fingers of the robotical arm. After the initialization, the robotical arm is in angular control mode. If you want to use the cartesian control mode, use the function setCartesianControl().
 * Move fingers to the full-open position to initialize them for use.
 * @warning This routine requires firmware version 5.05.x (or higher).
 */
void KinovaComm::initFingers(void)
{
    ROS_INFO("Initializing fingers...this will take a few seconds and the fingers should open completely");
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.initFingers();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not init fingers", result);
    }
    return;
}


/**
 * @brief This function Set the end effector offset's parameters. The end effector's offset is a translation offset applied to the end effector of the robotic arm.
 * @param status indicates if the offset is applied or not (0 = not applied, 1 = applied)
 * @param x Unit in meter
 * @param y Unit in meter
 * @param z Unit in meter
 */
void KinovaComm::setEndEffectorOffset(unsigned int status, float x, float y, float z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    int result = kinova_api_.setEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set end effector offset.", result);
    }
}


/**
 * @brief This function get the end effector offset's parameters. The end effector's offset is a translation offset applied to the end effector of the robotic arm.
 * @param status indicates if the offset is applied or not (0 = not applied, 1 = applied)
 * @param x Unit in meter
 * @param y Unit in meter
 * @param z Unit in meter
 */
void KinovaComm::getEndEffectorOffset(unsigned int &status, float &x, float &y, float &z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&status, 0, sizeof(status));
    memset(&x, 0, sizeof(x));
    memset(&y, 0, sizeof(y));
    memset(&z, 0, sizeof(z));
    int result = kinova_api_.getEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get current end effector offset.", result);
    }
}

void KinovaComm::SetTorqueControlState(int state)
{
    int result;
    if (state)
    {
        ROS_INFO("Switching to torque control");
        result = kinova_api_.switchTrajectoryTorque(TORQUE);
    }
    else
    {
        ROS_INFO("Switching to position control");
        result = kinova_api_.switchTrajectoryTorque(POSITION);
    }
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the torque control state", result);
    }
}

int KinovaComm::SelfCollisionAvoidanceInCartesianMode(int state)
{
    int result = kinova_api_.ActivateCollisionAutomaticAvoidance(state);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the self collision avoidance in cartesian mode", result);
    }
}

int KinovaComm::SingularityAvoidanceInCartesianMode(int state)
{
    int result = kinova_api_.ActivateSingularityAutomaticAvoidance(state);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the singularity avoidance in cartesian mode", result);
    }
}

int KinovaComm::SetRedundantJointNullSpaceMotion(int state)
{
    ROS_INFO("Setting null space mode to %d",state);
    int result;
    if (state)
        result = kinova_api_.StartRedundantJointNullSpaceMotion();
    else
        result = kinova_api_.StopRedundantJointNullSpaceMotion();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set redundant joint null space mode", result);
    }
}

int KinovaComm::SetRedundancyResolutionToleastSquares(int state)
{
    //Not Available in API
}


}  // namespace kinova
