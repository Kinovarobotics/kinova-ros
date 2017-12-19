/*
 * kinovalib.h
 *
 *  Created on: Feb 16, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#ifndef KINOVA_DRIVER_KINOVA_API_H
#define KINOVA_DRIVER_KINOVA_API_H

#include <dlfcn.h>
#include <ros/ros.h>

#include <iostream>

#include "kinova/Kinova.API.USBCommandLayerUbuntu.h"
#include "kinova/Kinova.API.EthCommandLayerUbuntu.h"
#include "kinova/KinovaTypes.h"


namespace kinova
{

#define KINOVA_USB_LIBRARY  "USBCommandLayerUbuntu.so"
#define KINOVA_COMM_USB_LIBRARY "USBCommLayerUbuntu.so"

#define KINOVA_ETH_LIBRARY  "EthCommandLayerUbuntu.so"
#define KINOVA_COMM_ETH_LIBRARY "EthCommLayerUbuntu.so"


enum KinovaAPIType {
    USB = 0,
    ETHERNET
};

class KinovaAPI
{
public:
    KinovaAPI(){}
    int initializeKinovaAPIFunctions(KinovaAPIType connection_type);
    int loadLibraries(const char *API_command_lib, const char *kinova_comm_lib);
    // %Tag(general function)%

    int (*initAPI)(void);

    //note - there is already an Ethernet_initAPI()
    //this one is also needed to init ethernet connection
    int (*initEthernetAPI)(EthernetCommConfig & config);
    int (*closeAPI)(void);
    int (*startControlAPI)();
    int (*stopControlAPI)();
    int (*startForceControl)();
    int (*stopForceControl)();
    int (*restoreFactoryDefault)();
    int (*sendJoystickCommand)(JoystickCommand);
    int (*getJoystickValue)(JoystickCommand &);

    int (*getCodeVersion)(int[CODE_VERSION_COUNT]);
    int (*getAPIVersion)(int[API_VERSION_COUNT]);
    int (*getDeviceCount)(int &); // only api directly link to CommLayerUbuntu.so
    int (*getDevices)(KinovaDevice[MAX_KINOVA_DEVICE], int &);
    int (*setActiveDevice)(KinovaDevice);
    int (*refresDevicesList)(void);
    int (*getControlType)(int &);
    int (*getClientConfigurations)(ClientConfigurations &);
    int (*setClientConfigurations)(ClientConfigurations);
    int (*setFrameType)(int);
    int (*startCurrentLimitation)();
    int (*stopCurrentLimitation)();

    int (*getGeneralInformations)(GeneralInformations &);
    int (*getQuickStatus)(QuickStatus &);
    int (*getForcesInfo)(ForcesInfo &);
    int (*getSensorsInfo)(SensorsInfo &);
    int (*getGripperStatus)(Gripper &);
    int (*getCommandVelocity)(float[CARTESIAN_SIZE], float[MAX_ACTUATORS]);    

    // %EndTag(general function)%


    // %Tag(joint angular)%

    int (*setAngularControl)();
    int (*getAngularCommand)(AngularPosition &);
    int (*getAngularPosition)(AngularPosition &);
    int (*getAngularVelocity)(AngularPosition &);
    int (*getAngularAcceleration)(AngularAcceleration &);

    int (*getAngularForce)(AngularPosition &);    

    int (*getAngularCurrent)(AngularPosition &);
    int (*getAngularCurrentMotor)(AngularPosition &);
    int (*getPositionCurrentActuators)(float[POSITION_CURRENT_COUNT]);
    int (*setActuatorPID)(unsigned int, float, float, float);

    // %EndTag(joint angular)%


    // %Tag(tool cartesian)%

    int (*setCartesianControl)();
    int (*getCartesianCommand)(CartesianPosition &);
    int (*getCartesianPosition)(CartesianPosition &);

    int (*getCartesianForce)(CartesianPosition &);
    int (*setCartesianForceMinMax)(CartesianInfo, CartesianInfo);
    int (*setCartesianInertiaDamping)(CartesianInfo, CartesianInfo);

    int (*getEndEffectorOffset)(unsigned int&, float&, float&, float&);
    int (*setEndEffectorOffset)(unsigned int, float, float, float);

    int (*getActualTrajectoryInfo)(TrajectoryPoint &);
    int (*getGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*sendAdvanceTrajectory)(TrajectoryPoint);
    int (*sendBasicTrajectory)(TrajectoryPoint);
    int (*eraseAllTrajectories)();
    int (*eraseAllProtectionZones)();
    int (*getProtectionZone)(ZoneList &);
    int (*setProtectionZone)(ZoneList);


    //! 7 dof - Enables control mode where robot moves in null space using the joystick
    int (*StartRedundantJointNullSpaceMotion)();

    //! 7 dof - Disables control mode where robot moves in null space using the joystick
    int (*StopRedundantJointNullSpaceMotion)();

    //! Only works for 7 dof for now - Activate(state =1) or deactivates (state =0) the
    //!  avoidance of robot self-collisions
    int (*ActivateCollisionAutomaticAvoidance)(int state);

    //! Only works for 7 dof for now - Activates (state =1) or deactivates (state =0)
    //! the automatic avoidance of robot singularities (but the fitness function stays active).
    int (*ActivateSingularityAutomaticAvoidance)(int state);

    //! 7 dof - Activates (state =1) or deactivates (state =0)  the fitness function 7 dof robot
    int (*ActivateAutoNullSpaceMotionCartesian)(int state);

    // %EndTag(tool cartesian)%


    // %Tag(pre-defined)%

    int (*moveHome)();
    int (*initFingers)();

    // %EndTag(pre-defined)%


    //%Tag(Torque control)%

    int (*switchTrajectoryTorque)(GENERALCONTROL_TYPE);
    int (*sendAngularTorqueCommand)(float[COMMAND_SIZE]);
    int (*setTorqueZero)(int actuator_address);
    int (*sendCartesianForceCommand)(float[COMMAND_SIZE]);

    //Torque Parameters
    int (*setAngularTorqueMinMax)(AngularInfo, AngularInfo);
    int (*setTorqueSafetyFactor)(float);

    //! @brief Sets COM and COMxyz for all links
    //! @arg command[42] - {m1,m2..m7,x1,x2,..x7,y1,y2,...,y7,z1,z2,...z7}
    int (*setGravityManualInputParam)(float command[GRAVITY_PARAM_SIZE]);
    int (*runGravityZEstimationSequence)(ROBOT_TYPE, double[OPTIMAL_Z_PARAM_SIZE]);
    int (*runGravityZEstimationSequence7DOF)(ROBOT_TYPE type,
                                   float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF]);
     int (*setGravityOptimalZParam)(float[GRAVITY_PARAM_SIZE]);
    //%EndTag(Torque control)%


    // The following APIs are not wrapped in kinova_comm, users should call kinova_api with extra caution.


    // %Tag(less interest in ROS)%

    int (*setControlMapping)(ControlMappingCharts);
    int (*getControlMapping)(ControlMappingCharts &);

    // %EndTag(less interest in ROS)%


    // %Tag(not function, place holder, etc)%

    int (*getSingularityVector)(SingularityVector &); // old style, not constantly set to zeros
    int (*setActuatorMaxVelocity)(float[COMMAND_SIZE]); // no corresponding DSP code.
//    int (*setActuatorsPosition)(float*); // in Kinova.API.UsbCommand.h but not in source file.

    // %EndTag(not function, place holder, etc)%


    // %Tag(experimental)%
    // force/torque control are not tested in ROS and may lead to unexpect motion.
    // do not use these functions unless you are know exactly what you are doing.

    int (*setTorqueControlType)(TORQUECONTROL_TYPE);

    int (*setActuatorPIDFilter)(int, float, float, float);
    int (*setAngularInertiaDamping)(AngularInfo, AngularInfo);
    int (*getAngularForceGravityFree)(AngularPosition &);    
    int (*getAngularTorqueCommand)(float[COMMAND_SIZE]);
    int (*getAngularTorqueGravityEstimation)(float[COMMAND_SIZE]);
    int (*setTorqueActuatorGain)(float[COMMAND_SIZE]);
    int (*setTorqueActuatorDamping)(float[COMMAND_SIZE]);
    int (*setTorqueCommandMax)(float[COMMAND_SIZE]);    
    int (*setTorqueRateLimiter)(float[COMMAND_SIZE]);
    int (*setTorqueFeedCurrent)(float[COMMAND_SIZE]);
    int (*setTorqueFeedVelocity)(float[COMMAND_SIZE]);
    int (*setTorquePositionLimitDampingGain)(float[COMMAND_SIZE]);
    int (*setTorquePositionLimitDampingMax)(float[COMMAND_SIZE]);
    int (*setTorquePositionLimitRepulsGain)(float[COMMAND_SIZE]);
    int (*setTorquePositionLimitRepulsMax)(float[COMMAND_SIZE]);
    int (*setTorqueFilterVelocity)(float[COMMAND_SIZE]);
    int (*setTorqueFilterMeasuredTorque)(float[COMMAND_SIZE]);
    int (*setTorqueFilterError)(float[COMMAND_SIZE]);
    int (*setTorqueFilterControlEffort)(float[COMMAND_SIZE]);

    int (*setGravityVector)(float[GRAVITY_VECTOR_SIZE]);
    int (*setGravityType)(GRAVITY_TYPE);
    int (*setSwitchThreshold)(float[COMMAND_SIZE]);
    int (*setPositionLimitDistance)(float[COMMAND_SIZE]);
    int (*setGravityPayload)(float[GRAVITY_PAYLOAD_SIZE]);
    int (*setTorqueVibrationController)(float);
    int (*setTorqueRobotProtection)(int);

    int (*getTrajectoryTorqueMode)(int &);
    int (*setTorqueInactivityType)(int);

    // %EndTag(experimental)%

  private:
    KinovaAPIType API_type_;
    void *API_command_lib_;
    void *kinova_comm_lib_;

    void* initCommandLayerFunction(const char* name);
    void* initCommLayerFunction(const char* name);
};


}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_API_H
