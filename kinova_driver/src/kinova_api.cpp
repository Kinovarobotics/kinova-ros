/*
 * kinovalib.cpp
 *
 *  Created on: Mar 10, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#include <kinova_driver/kinova_api.h>
#include <vector>


namespace kinova
{

void* checkApiInit(void * usbLib, const char* name)
{
    void * function_pointer = dlsym(usbLib, name);
    assert(function_pointer != NULL);
    return function_pointer;
}


KinovaAPI::KinovaAPI(void)
{
    void *usbLib  = dlopen(KINOVA_USB_LIBRARY,  RTLD_NOW | RTLD_GLOBAL);
    void *commLib = dlopen(KINOVA_COMM_LIBRARY, RTLD_NOW | RTLD_GLOBAL);

    if ((usbLib == NULL) || (commLib == NULL))
    {
        ROS_FATAL("%s", dlerror());
    }

    // %Tag(general function)%

    initAPI = (int (*)())checkApiInit(usbLib, "InitAPI");

    closeAPI = (int (*)())checkApiInit(usbLib, "CloseAPI");

    startControlAPI = (int (*)())checkApiInit(usbLib, "StartControlAPI");

    stopControlAPI = (int (*)())checkApiInit(usbLib, "StopControlAPI");

    startForceControl = (int (*)())checkApiInit(usbLib, "StartForceControl");

    stopForceControl = (int (*)())checkApiInit(usbLib, "StopForceControl");

    restoreFactoryDefault = (int (*)())checkApiInit(usbLib, "RestoreFactoryDefault");

    sendJoystickCommand = (int (*)(JoystickCommand))checkApiInit(usbLib, "SendJoystickCommand");

    getJoystickValue = (int (*)(JoystickCommand &))checkApiInit(usbLib, "GetJoystickValue");


    getCodeVersion = (int (*)(int[CODE_VERSION_COUNT]))checkApiInit(usbLib, "GetCodeVersion");

    getAPIVersion = (int (*)(int[API_VERSION_COUNT]))checkApiInit(usbLib, "GetAPIVersion");

    getDeviceCount = (int (*)(int &))checkApiInit(commLib, "GetDeviceCount");

    getDevices = (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int &))checkApiInit(usbLib, "GetDevices");

    setActiveDevice = (int (*)(KinovaDevice))checkApiInit(usbLib, "SetActiveDevice");

    refresDevicesList = (int (*)())checkApiInit(usbLib, "RefresDevicesList");

    getControlType = (int (*)(int &)) checkApiInit(usbLib, "GetControlType");

    getClientConfigurations = (int (*)(ClientConfigurations &))checkApiInit(usbLib, "GetClientConfigurations");

    setClientConfigurations = (int (*)( ClientConfigurations))checkApiInit(usbLib, "SetClientConfigurations");

    setFrameType = (int (*)(int))checkApiInit(usbLib, "SetFrameType");

    startCurrentLimitation = (int (*)())checkApiInit(usbLib, "StartCurrentLimitation");

    stopCurrentLimitation = (int (*)())checkApiInit(usbLib, "StopCurrentLimitation");


    getGeneralInformations = (int (*)(GeneralInformations &))checkApiInit(usbLib, "GetGeneralInformations");

    getQuickStatus = (int (*)(QuickStatus &))checkApiInit(usbLib, "GetQuickStatus");

    getForcesInfo = (int (*)(ForcesInfo &))checkApiInit(usbLib, "GetForcesInfo");

    getSensorsInfo = (int (*)(SensorsInfo &))checkApiInit(usbLib, "GetSensorsInfo");

    getGripperStatus = (int (*)(Gripper &))checkApiInit(usbLib, "GetGripperStatus");

    getCommandVelocity = (int (*)(float[CARTESIAN_SIZE], float[MAX_ACTUATORS]))checkApiInit(usbLib, "GetCommandVelocity");

    // %EndTag(general function)%


    // %Tag(joint angular)%

    setAngularControl = (int (*)())checkApiInit(usbLib, "SetAngularControl");

    getAngularCommand = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularCommand");

    getAngularPosition = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularPosition");

    getAngularVelocity = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularVelocity");

    getAngularAcceleration = (int (*)(AngularAcceleration &))checkApiInit(usbLib, "GetActuatorAcceleration");


    getAngularForce = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularForce");

    setAngularTorqueMinMax = (int (*)(AngularInfo, AngularInfo))checkApiInit(usbLib, "SetAngularTorqueMinMax");


    getAngularCurrent = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularCurrent");

    getAngularCurrentMotor = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularCurrentMotor");

    getPositionCurrentActuators = (int (*)(float[POSITION_CURRENT_COUNT]))checkApiInit(usbLib, "GetPositionCurrentActuators");

    setActuatorPID = (int (*)(unsigned int, float, float, float))checkApiInit(usbLib, "SetActuatorPID");

    // %EndTag(joint angular)%


    // %Tag(tool cartesian)%

    setCartesianControl = (int (*)())checkApiInit(usbLib, "SetCartesianControl");

    getCartesianCommand = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianCommand");

    getCartesianPosition = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianPosition");


    getCartesianForce = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianForce");

    setCartesianForceMinMax = (int (*)(CartesianInfo, CartesianInfo))checkApiInit(usbLib, "SetCartesianForceMinMax");

    setCartesianInertiaDamping = (int (*)(CartesianInfo, CartesianInfo))checkApiInit(usbLib, "SetCartesianInertiaDamping");


    setEndEffectorOffset = (int (*)(unsigned int, float, float, float))checkApiInit(usbLib, "SetEndEffectorOffset");

    getEndEffectorOffset = (int (*)(unsigned int&, float&, float&, float&))checkApiInit(usbLib, "GetEndEffectorOffset");


    getActualTrajectoryInfo = (int (*)(TrajectoryPoint &))checkApiInit(usbLib, "GetActualTrajectoryInfo");

    getGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &))checkApiInit(usbLib, "GetGlobalTrajectoryInfo");

    sendAdvanceTrajectory = (int (*)(TrajectoryPoint))checkApiInit(usbLib, "SendAdvanceTrajectory");

    sendBasicTrajectory = (int (*)(TrajectoryPoint))checkApiInit(usbLib, "SendBasicTrajectory");

    eraseAllTrajectories = (int (*)())checkApiInit(usbLib, "EraseAllTrajectories");

     eraseAllProtectionZones = (int (*)())checkApiInit(usbLib, "EraseAllProtectionZones");

    getProtectionZone = (int (*)(ZoneList &))checkApiInit(usbLib, "GetProtectionZone");

    setProtectionZone = (int (*)(ZoneList))checkApiInit(usbLib, "SetProtectionZone");

    // %EndTag(tool cartesian)%


    // %Tag(pre-defined)%

    moveHome = (int (*)())checkApiInit(usbLib, "MoveHome");

    initFingers = (int (*)())checkApiInit(usbLib, "InitFingers");

    // %EndTag(pre-defined)%




    // The following APIs are not wrapped in kinova_comm, users should call kinova_api with extra caution.


    // %Tag(less interest in ROS)%

    setControlMapping = (int (*)(ControlMappingCharts)) checkApiInit(usbLib, "SetControlMapping");

    getControlMapping = (int (*)(ControlMappingCharts &)) checkApiInit(usbLib, "GetControlMapping");

    // %EndTag(less interest in ROS)%



    // %Tag(not function, place holder, etc)%

    getSingularityVector = (int (*)(SingularityVector &))checkApiInit(usbLib, "SendCartesianForceCommand"); // old style, not constantly set to zeros

    setActuatorMaxVelocity = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetActuatorMaxVelocity"); // no corresponding DSP code.


    // %EndTag(not function, place holder, etc)%





    // %Tag(experimental)%
    // force/torque control are not tested in ROS and may lead to unexpect motion.
    // do not use these functions unless you are know exactly what you are doing.

    setTorqueControlType = (int (*)(TORQUECONTROL_TYPE))checkApiInit(usbLib, "SetTorqueControlType");


    setActuatorPIDFilter = (int (*)(int, float, float, float))checkApiInit(usbLib, "SetActuatorPIDFilter");

    setAngularInertiaDamping = (int (*)(AngularInfo, AngularInfo))checkApiInit(usbLib, "SetAngularInertiaDamping");

    getAngularForceGravityFree = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularForceGravityFree");

    sendAngularTorqueCommand = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SendAngularTorqueCommand");

    setTorqueActuatorGain = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueActuatorGain");

    setTorqueActuatorDamping = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueActuatorDamping");

    setTorqueCommandMax = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueCommandMax");

    setTorqueSafetyFactor = (int (*)(float))checkApiInit(usbLib, "SetTorqueSafetyFactor");

    setTorqueRateLimiter = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueRateLimiter");

    setTorqueFeedCurrent = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueFeedCurrent");

    setTorqueFeedVelocity = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueFeedVelocity");

    setTorquePositionLimitDampingGain = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorquePositionLimitDampingGain");

    setTorquePositionLimitDampingMax = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorquePositionLimitDampingMax");

    setTorquePositionLimitRepulsGain = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorquePositionLimitRepulsGain");

    setTorquePositionLimitRepulsMax = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorquePositionLimitRepulsMax");

    setTorqueFilterVelocity = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueFilterVelocity");


    setTorqueFilterMeasuredTorque = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueFilterMeasuredTorque");

    setTorqueFilterError = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueFilterError");

    setTorqueFilterControlEffort = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetTorqueFilterControlEffort");


    sendCartesianForceCommand = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SendCartesianForceCommand");

    switchTrajectoryTorque = (int (*)(GENERALCONTROL_TYPE))checkApiInit(usbLib, "SwitchTrajectoryTorque");

    setGravityType = (int (*)(GRAVITY_TYPE))checkApiInit(usbLib, "SetGravityType");

    setGravityVector = (int (*)(float[GRAVITY_VECTOR_SIZE]))checkApiInit(usbLib, "SetGravityVector");

    setGravityOptimalZParam = (int (*)(float[GRAVITY_PARAM_SIZE]))checkApiInit(usbLib, "SetGravityOptimalZParam");

    setGravityManualInputParam = (int (*)(float[GRAVITY_PARAM_SIZE]))checkApiInit(usbLib, "SetGravityManualInputParam");

    getAngularTorqueCommand = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "GetAngularTorqueCommand");

    getAngularTorqueGravityEstimation = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "GetAngularTorqueGravityEstimation");

    setSwitchThreshold = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetSwitchThreshold");

    setPositionLimitDistance = (int (*)(float[COMMAND_SIZE]))checkApiInit(usbLib, "SetPositionLimitDistance");

    setGravityPayload = (int (*)(float[GRAVITY_PAYLOAD_SIZE]))checkApiInit(usbLib, "SetGravityPayload");

    setTorqueVibrationController = (int (*)(float))checkApiInit(usbLib, "SetTorqueVibrationController");

    setTorqueRobotProtection = (int (*)(int))checkApiInit(usbLib, "SetTorqueRobotProtection");

    runGravityZEstimationSequence = (int (*)(ROBOT_TYPE, double[OPTIMAL_Z_PARAM_SIZE]))checkApiInit(usbLib, "RunGravityZEstimationSequence");

    getTrajectoryTorqueMode = (int (*)(int &))checkApiInit(usbLib, "GetTrajectoryTorqueMode");

    setTorqueInactivityType = (int (*)(int))checkApiInit(usbLib, "SetTorqueInactivityType");


        // %EndTag(experimental)%
}

}  // namespace kinova

