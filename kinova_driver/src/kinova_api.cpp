/*
 * jacolib.cpp
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


JacoAPI::JacoAPI(void)
{
    void *usbLib  = dlopen(KINOVA_USB_LIBRARY,  RTLD_NOW | RTLD_GLOBAL);
    void *commLib = dlopen(KINOVA_COMM_LIBRARY, RTLD_NOW | RTLD_GLOBAL);

    if ((usbLib == NULL) || (commLib == NULL))
    {
        ROS_FATAL("%s", dlerror());
    }

    initAPI = (int (*)())checkApiInit(usbLib, "InitAPI");

    closeAPI = (int (*)())checkApiInit(usbLib, "CloseAPI");

    getAPIVersion = (int (*)(int[API_VERSION_COUNT]))checkApiInit(usbLib, "GetAPIVersion");

    getDevices = (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int &))checkApiInit(usbLib, "GetDevices");

    getDeviceCount = (int (*)(int &))checkApiInit(commLib, "GetDeviceCount");

    setActiveDevice = (int (*)(KinovaDevice))checkApiInit(usbLib, "SetActiveDevice");

    getCodeVersion = (int (*)(int[CODE_VERSION_COUNT]))checkApiInit(usbLib, "GetCodeVersion");

    getGeneralInformations = (int (*)(GeneralInformations &))checkApiInit(usbLib, "GetGeneralInformations");

    getCartesianPosition = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianPosition");

    getAngularPosition = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularPosition");

    getAngularVelocity = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularVelocity");

    getCartesianForce = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianForce");

    setCartesianForceMinMax = (int (*)(CartesianInfo, CartesianInfo))checkApiInit(usbLib, "SetCartesianForceMinMax");

    setCartesianInertiaDamping = (int (*)(CartesianInfo, CartesianInfo))checkApiInit(usbLib, "SetCartesianInertiaDamping");

    startForceControl = (int (*)())checkApiInit(usbLib, "StartForceControl");

    stopForceControl = (int (*)())checkApiInit(usbLib, "StopForceControl");

    getAngularForce = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularForce");

    getAngularCurrent = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularCurrent");

    getActualTrajectoryInfo = (int (*)(TrajectoryPoint &))checkApiInit(usbLib, "GetActualTrajectoryInfo");

    getGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &))checkApiInit(usbLib, "GetGlobalTrajectoryInfo");

    getSensorsInfo = (int (*)(SensorsInfo &))checkApiInit(usbLib, "GetSensorsInfo");

    setAngularControl = (int (*)())checkApiInit(usbLib, "SetAngularControl");

    setCartesianControl = (int (*)())checkApiInit(usbLib, "SetCartesianControl");

    startControlAPI = (int (*)())checkApiInit(usbLib, "StartControlAPI");

    stopControlAPI = (int (*)())checkApiInit(usbLib, "StopControlAPI");

    moveHome = (int (*)())checkApiInit(usbLib, "MoveHome");

    initFingers = (int (*)())checkApiInit(usbLib, "InitFingers");

    restoreFactoryDefault = (int (*)())checkApiInit(usbLib, "RestoreFactoryDefault");

    sendJoystickCommand = (int (*)(JoystickCommand))checkApiInit(usbLib, "SendJoystickCommand");

    sendAdvanceTrajectory = (int (*)(TrajectoryPoint))checkApiInit(usbLib, "SendAdvanceTrajectory");

    sendBasicTrajectory = (int (*)(TrajectoryPoint))checkApiInit(usbLib, "SendBasicTrajectory");

    getControlType = (int (*)(int &)) checkApiInit(usbLib, "GetControlType");

    getQuickStatus = (int (*)(QuickStatus &))checkApiInit(usbLib, "GetQuickStatus");

    getClientConfigurations = (int (*)(ClientConfigurations &))checkApiInit(usbLib, "GetClientConfigurations");

    setClientConfigurations = (int (*)( ClientConfigurations))checkApiInit(usbLib, "SetClientConfigurations");

    eraseAllTrajectories = (int (*)())checkApiInit(usbLib, "EraseAllTrajectories");

    getPositionCurrentActuators = (int (*)(float[POSITION_CURRENT_COUNT]))checkApiInit(usbLib, "GetPositionCurrentActuators");

    setActuatorPID = (int (*)(unsigned int, float, float, float))checkApiInit(usbLib, "SetActuatorPID");

    setEndEffectorOffset = (int (*)(unsigned int, float, float, float))checkApiInit(usbLib, "SetEndEffectorOffset");
    getEndEffectorOffset = (int (*)(unsigned int&, float&, float&, float&))checkApiInit(usbLib, "GetEndEffectorOffset");

}

}  // namespace kinova
