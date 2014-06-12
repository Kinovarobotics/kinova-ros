/*
 * jacolib.cpp
 *
 *  Created on: Mar 10, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#include <jaco_driver/jaco_api.h>

namespace jaco {

void* checkApiInit(void * usbLib, const char* name) {
    void * function_pointer = dlsym(usbLib, name);
    assert(function_pointer != NULL);
    return function_pointer;
}

JacoAPI::JacoAPI(void) {
    void *USBLib = dlopen(JACO_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    if (USBLib == NULL) {
        ROS_WARN("%s", dlerror());
    }

    initAPI = (int (*)())checkApiInit(USBLib, "InitAPI");


    closeAPI = (int (*)())checkApiInit(USBLib, "CloseAPI");

    getCodeVersion = (int (*)(
            std::vector<int> &))checkApiInit(USBLib, "GetCodeVersion");

    getCartesianPosition = (int (*)(
            CartesianPosition &))checkApiInit(USBLib, "GetCartesianPosition");

    getAngularPosition = (int (*)(
            AngularPosition &))checkApiInit(USBLib, "GetAngularPosition");

    getCartesianForce = (int (*)(
            CartesianPosition &))checkApiInit(USBLib, "GetCartesianForce");

    getAngularForce = (int (*)(
            AngularPosition &))checkApiInit(USBLib, "GetAngularForce");

    getAngularCurrent = (int (*)(
            AngularPosition &))checkApiInit(USBLib, "GetAngularCurrent");

    //getControlOperator = (int (*)(int &))checkApiInit(USBLib, "GetControlOperator");

    getActualTrajectoryInfo = (int (*)(
            TrajectoryPoint &))checkApiInit(USBLib, "GetActualTrajectoryInfo");

    getGlobalTrajectoryInfo = (int (*)(
            TrajectoryFIFO &))checkApiInit(USBLib, "GetGlobalTrajectoryInfo");

    getSensorsInfo = (int (*)(SensorsInfo &))checkApiInit(USBLib, "GetSensorsInfo");

    getSingularityVector = (int (*)(
            SingularityVector &))checkApiInit(USBLib, "GetSingularityVector");

    setAngularControl = (int (*)())checkApiInit(USBLib, "SetAngularControl");

    setCartesianControl = (int (*)())checkApiInit(USBLib, "SetCartesianControl");

    startControlAPI = (int (*)())checkApiInit(USBLib, "StartControlAPI");

    stopControlAPI = (int (*)())checkApiInit(USBLib, "StopControlAPI");

    moveHome = (int (*)())checkApiInit(USBLib, "MoveHome");

    initFingers = (int (*)())checkApiInit(USBLib, "InitFingers");

    restoreFactoryDefault = (int (*)())checkApiInit(USBLib, "RestoreFactoryDefault");

    sendJoystickCommand = (int (*)(
            JoystickCommand))checkApiInit(USBLib, "SendJoystickCommand");

    sendAdvanceTrajectory = (int (*)(
            TrajectoryPoint))checkApiInit(USBLib, "SendAdvanceTrajectory");

    sendBasicTrajectory = (int (*)(
            TrajectoryPoint))checkApiInit(USBLib, "SendBasicTrajectory");

    getQuickStatus = (int (*)(
            QuickStatus &))checkApiInit(USBLib, "GetQuickStatus");

    getClientConfigurations = (int (*)(
            ClientConfigurations &))checkApiInit(USBLib, "GetClientConfigurations");

    setClientConfigurations = (int (*)(
            ClientConfigurations))checkApiInit(USBLib, "SetClientConfigurations");

    eraseAllTrajectories = (int (*)())checkApiInit(USBLib, "EraseAllTrajectories");

    getPositionCurrentActuators = (int (*)(
            std::vector<float> &))checkApiInit(USBLib, "GetPositionCurrentActuators");

    setActuatorPID = (int (*)(unsigned int, float, float,
            float))checkApiInit(USBLib, "SetActuatorPID");
}

}  // namespace jaco
