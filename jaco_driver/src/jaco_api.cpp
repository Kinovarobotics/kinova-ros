/*
 * jacolib.cpp
 *
 *  Created on: Mar 10, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#include <jaco_driver/jaco_api.h>

namespace jaco {

JacoAPI::JacoAPI(void) {

    void *USBLib = dlopen(JACO_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    if (USBLib == NULL) {
        ROS_WARN("%s", dlerror());
    }

    initAPI = (int (*)())dlsym(USBLib, "InitAPI");

    closeAPI = (int (*)())dlsym(USBLib, "CloseAPI");

    getCodeVersion = (int (*)(
            std::vector<int> &))dlsym(USBLib, "GetCodeVersion");

    getCartesianPosition = (int (*)(
            CartesianPosition &))dlsym(USBLib, "GetCartesianPosition");

    getAngularPosition = (int (*)(
            AngularPosition &))dlsym(USBLib, "GetAngularPosition");

    getCartesianForce = (int (*)(
            CartesianPosition &))dlsym(USBLib, "GetCartesianForce");

    getAngularForce = (int (*)(
            AngularPosition &))dlsym(USBLib, "GetAngularForce");

    getAngularCurrent = (int (*)(
            AngularPosition &))dlsym(USBLib, "GetAngularCurrent");

    getControlOperator = (int (*)(int &))dlsym(USBLib, "GetControlOperator");

    getActualTrajectoryInfo = (int (*)(
            TrajectoryPoint &))dlsym(USBLib, "GetActualTrajectoryInfo");

    getGlobalTrajectoryInfo = (int (*)(
            TrajectoryFIFO &))dlsym(USBLib, "GetGlobalTrajectoryInfo");

    getSensorsInfo = (int (*)(SensorsInfo &))dlsym(USBLib, "GetSensorsInfo");

    getSingularityVector = (int (*)(
            SingularityVector &))dlsym(USBLib, "GetSingularityVector");

    setAngularControl = (int (*)())dlsym(USBLib, "SetAngularControl");

    setCartesianControl = (int (*)())dlsym(USBLib, "SetCartesianControl");

    startControlAPI = (int (*)())dlsym(USBLib, "StartControlAPI");

    stopControlAPI = (int (*)())dlsym(USBLib, "StopControlAPI");

    moveHome = (int (*)())dlsym(USBLib, "MoveHome");

    initFingers = (int (*)())dlsym(USBLib, "InitFingers");

    restoreFactoryDefault = (int (*)())dlsym(USBLib, "RestoreFactoryDefault");

    sendJoystickCommand = (int (*)(
            JoystickCommand))dlsym(USBLib, "SendJoystickCommand");

    sendAdvanceTrajectory = (int (*)(
            TrajectoryPoint))dlsym(USBLib, "SendAdvanceTrajectory");

    sendBasicTrajectory = (int (*)(
            TrajectoryPoint))dlsym(USBLib, "SendBasicTrajectory");

    getQuickStatus = (int (*)(
            QuickStatus &))dlsym(USBLib, "GetQuickStatus");

    getClientConfigurations = (int (*)(
            ClientConfigurations &))dlsym(USBLib, "GetClientConfigurations");

    setClientConfigurations = (int (*)(
            ClientConfigurations))dlsym(USBLib, "SetClientConfigurations");

    eraseAllTrajectories = (int (*)())dlsym(USBLib, "EraseAllTrajectories");

    getPositionCurrentActuators = (int (*)(
            std::vector<float> &))dlsym(USBLib, "GetPositionCurrentActuators");

    setActuatorPID = (int (*)(unsigned int, float, float,
            float))dlsym(USBLib, "SetActuatorPID");
}

}
