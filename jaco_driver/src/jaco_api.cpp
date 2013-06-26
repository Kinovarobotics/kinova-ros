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

	this->InitAPI = (int (*)())dlsym(USBLib, "InitAPI");

this	->CloseAPI = (int (*)())dlsym(USBLib, "CloseAPI");

this	->GetCodeVersion = (int (*)(
			std::vector<int> &))dlsym(USBLib, "GetCodeVersion");

this	->GetCartesianPosition = (int (*)(
			CartesianPosition &))dlsym(USBLib, "GetCartesianPosition");

this	->GetAngularPosition = (int (*)(
			AngularPosition &))dlsym(USBLib, "GetAngularPosition");

this	->GetCartesianForce = (int (*)(
			CartesianPosition &))dlsym(USBLib, "GetCartesianForce");

this	->GetAngularForce = (int (*)(
			AngularPosition &))dlsym(USBLib, "GetAngularForce");

this	->GetAngularCurrent = (int (*)(
			AngularPosition &))dlsym(USBLib, "GetAngularCurrent");

this	->GetControlOperator = (int (*)(int &))dlsym(USBLib, "GetControlOperator");

this	->GetActualTrajectoryInfo = (int (*)(
			TrajectoryPoint &))dlsym(USBLib, "GetActualTrajectoryInfo");

this	->GetGlobalTrajectoryInfo = (int (*)(
			TrajectoryFIFO &))dlsym(USBLib, "GetGlobalTrajectoryInfo");

this	->GetSensorsInfo = (int (*)(SensorsInfo &))dlsym(USBLib, "GetSensorsInfo");

this	->GetSingularityVector = (int (*)(
			SingularityVector &))dlsym(USBLib, "GetSingularityVector");

this	->SetAngularControl = (int (*)())dlsym(USBLib, "SetAngularControl");

this	->SetCartesianControl = (int (*)())dlsym(USBLib, "SetCartesianControl");

this	->StartControlAPI = (int (*)())dlsym(USBLib, "StartControlAPI");

this	->StopControlAPI = (int (*)())dlsym(USBLib, "StopControlAPI");

this	->RestoreFactoryDefault = (int (*)())dlsym(USBLib, "RestoreFactoryDefault");

this	->SendJoystickCommand = (int (*)(
			JoystickCommand))dlsym(USBLib, "SendJoystickCommand");

this	->SendAdvanceTrajectory = (int (*)(
			TrajectoryPoint))dlsym(USBLib, "SendAdvanceTrajectory");

this	->SendBasicTrajectory = (int (*)(
			TrajectoryPoint))dlsym(USBLib, "SendBasicTrajectory");

this	->GetClientConfigurations = (int (*)(
			ClientConfigurations &))dlsym(USBLib, "GetClientConfigurations");

this	->SetClientConfigurations = (int (*)(
			ClientConfigurations))dlsym(USBLib, "SetClientConfigurations");

this	->EraseAllTrajectories = (int (*)())dlsym(USBLib, "EraseAllTrajectories");

this	->GetPositionCurrentActuators = (int (*)(
			std::vector<float> &))dlsym(USBLib, "GetPositionCurrentActuators");

this	->SetActuatorPID = (int (*)(unsigned int, float, float,
			float))dlsym(USBLib, "SetActuatorPID");}}
