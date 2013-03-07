/*
 * jacolib.h
 *
 *  Created on: Feb 16, 2013
 *      Author: mdedonato
 */

#ifndef JACOLIB_H_
#define JACOLIB_H_

#include "jaco_driver/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_driver/KinovaTypes.h"
#include <dlfcn.h>
#include <iostream>
#include <ros/ros.h>

extern int (*Jaco_InitAPI)(void);
extern int (*Jaco_CloseAPI)(void);
extern int (*Jaco_GetCodeVersion)(std::vector<int> &);
extern int (*Jaco_GetCartesianPosition)(CartesianPosition &);
extern int (*Jaco_GetAngularPosition)(AngularPosition &);
extern int (*Jaco_GetCartesianForce)(CartesianPosition &);
extern int (*Jaco_GetAngularForce)(AngularPosition &);
extern int (*Jaco_GetAngularCurrent)(AngularPosition &);
extern int (*Jaco_GetControlOperator)(int &);
extern int (*Jaco_GetActualTrajectoryInfo)(TrajectoryPoint &);
extern int (*Jaco_GetGlobalTrajectoryInfo)(TrajectoryFIFO &);
extern int (*Jaco_GetSensorsInfo)(SensorsInfo &);
extern int (*Jaco_GetSingularityVector)(SingularityVector &);
extern int (*Jaco_SetAngularControl)();
extern int (*Jaco_SetCartesianControl)();
extern int (*Jaco_StartControlAPI)();
extern int (*Jaco_StopControlAPI)();
extern int (*Jaco_RestoreFactoryDefault)();
extern int (*Jaco_SendJoystickCommand)(JoystickCommand);
extern int (*Jaco_SendAdvanceTrajectory)(TrajectoryPoint);
extern int (*Jaco_SendBasicTrajectory)(TrajectoryPoint);
extern int (*Jaco_GetClientConfigurations)(ClientConfigurations &);
extern int (*Jaco_SetClientConfigurations)(ClientConfigurations);
extern int (*Jaco_EraseAllTrajectories)();
extern int (*Jaco_GetPositionCurrentActuators)(std::vector<float> &);
extern int (*Jaco_SetActuatorPID)(unsigned int, float, float, float);




//}
void Jacolib_Init(void);

#endif /* JACOLIB_H_ */
