


#include "kinova/Kinova.API.UsbCommandLayerUbuntu.h"
#include "kinova/KinovaTypes.h"

#include "vrep_jaco_sim/JacoVREPInterface.h"

static JacoVREPInterface * interface = NULL;
static TrajectoryPoint lastTrajectoryPoint;

extern "C" {
    int InitAPI(void) {
        if (!interface) {
            interface = new JacoVREPInterface();
            lastTrajectoryPoint.InitStruct();
        }
        return NO_ERROR_KINOVA;
    }

    int CloseAPI(void) {
        if (interface) {
            delete interface;
            interface = NULL;
        }
        return NO_ERROR_KINOVA;
    }

    int GetAPIVersion(std::vector<int> & api) {
        api.resize(3);
        api[0] = 1; api[1] = 0; api[2] = 0;
        return NO_ERROR_KINOVA;
    }
    int GetDevices(std::vector<KinovaDevice> & dev, int &result) {
        result = result = NO_ERROR_KINOVA;
        dev.resize(1);
        strcpy(dev[0].SerialNumber,"VREP_SIMULATOR");
        dev[0].DeviceID = 0;
        return NO_ERROR_KINOVA;
    }
    int SetActiveDevice(KinovaDevice) {
        return NO_ERROR_KINOVA;
    }

    int GetGeneralInformations(GeneralInformations & info) {
        // No other bits used in the code
        info.CodeVersion = 1;
        info.CodeRevision = 0;
        return NO_ERROR_KINOVA;
    }

    int GetQuickStatus(QuickStatus & status) {
        status.Finger1Status = 1;
        status.Finger2Status = 1;
        status.Finger3Status = 1;
        status.RetractType = RETRACT_TYPE_NORMAL_TO_READY;
        status.RetractComplexity = 0;
        status.ControlEnableStatus = 0;
        status.ControlActiveModule = CONTROL_MODULE_ANGULAR_VELOCITY;
        status.ControlFrameType = 0;
        status.CartesianFaultState = 0;
        status.ForceControlStatus = 1;
        status.CurrentLimitationStatus = 1;
        status.RobotType = 0;
        status.RobotEdition = 0;
        status.TorqueSensorsStatus = 0;

        return NO_ERROR_KINOVA;
    }


    int GetCodeVersion(std::vector<int> & version) {
        version.resize(3);
        version[0] = 1; version[1] = 0; version[2] = 0;
        return NO_ERROR_KINOVA;
    }

    int StartControlAPI() {
        return NO_ERROR_KINOVA;
    }

    int StopControlAPI() {
        return NO_ERROR_KINOVA;
    }

    int InitFingers() {
        return NO_ERROR_KINOVA;
    }


    int MoveHome() {
        return NO_ERROR_KINOVA;
    }


    int GetCartesianPosition(CartesianPosition &) {
        return NO_ERROR_KINOVA;
    }

    int GetAngularPosition(AngularPosition & ap) {
        if (!interface) {
            return ERROR_NOT_INITIALIZED;
        }
        ap.Actuators.InitStruct();
        ap.Fingers.InitStruct();
        interface->getPositions( ap.Actuators.Actuator1, ap.Actuators.Actuator2,
                ap.Actuators.Actuator3, ap.Actuators.Actuator4,
                ap.Actuators.Actuator5, ap.Actuators.Actuator6);
        return NO_ERROR_KINOVA;
    }

    int GetAngularVelocity(AngularPosition & ap) {
        if (!interface) {
            return ERROR_NOT_INITIALIZED;
        }
        ap.Actuators.InitStruct();
        ap.Fingers.InitStruct();
        interface->getVelocities( ap.Actuators.Actuator1, ap.Actuators.Actuator2,
                ap.Actuators.Actuator3, ap.Actuators.Actuator4,
                ap.Actuators.Actuator5, ap.Actuators.Actuator6);
        return NO_ERROR_KINOVA;
    }


    int GetCartesianForce(CartesianPosition & cp) {
        cp.InitStruct();
        return NO_ERROR_KINOVA;
    }

    int SetCartesianForceMinMax(CartesianInfo, CartesianInfo) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int SetCartesianInertiaDamping(CartesianInfo, CartesianInfo) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int StartForceControl() {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int StopForceControl() {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int GetAngularForce(AngularPosition &) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int GetAngularCurrent(AngularPosition &) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int GetControlType(int & ct) {
        ct = CONTROL_TYPE_ANGULAR;
        return NO_ERROR_KINOVA;
    }

    int GetActualTrajectoryInfo(TrajectoryPoint & tp) {
        tp = lastTrajectoryPoint;
        return NO_ERROR_KINOVA;
    }

    int GetGlobalTrajectoryInfo(TrajectoryFIFO &) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int GetSensorsInfo(SensorsInfo & si) {
        si.InitStruct();
        return NO_ERROR_KINOVA;
    }

    int SetAngularControl() {
        return NO_ERROR_KINOVA;
    }

    int SetCartesianControl() {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int RestoreFactoryDefault() {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int SendJoystickCommand(JoystickCommand) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int SendBasicTrajectory(TrajectoryPoint tp) {
        if (!interface) {
            return ERROR_NOT_INITIALIZED;
        }
        AngularInfo & ai = tp.Position.Actuators;
        CartesianInfo & ci = tp.Position.CartesianPosition;
        switch (tp.Position.Type) {
            case ANGULAR_POSITION:
                interface->setPositionControl();
                interface->publishControl(ai.Actuator1,ai.Actuator2,
                        ai.Actuator3,ai.Actuator4,
                        ai.Actuator5,ai.Actuator6);
                break;
            case ANGULAR_VELOCITY:
                interface->setVelocityControl();
                interface->publishControl(ai.Actuator1,ai.Actuator2,
                        ai.Actuator3,ai.Actuator4,
                        ai.Actuator5,ai.Actuator6);
                break;
            case CARTESIAN_VELOCITY:
                if ((fabs(ci.X)+fabs(ci.Y)+fabs(ci.Z)) > 1e-2)
                    return ERROR_LIBUSB_NOT_SUPPORTED;
                if ((fabs(ci.ThetaX)+fabs(ci.ThetaY)+fabs(ci.ThetaZ)) > 1e-2)
                    return ERROR_LIBUSB_NOT_SUPPORTED;
                return NO_ERROR_KINOVA;
            default:
                return ERROR_LIBUSB_NOT_SUPPORTED;
        }
        return NO_ERROR_KINOVA;
    }

    int SendAdvanceTrajectory(TrajectoryPoint tp) {
        return SendBasicTrajectory(tp);
    }

    int GetClientConfigurations(ClientConfigurations & cfg) {
        strcpy(cfg.ClientID,"DEADBEEF");
        strcpy(cfg.ClientName,"BLAH BLAH");
        strcpy(cfg.Organization,"HI HI");
        strcpy(cfg.Serial,"VREP_SIMULATOR");
        strcpy(cfg.Model,"JACO 2");
        return NO_ERROR_KINOVA;
    }

    int SetClientConfigurations(ClientConfigurations) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int EraseAllTrajectories() {
        lastTrajectoryPoint.InitStruct();
        return NO_ERROR_KINOVA;
    }

    int GetPositionCurrentActuators(std::vector<float> &) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }

    int SetActuatorPID(unsigned int, float, float, float) {
        return ERROR_LIBUSB_NOT_SUPPORTED;
    }


};



