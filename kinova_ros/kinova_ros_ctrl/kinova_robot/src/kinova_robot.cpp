#include <kinova_robot/kinova_robot.hpp>
#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.UsbCommandLayerUbuntu.h>

using namespace kinova_robot;

namespace {
    static const double PI = 3.14159;

    /// \brief Convert degrees to radians.
    double deg2rad(double d)
    {
        return d * PI / 180.0;
    }

    /// \brief Convert radians to degrees.
    double rad2deg(double r)
    {
        return r / PI * 180.0;
    }
    /// \brief API degrees into 'regular' degrees.
    ///
    /// The Kinova API returns angle velocities (and torque, apparently) in
    /// the 0..180 range for positive values, and the 360..181 range for
    /// negative ones.
    double kindeg(double d)
    {
        if (d > 180.0) {
            return d - 360.0;
        } else {
            return d;
        }
    }
}

KinovaRobot::KinovaRobot(const std::string& serial)
{
    if (InitAPI() != NO_ERROR_KINOVA) {
        throw KinovaException("Could not initialize API.");
    }

    // Enumerate devices, find the one who corresponds to the given serial
    // number.
    typedef std::vector<KinovaDevice> KinovaDevices;
    KinovaDevices devices;
    int r;
    GetDevices(devices, r);
    if (r != NO_ERROR_KINOVA) {
        throw KinovaException("Could not get the devices list.");
    }
    bool found = false;
    typedef KinovaDevices::const_iterator KDIt;
    for (KDIt i = devices.begin(); i != devices.end(); ++i) {
        const KinovaDevice& d = *i;
        if ((serial == "") ||
            (serial == d.SerialNumber)) {

            // Found the device.
            QuickStatus qs;
            GetQuickStatus(qs);
            switch (qs.RobotType) {
                case 0: // JACO r1
                    robot_name_  = "JACO";
                    num_joints_  = 9;
                    num_fingers_ = 3;
                    break;
                case 1: // MICO
                    robot_name_  = "MICO";
                    num_joints_  = 8;
                    num_fingers_ = 2;
                    break;
                case 3: // JACO r2
                    robot_name_  = "JACO2";
                    num_joints_  = 9;
                    num_fingers_ = 3;
                    break;
                default:
                    throw KinovaException("Unknown device type");
                    break;
            };

            state_ = RobotState(numJoints());
            cmd_   = StateVector(numJoints(), 0.0);

            found = true;
            break;
        }
    }

    if (!found) {
        throw KinovaException("Could not find the specified device");
    }

    InitFingers();

}

KinovaRobot::~KinovaRobot()
{
    CloseAPI();
}

void KinovaRobot::updateState()
{
    AngularPosition ap_p;
    AngularPosition ap_v;
    AngularPosition ap_t;

    if (GetAngularPosition(ap_p) != NO_ERROR_KINOVA) {
        throw KinovaException("Could not get joint angles.");
    }
    if (GetAngularVelocity(ap_v) != NO_ERROR_KINOVA) {
        throw KinovaException("Could not get joint velocities.");
    }
    if (GetAngularForce(ap_t) != NO_ERROR_KINOVA) {
        throw KinovaException("Could not get joint torques.");
    }

    AngularInfo& q  = ap_p.Actuators;
    AngularInfo& qd = ap_v.Actuators;
    AngularInfo& tq = ap_t.Actuators;

    // See Jaco/Mico DH configuration for details on angle conversion.
    state_.position[0] = deg2rad(180.0 - q.Actuator1);
    state_.position[1] = deg2rad(q.Actuator2 - 270.0);
    state_.position[2] = deg2rad( 90.0 - q.Actuator3);
    state_.position[3] = deg2rad(180.0 - q.Actuator4);
    state_.position[4] = deg2rad(180.0 - q.Actuator5);
    state_.position[5] = deg2rad(270.0 - q.Actuator6);

    // TODO: The need for 'kindeg' might be related to an API issue.
    //       Add a test for this.
    state_.velocity[0] = deg2rad(kindeg(qd.Actuator1));
    state_.velocity[1] = deg2rad(kindeg(qd.Actuator2));
    state_.velocity[2] = deg2rad(kindeg(qd.Actuator3));
    state_.velocity[3] = deg2rad(kindeg(qd.Actuator4));
    state_.velocity[4] = deg2rad(kindeg(qd.Actuator5));
    state_.velocity[5] = deg2rad(kindeg(qd.Actuator6));

    state_.torque[0] = qd.Actuator1;
    state_.torque[1] = qd.Actuator2;
    state_.torque[2] = qd.Actuator3;
    state_.torque[3] = qd.Actuator4;
    state_.torque[4] = qd.Actuator5;
    state_.torque[5] = qd.Actuator6;

    // TODO: Full (or at least) partial finger state:
    for (int i = 6; i < numJoints(); ++i) {
        state_.position[i] = 0.0;
        state_.velocity[i] = 0.0;
        state_.torque[i]   = 0.0;
    }

}

void KinovaRobot::setPosition(const StateVector& cmd)
{
    if (cmd.size() != numJoints()) {
        throw KinovaException("Wrong number of joints in command vector.");  
    }

    // TODO: Angle saturation?
    cmd_ = cmd;
}

void KinovaRobot::sendCommand()
{
    TrajectoryPoint point;
    point.InitStruct();

    point.Position.Delay               = 0.0;
    point.Position.Type                = ANGULAR_POSITION;

    point.Position.Actuators.Actuator1 = 180.0 - rad2deg(cmd_[0]);
    point.Position.Actuators.Actuator2 = rad2deg(cmd_[1]) - 270.0;
    point.Position.Actuators.Actuator3 =  90.0 - rad2deg(cmd_[2]);
    point.Position.Actuators.Actuator4 = 180.0 - rad2deg(cmd_[3]);
    point.Position.Actuators.Actuator5 = 180.0 - rad2deg(cmd_[4]);
    point.Position.Actuators.Actuator6 = 270.0 - rad2deg(cmd_[5]);

    point.Position.Fingers.Finger1     = cmd_[6];
    point.Position.Fingers.Finger2     = cmd_[7];
    if (numFingers() >= 3) {
        point.Position.Fingers.Finger3 = cmd_[8];
    }

    SetAngularControl(); // TODO: Make sure this is necessary.
    if (SendAdvanceTrajectory(point) != NO_ERROR_KINOVA) {
        throw KinovaException("Could not send command point.");
    }

}
