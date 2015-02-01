#include <kinova_robot/kinova_robot.hpp>
#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.UsbCommandLayerUbuntu.h>
#include <sstream>

using namespace kinova_robot;

namespace {
    static const double PI = 3.14159;

    /// \brief Clamp an angle (in radians) to -pi..pi.
    inline double clampRad(double r)
    {
        while (r < -PI) {
            r += 2 * PI;
        }
        while (r > PI) {
            r -= 2 * PI;
        }
        return r;
    }

    /// \brief Clamp an angle (in radians) to -2pi..2pi.
    inline double clampRd2(double r)
    {
        while (r < -2*PI) {
            r += 2 * PI;
        }
        while (r > 2*PI) {
            r -= 2 * PI;
        }
        return r;
    }

    /// \brief Clamp an angle (in degrees) to 0..360.
    inline double clampDeg(double d)
    {
        while (d < 0) {
            d += 360.0;
        }
        while (d > 360) {
            d -= 360.0;
        }

        return d;
    }

    /// \brief Convert degrees to radians.
    inline double deg2rad(double d)
    {
        return (d * PI / 180.0);
    }

    /// \brief Convert radians to degrees.
    inline double rad2deg(double r)
    {
        return r / PI * 180.0;
    }
    /// \brief API degrees into 'regular' degrees.
    ///
    /// The Kinova API returns angle velocities (and torque, apparently) in
    /// the 0..180 range for positive values, and the 360..181 range for
    /// negative ones.
    inline double kindeg(double d)
    {
#ifndef DISABLE_KINDEG
        if (d > 180.0) {
            return d - 360.0;
        } else {
            return d;
        }
#else
        return d;
#endif
    }

    void throwError(const std::string& base, int err)
    {
        std::stringstream ss;
        ss << base << ", error number: " << err;
        throw KinovaException(ss.str());
    }
}

KinovaRobot::KinovaRobot(const std::string& serial)
{
    int r = InitAPI();
    if (r != NO_ERROR_KINOVA) {
        throwError("Could not initialize API", r);
    }

    // Enumerate devices, find the one who corresponds to the given serial
    // number.
    typedef std::vector<KinovaDevice> KinovaDevices;
    KinovaDevices devices;
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
            setupRobot(qs.RobotType);

            state_    = RobotState(numJoints());
            cur_cmd_  = StateVector(numJoints(), 0.0);
            last_cmd_ = cur_cmd_; 

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

void KinovaRobot::setupRobot(int type)
{
    switch (type) {
        case 0: // JACO r1
            robot_name_      = "jaco";
            num_joints_      = 9;
            num_fingers_     = 3;
            j6_angle_offset_ = 260.0;
            break;
        case 1: // MICO
            robot_name_      = "mico";
            num_joints_      = 8;
            num_fingers_     = 2;
            j6_angle_offset_ = 270.0;
            break;
        case 3: // JACO r2
            robot_name_      = "jaco";
            num_joints_      = 9;
            num_fingers_     = 3;
            j6_angle_offset_ = 260.0; // TODO: confirm this.
            break;
            break;
        default:
            throw KinovaException("Unknown device type");
            break;
    };
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
    const double& j6o = j6_angle_offset_;
    state_.position[0] = clampRad(deg2rad(180.0 - q.Actuator1));
    state_.position[1] = clampRd2(deg2rad(q.Actuator2 - 270.0));
    state_.position[2] = clampRd2(deg2rad( 90.0 - q.Actuator3));
    state_.position[3] = clampRad(deg2rad(180.0 - q.Actuator4));
    state_.position[4] = clampRad(deg2rad(180.0 - q.Actuator5));
    state_.position[5] = clampRad(deg2rad(j6o   - q.Actuator6));

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
    cur_cmd_  = cmd;
}

void KinovaRobot::sendCommand()
{
    // Do not send trajectory points if the command did not change enough - this
    // seems to saturate the bus.
    if (!commandChanged()) {
        return;
    }

    TrajectoryPoint point;
    point.InitStruct();

    point.Position.Delay               = 0.0;
    point.Position.Type                = ANGULAR_POSITION;
    point.Position.HandMode            = POSITION_MODE;

    const double& j6o = j6_angle_offset_;
    point.Position.Actuators.Actuator1 =         (180.0 - rad2deg(cur_cmd_[0]));
    point.Position.Actuators.Actuator2 = clampDeg(270.0 + rad2deg(cur_cmd_[1]));
    point.Position.Actuators.Actuator3 = clampDeg( 90.0 - rad2deg(cur_cmd_[2]));
    point.Position.Actuators.Actuator4 =         (180.0 - rad2deg(cur_cmd_[3]));
    point.Position.Actuators.Actuator5 =         (180.0 - rad2deg(cur_cmd_[4]));
    point.Position.Actuators.Actuator6 =         (j6o   - rad2deg(cur_cmd_[5]));

    // TEST: partially closed fingers:
    point.Position.Fingers.Finger1     = 10.0; //cur_cmd_[6];
    point.Position.Fingers.Finger2     = 10.0; //cur_cmd_[7];
    if (numFingers() >= 3) {
        point.Position.Fingers.Finger3 = 10.0; //cur_cmd_[8];
    }

    EraseAllTrajectories();
    //SetAngularControl(); // TODO: Make sure this is necessary.
    if (SendBasicTrajectory(point) != NO_ERROR_KINOVA) {
        throw KinovaException("Could not send command point.");
    }

    // Save the last command actually sent:
    last_cmd_ = cur_cmd_;

}

bool KinovaRobot::commandChanged() const
{
    // Check if the squared sum of differences between the last sent command and
    // the current one is over a certain threshold.
    static const double EPS = 1e-6;

    double sum = 0.0;
    for (int i = 0; i < cur_cmd_.size(); ++i) {
        double d = cur_cmd_[i] - last_cmd_[i];
        sum += d*d;
    }

    return sum > EPS;
}

