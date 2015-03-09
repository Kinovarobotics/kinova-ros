#include <kinova_ros_ctrl/kinova_hardware_interface.hpp>
#include <kinova_robot/kinova_robot.hpp>

using namespace kinova_ros_ctrl;
using namespace kinova_robot;

KinovaHardwareInterface::KinovaHardwareInterface(ros::NodeHandle& n,
                                                 ros::NodeHandle& np):
    cycle_(0)
{
    std::string serial;
    np.param("serial", serial, std::string(""));
    try
    {
        robot_.reset(new KinovaRobot(serial));
    }
    catch (KinovaException e)
    {
        ROS_FATAL("Could not initialize hardware interface: %s", e.what());
        return;
    }

    np.param("w_skip", w_skip_, 0);

    cmd_.resize(robot_->numJoints());

    typedef hardware_interface::JointStateHandle JSH;
    typedef hardware_interface::JointHandle      JH;
    for (int i = 0; i < robot_->numJoints(); ++i)
    {
        std::stringstream jns;

        // Change name of robot based on type and name
        jns << robot_->robotName();

        if (i < 6)
        {
            jns << "_joint_"        << i + 1;
        }
        else
        {
            jns << "_joint_finger_" << i - 5;
        }
        std::string jn = jns.str();
        std::cout << jn << std::endl;
        JSH handle(jn,
                   &(robot_->state().position[i]),
                   &(robot_->state().velocity[i]),
                   &(robot_->state().torque[i]));
        jsi_.registerHandle(handle);

        JH pos_h(jsi_.getHandle(jn),
                 &cmd_[i]);
        pji_.registerHandle(pos_h);

    }
    registerInterface(&jsi_); // From RobotHW base class.
    registerInterface(&pji_); // From RobotHW base class.

}

KinovaHardwareInterface::~KinovaHardwareInterface()
{
}

void KinovaHardwareInterface::read()
{
    if (robot_)
    {
        robot_->updateState();
        // Copy current position over command vector, command points will
        // overwrite them for the controlled joints.
        cmd_ = robot_->state().position;
    }
    else
    {
        ROS_WARN_THROTTLE(1.0,
                          "Invalid robot hardware interface, "
                          "cannot update state.");
    }
}

void KinovaHardwareInterface::write()
{
    if (!robot_)
    {
        ROS_WARN_THROTTLE(1.0,
                          "Invalid robot hardware interface, "
                          "cannot write commands.");
        return;
    }

    robot_->setPosition(cmd_);

    // Throttling test:
    if (w_skip_ > 0)
    {
        if ((++cycle_ % w_skip_) > 0)
        {
            // Only actually send commands 1 every w_skip cycles.
            return;
        }
        cycle_ = 0;
    }

    robot_->sendCommand();
}
