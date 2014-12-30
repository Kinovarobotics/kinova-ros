#include <kinova_ros_ctrl/kinova_hardware_interface.hpp>
#include <kinova_robot/kinova_robot.hpp>

using namespace kinova_ros_ctrl;
using namespace kinova_robot;

KinovaHardwareInterface::KinovaHardwareInterface(ros::NodeHandle& n,
                                                 ros::NodeHandle& np)
{
    std::string serial;
    np.param("serial", serial, std::string(""));
    try {
        robot_.reset(new KinovaRobot(serial));
    } catch (KinovaException e) {
        ROS_FATAL("Could not initialize hardware interface: %s", e.what());
        return;
    }
    
    typedef hardware_interface::JointStateHandle JSH;
    for (int i = 0; i < robot_->numJoints(); ++i) {
        std::stringstream jns;
        jns << robot_->robotName()
            << "_joint_"
            << i + 1;
        JSH handle(jns.str(),
                   &(robot_->state().position[i]),
                   &(robot_->state().velocity[i]),
                   &(robot_->state().torque[i]));
        jsi_.registerHandle(handle); 
    }
    registerInterface(&jsi_); // From RobotHW base class.

    // TODO: Command interface.
   
}

KinovaHardwareInterface::~KinovaHardwareInterface()
{
}

void KinovaHardwareInterface::read()
{
    if (robot_) {
        robot_->updateState();
    } else {
        ROS_WARN_THROTTLE(1.0, 
                          "Invalid robot hardware interface, "
                          "cannot update state.");
    }
}

void KinovaHardwareInterface::write()
{
    // TODO: robot_->setPosition(...).
    // robot_->sendCommand();
}

