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
    
    cmd_.resize(robot_->numJoints());

    typedef hardware_interface::JointStateHandle JSH;
    typedef hardware_interface::JointHandle      JH;
    for (int i = 0; i < robot_->numJoints(); ++i) {
        std::stringstream jns;
        jns << robot_->robotName()
            << "_joint_"
            << i + 1;
        std::string jn = jns.str();

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
    if (robot_) {
        robot_->updateState();
        // Copy current position over command vector, command points will
        // overwrite them for the controlled joints.
        cmd_ = robot_->state().position;
    } else {
        ROS_WARN_THROTTLE(1.0, 
                          "Invalid robot hardware interface, "
                          "cannot update state.");
    }
}

void KinovaHardwareInterface::write()
{
    if (!robot_) {
        ROS_WARN_THROTTLE(1.0, 
                          "Invalid robot hardware interface, "
                          "cannot write commands.");
        return;
    }

    robot_->setPosition(cmd_);
    robot_->sendCommand();
}

