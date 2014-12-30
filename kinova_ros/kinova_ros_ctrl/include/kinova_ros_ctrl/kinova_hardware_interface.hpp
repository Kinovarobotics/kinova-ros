#ifndef KINOVA_HARDWARE_INTERFACE_HPP
#define KINOVA_HARDWARE_INTERFACE_HPP

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace kinova_robot
{
    // Forward declaration:
    class KinovaRobot;
};

namespace kinova_ros_ctrl
{
    /// \brief Hardware interface for Kinova robots.
    ///
    /// This is meant to be run as a ROS node to support ros_control plugins.
    /// It will automatically connect to a single Kinova robot and expose a set
    /// of position command interfaces for joint-space trajectory control.
    ///
    /// ROS Parameters:
    ///  - serial: The serial number of the robot to connect to.
    ///            If left blank (""), the node will connect to the first 
    ///            one found.
    ///            Default: "".
    class KinovaHardwareInterface: public hardware_interface::RobotHW
    {
    private:
        boost::scoped_ptr<kinova_robot::KinovaRobot> robot_;
        hardware_interface::JointStateInterface      jsi_;
        hardware_interface::PositionJointInterface   pji_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        KinovaHardwareInterface(ros::NodeHandle& n, ros::NodeHandle& np);

        /// \brief Destructor.
        ~KinovaHardwareInterface();
        


    };
}

#endif // KINOVA_HARDWARE_INTERFACE_HPP

