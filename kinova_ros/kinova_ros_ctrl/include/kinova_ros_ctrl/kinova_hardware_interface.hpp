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
    /// The interface is meant to be used in a ControllerManager in this
    /// fashion:
    ///  KinovaHardwareInterface hw(n, np);    // The robot will be ready.
    ///  ControllerManager       cm(&hw);
    ///  ros::Duration           period(0.01); // 100 Hz.
    ///  while (ros::ok()) {
    ///    ros::Time start = ros::Time::now();
    ///    ros::Time end   = start + period;
    ///    hw.read();
    ///    cm.update(now, period);
    ///    hw.write(); 
    ///    ros::Duration left(end - ros::Time::now()).sleep();
    ///  }
    ///
    /// ROS Parameters:
    ///  - serial: The serial number of the robot to connect to.
    ///            If left blank (""), the node will connect to the first 
    ///            one found.
    ///            Default: "".
    ///  - w_skip: Optional throttle of sent commands.
    ///            If w_skip > 0, the controller will only send 1 in every
    ///            w_skip loop cycles.
    ///            This is meant to avoid overflowing the command buffer of the
    ///            robot.
    ///            Default: 0 (throttling disabled).
    class KinovaHardwareInterface: public hardware_interface::RobotHW
    {
    private:
        boost::scoped_ptr<kinova_robot::KinovaRobot> robot_;

        hardware_interface::JointStateInterface      jsi_;
        hardware_interface::PositionJointInterface   pji_;

        std::vector<double>                          cmd_;

        int                                          cycle_;
        int                                          w_skip_;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        KinovaHardwareInterface(ros::NodeHandle& n, ros::NodeHandle& np);

        /// \brief Destructor.
        ~KinovaHardwareInterface();
        
        /// \brief Read the state from the robot hardware.
        void read();

        /// \brief write the command to the robot hardware.
        void write();

    };
}

#endif // KINOVA_HARDWARE_INTERFACE_HPP

