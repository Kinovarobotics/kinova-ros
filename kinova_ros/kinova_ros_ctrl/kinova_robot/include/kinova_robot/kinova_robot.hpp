#ifndef KINOVA_ROBOT_HPP
#define KINOVA_ROBOT_HPP

#include <vector>
#include <string>
#include <stdexcept>

namespace kinova_robot
{
    /// \brief A vector containing a single value (position, velocity, ...) for
    ///        all joints of the robot.
    typedef std::vector<double> StateVector;
    
    /// \brief The full state of the robot.
    struct RobotState
    {
        /// \brief Constructor.
        /// 
        /// \param n Number of joints.
        ///          Default: 0.
        RobotState(int n = 0):
            position(n, 0.0),
            velocity(n, 0.0),
            torque(n, 0.0)
        {}

        StateVector position;
        StateVector velocity;
        StateVector torque;
    };

    /// \brief Default (runtime) exception type.
    typedef std::runtime_error KinovaException;

    /// \brief A minimalist wrapper for the Kinova JACO API.
    ///
    /// An instance of this class represents a single Kinova JACO arm.
    /// Only includes joint-space control for now.
    ///
    /// A typical control loop for the robot would look like this:
    ///
    ///   KinovaRobot robot;
    ///   StateVector cmd;
    ///
    ///   while (running()) {
    ///       robot.updateState();                   // Get the current state of
    ///                                              // the robot.
    ///       higher_level_ctrl(robot.state(), cmd); // High-level control to
    ///                                              // calculate the next 
    ///                                              // control point.
    ///       robot.setPosition(cmd);                // Set the new control
    ///                                              // point.
    ///       robot.sendCommand();                   // Send the new command
    ///                                              // through the API.
    ///       sleep(time_to_next_cycle);             // Regulate cycles to
    ///                                              // 6-10ms.
    ///   }
    /// 
    class KinovaRobot
    {
    public:

    private:
        std::string robot_name_;
        RobotState  state_;
        StateVector cur_cmd_;           // In position only!
        StateVector last_cmd_;

        int         num_joints_;
        int         num_fingers_;
        double      j6_angle_offset_;   // Either 260 or 270 (depends on robot
                                        // type).

    public:
        /// \brief Constructor.
        ///
        /// Automatically connect to a specific Kinova arm with the USB API.
        /// Will stay connected until the instance is destroyed
        /// 
        /// \param serial Serial number of the arm to connect to.
        ///        If left empty, the instance will connect to the first robot
        //
        ///        found.
        KinovaRobot(const std::string& serial = std::string(""));

        /// \brief Destructor.
        ///
        /// Automatically disconnects from the API.
        virtual ~KinovaRobot();

        /// \brief Return the model name of the robot currently controlled.
        std::string robotName() const { return robot_name_; }

        /// \brief Return the total number of joints (including fingers).
        int numJoints()  const { return num_joints_; }
        /// \brief Return the number of fingers.
        int numFingers() const { return num_fingers_; }

        /// \brief Request a state update from the API.
        void updateState();

        /// \brief Return the current full state of the robot.
        const RobotState& state() const { return state_; }

        /// \brief Set the position control point for the robot.
        ///
        /// NOTE: Does not directly send the command to the arm, see
        /// sendCommand() for this.
        void setPosition(const StateVector& cmd);

        /// \brief Update the command point through the API.
        void sendCommand();

    private:
        void setupRobot(int type);
        bool commandChanged() const;

    };
}


#endif // KINOVA_ROBOT_HPP
