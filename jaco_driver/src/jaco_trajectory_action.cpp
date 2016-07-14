/**
 * @file jaco_trajectory_action.cpp
 * 
 * @author Bc. Matěj Balga
 */

#include <kinova/KinovaTypes.h>
#include <cmath>
#include "jaco_driver/jaco_trajectory_action.h"
#include "jaco_driver/jaco_types.h"
#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_defines.h"

#define PI 3.14159265359
#define TOLERANCE 1 // Goal tolerance in degrees
#define WAYPOINT_TOLERANCE 60
#define MAX_ERROR_TOLERANCE 40 //Max Goal tolerance in degress for final snap to joint pose
#define KP 2 // P-regulator constant

namespace jaco {

    JacoTrajectoryActionServer::JacoTrajectoryActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh, "controller"),
      action_server_(node_handle_,
        "follow_joint_trajectory",
        boost::bind(&JacoTrajectoryActionServer::actionCallback, this, _1),
        false)
    {
        action_server_.start();
        ROS_INFO_STREAM("JACO FollowJointTrajectory action server has started.");
    }

    JacoTrajectoryActionServer::~JacoTrajectoryActionServer() {
    }

    void JacoTrajectoryActionServer::actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
        try {
            if (arm_comm_.isStopped()) {
                ROS_ERROR_STREAM("Could not complete joint angle action because the arm is 'stopped'.");
                action_server_.setAborted();
                return;
            }

            // Clear all previous trajectories and points
            arm_comm_.initTrajectory();

            trajectory_msgs::JointTrajectoryPoint goal_waypoint =
                    goal->trajectory.points[goal->trajectory.points.size() - 1];

            trajectory_msgs::JointTrajectoryPoint start_waypoint =
                    goal->trajectory.points[0];

            JacoAngles current_joint_angles, goal_joint_angles, start_joint_angles;
            goal_joint_angles.Actuator1 = goal_waypoint.positions[0] * (180 / PI);
            goal_joint_angles.Actuator2 = goal_waypoint.positions[1] * (180 / PI);
            goal_joint_angles.Actuator3 = goal_waypoint.positions[2] * (180 / PI);
            goal_joint_angles.Actuator4 = goal_waypoint.positions[3] * (180 / PI);
            goal_joint_angles.Actuator5 = goal_waypoint.positions[4] * (180 / PI);
            goal_joint_angles.Actuator6 = goal_waypoint.positions[5] * (180 / PI);
            convertDHAnglesToPhysical(goal_joint_angles);
            normalizeAngles(goal_joint_angles);

            start_joint_angles.Actuator1 = start_waypoint.positions[0] * (180 / PI);
            start_joint_angles.Actuator2 = start_waypoint.positions[1] * (180 / PI);
            start_joint_angles.Actuator3 = start_waypoint.positions[2] * (180 / PI);
            start_joint_angles.Actuator4 = start_waypoint.positions[3] * (180 / PI);
            start_joint_angles.Actuator5 = start_waypoint.positions[4] * (180 / PI);
            start_joint_angles.Actuator6 = start_waypoint.positions[5] * (180 / PI);
            convertDHAnglesToPhysical(start_joint_angles);
            normalizeAngles(start_joint_angles);

            ///////////////////////////////////
            //Sanity Check that arm is actually starting from the first waypoint in the trajectory
            ///////////////////////////////////
            arm_comm_.getJointAngles(current_joint_angles);
            normalizeAngles(current_joint_angles);
            AngularInfo error = computeError(start_joint_angles, current_joint_angles);
            if (std::abs(error.Actuator1) > MAX_ERROR_TOLERANCE) {
                ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator 1 before even starting, setting to aborted");
                action_server_.setAborted();
                ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
                return;
            }
            if (std::abs(error.Actuator2) > MAX_ERROR_TOLERANCE) {
                ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator 2 before even starting, setting to aborted");
                action_server_.setAborted();
                ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
                return;
            }
            if (std::abs(error.Actuator3) > MAX_ERROR_TOLERANCE) {
                ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator 3 before even starting, setting to aborted");
                action_server_.setAborted();
                ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
                return;
            }
            if (std::abs(error.Actuator4) > MAX_ERROR_TOLERANCE) {
                ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator 4 before even starting, setting to aborted");
                action_server_.setAborted();
                ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
                return;
            }
            if (std::abs(error.Actuator5) > MAX_ERROR_TOLERANCE) {
                ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator 5 before even starting, setting to aborted");
                action_server_.setAborted();
                return;
            }
            if (std::abs(error.Actuator6) > MAX_ERROR_TOLERANCE) {
                ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator 6 before even starting, setting to aborted");
                action_server_.setAborted();
                ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
                return;
            }

            ros::Time start_time = ros::Time::now();
            bool stop = false;
            bool read_joint_angles = false;
            for (unsigned int i = 0; i < goal->trajectory.points.size(); i++) {

                // Get the waypoint to be reached
                trajectory_msgs::JointTrajectoryPoint waypoint =
                        goal->trajectory.points[i];

                // Initialize a trajectory point which we will be constantly
                // feeding to the robot until the waypoint time's up or until
                // we are in the final configuration.
                TrajectoryPoint point;
                point.InitStruct();
                memset(&point, 0, sizeof (point));

                // The trajectory consists of angular velocity waypoints
                point.Position.Type = ANGULAR_VELOCITY;
                
                // Set up the trajectory point with waypoint values converted
                // from [rad/s] to [deg/s]
                point.Position.Actuators.Actuator1 = -waypoint.velocities[0] * (180 / PI);
                point.Position.Actuators.Actuator2 = +waypoint.velocities[1] * (180 / PI);
                point.Position.Actuators.Actuator3 = -waypoint.velocities[2] * (180 / PI);
                point.Position.Actuators.Actuator4 = -waypoint.velocities[3] * (180 / PI);
                point.Position.Actuators.Actuator5 = -waypoint.velocities[4] * (180 / PI);
                point.Position.Actuators.Actuator6 = -waypoint.velocities[5] * (180 / PI);

                JacoAngles current_waypoint_angles;
                current_waypoint_angles.Actuator1 = waypoint.positions[0] * (180 / PI);
                current_waypoint_angles.Actuator2 = waypoint.positions[1] * (180 / PI);
                current_waypoint_angles.Actuator3 = waypoint.positions[2] * (180 / PI);
                current_waypoint_angles.Actuator4 = waypoint.positions[3] * (180 / PI);
                current_waypoint_angles.Actuator5 = waypoint.positions[4] * (180 / PI);
                current_waypoint_angles.Actuator6 = waypoint.positions[5] * (180 / PI);
                convertDHAnglesToPhysical(current_waypoint_angles);
                normalizeAngles(current_waypoint_angles);

                read_joint_angles = false;
                while(!read_joint_angles){
                    try{
                        arm_comm_.getJointAngles(current_joint_angles);
                        read_joint_angles = true;
                    } catch (const std::exception& e) {
                        ROS_ERROR_STREAM(e.what());
                        ROS_ERROR_STREAM("Carrying on anyway!");
                    }
                }

                normalizeAngles(current_joint_angles);
                error = computeError(current_waypoint_angles, current_joint_angles);

                stop = false;
                checkCurrentWayPointError(error.Actuator1, stop);
                checkCurrentWayPointError(error.Actuator2, stop);
                checkCurrentWayPointError(error.Actuator3, stop);
                checkCurrentWayPointError(error.Actuator4, stop);
                checkCurrentWayPointError(error.Actuator5, stop);
                checkCurrentWayPointError(error.Actuator6, stop);
                if(stop)
                {
                    ROS_ERROR_STREAM("Inside Open Loop Velocity Controller: Way to large of error in JacoTrajectoryActionServer actuator setting to aborted");
                    ROS_ERROR_STREAM("error.Actuator1: " << error.Actuator1 << std::endl);
                    ROS_ERROR_STREAM("error.Actuator2: " << error.Actuator2 << std::endl);
                    ROS_ERROR_STREAM("error.Actuator3: " << error.Actuator3 << std::endl);
                    ROS_ERROR_STREAM("error.Actuator4: " << error.Actuator4 << std::endl);
                    ROS_ERROR_STREAM("error.Actuator5: " << error.Actuator5 << std::endl);
                    ROS_ERROR_STREAM("error.Actuator6: " << error.Actuator6 << std::endl);
                    action_server_.setAborted();
                    return;
                }

                ros::Rate r(100);   // The loop below will run at 100Hz
                
                while ((waypoint.time_from_start >= ros::Time::now() - start_time)) {
                    ros::spinOnce();

                    // If preempted, stop the motion, enable the arm again and
                    // send a notification of preemption
                    if (action_server_.isPreemptRequested() || !ros::ok()) {
                        arm_comm_.stopAPI();
                        arm_comm_.startAPI();
                        action_server_.setPreempted();
                        return;
                    } else if (arm_comm_.isStopped()) {
                        action_server_.setAborted();
                        return;
                    }

                    // Get the current real angles and normalize them for comparing
                    // with the target and adjusting the execution
//                    arm_comm_.getJointAngles(current_joint_angles);
//                    normalizeAngles(current_joint_angles);

                    // Adjust actuator velocities accordingly
                    arm_comm_.addTrajectoryPoint(point);

                    // Ensures executing at 100Hz
                    r.sleep();
                }
            }

            stop = false;

            ros::Rate r(100); // The loop below will run at 100Hz (every 10ms)
            
            // This loop finalizes the movement by checking angular distance
            // of joints from the desired configuration. Instead of moving
            // to the last waypoint, we move and check each joint separately
            // to ensure we are at the right position.
            bool reached_goal = false;
            while (!reached_goal) {
                // Get the current real angles and normalize them for comparing
                // with the target and adjusting the execution
                arm_comm_.getJointAngles(current_joint_angles);
                normalizeAngles(current_joint_angles);
                AngularInfo error = computeError(goal_joint_angles, current_joint_angles);
                
                // Initialize a trajectory point which we will be constantly
                // feeding to the robot untilwe are in the final configuration.
                TrajectoryPoint point;
                point.InitStruct();
                memset(&point, 0, sizeof (point));
                
                point.Position.Actuators.Actuator1 = 0.0;
                point.Position.Actuators.Actuator2 = 0.0;
                point.Position.Actuators.Actuator3 = 0.0;
                point.Position.Actuators.Actuator4 = 0.0;
                point.Position.Actuators.Actuator5 = 0.0;
                point.Position.Actuators.Actuator6 = 0.0;
                

                // The trajectory consists of angular velocity waypoints
                point.Position.Type = ANGULAR_VELOCITY;
                
                reached_goal = true;
                bool exit_now = false;//is something really wrong
                setPControllerJointVelocity(point.Position.Actuators.Actuator1, error.Actuator1,  reached_goal, exit_now);
                setPControllerJointVelocity(point.Position.Actuators.Actuator2, error.Actuator2,  reached_goal, exit_now);
                setPControllerJointVelocity(point.Position.Actuators.Actuator3, error.Actuator3,  reached_goal, exit_now);
                setPControllerJointVelocity(point.Position.Actuators.Actuator4, error.Actuator4,  reached_goal, exit_now);
                setPControllerJointVelocity(point.Position.Actuators.Actuator5, error.Actuator5,  reached_goal, exit_now);
                setPControllerJointVelocity(point.Position.Actuators.Actuator6, error.Actuator6,  reached_goal, exit_now);
                if(exit_now)
                {
                    ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator setting to aborted");
                    action_server_.setAborted();
                    ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
                    return;
                }
                
                // Adjust actuator velocities accordingly
                arm_comm_.addTrajectoryPoint(point);
                ros::spinOnce();
                // Ensure execution at 100Hz
                r.sleep();
            }
            
            // If we got here, it seems that we succeeded
            action_server_.setSucceeded();

        } catch (const std::exception& e) {
            // Something rather terrible has happened - report it!
            ROS_ERROR_STREAM(e.what());
            action_server_.setAborted();
        }
    }

    void JacoTrajectoryActionServer::checkCurrentWayPointError(float &error, bool &stop)
    {
        if (std::abs(error) > WAYPOINT_TOLERANCE)
        {
            ROS_ERROR_STREAM("Way to large of error in JacoTrajectoryActionServer actuator, error: " << error);
            action_server_.setAborted();
            ROS_INFO_STREAM("Leaving: JacoTrajectoryActionServer::actionCallback\n");
            stop=true;
        }
    }

    void JacoTrajectoryActionServer::setPControllerJointVelocity(float & actuator, float error, bool &reached_goal, bool &exit_now) {
        // If not in the position yet, adjust the motor
        if (std::abs(error) > TOLERANCE)
        {
            if (std::abs(error) > MAX_ERROR_TOLERANCE) {
                exit_now = true;
            }

            reached_goal = false;

            float min_speed = 0.1;
            if (std::abs(error) < min_speed )
            {
                if (error < 0)
                {
                    actuator = -min_speed;
                }
                else
                {
                    actuator = min_speed;
                }
            }
            else
            {
                actuator = KP * error;
            }
        }

    }


    /**
     * Converts angles from DH convention to JACO's physical angles.
     * 
     * @param angles Angles to be converted
     */
    void JacoTrajectoryActionServer::convertDHAnglesToPhysical(AngularInfo &angles) {
        double j6o = arm_comm_.robotType() == ROBOT_TYPE_JACO ? JACO_JOINT_2_ANGLE : MICO_JOINT_2_ANGLE;
        angles.Actuator1 = 180 - angles.Actuator1;
        angles.Actuator2 = j6o + angles.Actuator2;
        angles.Actuator3 =  90 - angles.Actuator3;
        angles.Actuator4 = 180 - angles.Actuator4;
        angles.Actuator5 = 180 - angles.Actuator5;
        angles.Actuator6 = j6o - angles.Actuator6;
    }
    
    /**
     * Converts angles from JACO's physical angles to DH convention.
     * 
     * @param angles Angles to be converted
     */
    void JacoTrajectoryActionServer::convertPhysicalAnglesToDH(AngularInfo &angles) {
        double j6o = arm_comm_.robotType() == ROBOT_TYPE_JACO ? JACO_JOINT_2_ANGLE : MICO_JOINT_2_ANGLE;
        angles.Actuator1 = 180 - angles.Actuator1;
        angles.Actuator2 = angles.Actuator2 - j6o;
        angles.Actuator3 =  90 - angles.Actuator3;
        angles.Actuator4 = 180 - angles.Actuator4;
        angles.Actuator5 = 180 - angles.Actuator5;
        angles.Actuator6 = j6o - angles.Actuator6;
    }

    /**
     * Returns the difference between two joint configurations.
     * 
     * @param goal The goal configuration.
     * @param current Current configuration.
     * @return Difference joint configuration.
     */
    AngularInfo JacoTrajectoryActionServer::computeError(AngularInfo &reference, AngularInfo &current) {
        AngularInfo error;
        
        error.Actuator1 = getShortestAngleDistance(reference.Actuator1, current.Actuator1);      
        error.Actuator2 = getShortestAngleDistance(reference.Actuator2, current.Actuator2);
        error.Actuator3 = getShortestAngleDistance(reference.Actuator3, current.Actuator3);
        error.Actuator4 = getShortestAngleDistance(reference.Actuator4, current.Actuator4);
        error.Actuator5 = getShortestAngleDistance(reference.Actuator5, current.Actuator5);
        error.Actuator6 = getShortestAngleDistance(reference.Actuator6, current.Actuator6);
        return error;
    }
    
    float JacoTrajectoryActionServer::getShortestAngleDistance(float &reference, float &current) {
        float error = reference - current; 
        
        // Go the shortest way: if the error is bigger than PI, let's take
        // the other way around (typical e.g. for reference=-179 and feedback=179)
        // error in that case would be 358 but in fact those positions are
        // just 2 degrees from each other
        if (std::abs(error) > 180) {
            error = (error > 0 ? (reference - current) - 360 : 360 - (current - reference));
        }
        
        return error;
    }

    /**
     * Normalizes the angles to lie within -180 to 180 degrees.
     * 
     * @param angles
     */
    void JacoTrajectoryActionServer::normalizeAngles(AngularInfo &angles) {
        angles.Actuator1 = normalize(angles.Actuator1, -180.0, 180.0);
        angles.Actuator2 = normalize(angles.Actuator2, -180.0, 180.0);
        angles.Actuator3 = normalize(angles.Actuator3, -180.0, 180.0);
        angles.Actuator4 = normalize(angles.Actuator4, -180.0, 180.0);
        angles.Actuator5 = normalize(angles.Actuator5, -180.0, 180.0);
        angles.Actuator6 = normalize(angles.Actuator6, -180.0, 180.0);
    }

    // DEBUG FUNCTION
    void JacoTrajectoryActionServer::printAngles(const char* desc, AngularInfo &angles) {
        char wp_info[1024];
        sprintf(wp_info, "%s:\t[%f, %f, %f, %f, %f, %f]",
                desc,
                angles.Actuator1,
                angles.Actuator2,
                angles.Actuator3,
                angles.Actuator4,
                angles.Actuator5,
                angles.Actuator6);
        ROS_INFO_STREAM(wp_info);
    }
    
    /**
     * Normalizes any number to an arbitrary range by assuming the range wraps 
     * around when going below min or above max.
     * 
     * @param value The number to be normalized
     * @param start Lower threshold
     * @param end   Upper threshold
     * @return Returns the normalized number.
     */
    double JacoTrajectoryActionServer::normalize(const double value, const double start, const double end) {
        const double width = end - start; // 
        const double offsetValue = value - start; // value relative to 0

        return ( offsetValue - (floor(offsetValue / width) * width)) +start;
        // + start to reset back to start of original range
    }
    
    /**
     * Returns true if the distance of the values lies within _tolerance_.
     * 
     * @param first The first value
     * @param second The second value
     * @param tolerance Maximum distance of the values
     * @return TRUE if the distance is less than tolerance
     */
    bool JacoTrajectoryActionServer::areValuesClose(float first, float second, float tolerance) {
        return ((first <= second + tolerance) && (first >= second - tolerance));
    }
}
