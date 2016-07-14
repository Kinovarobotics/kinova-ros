#ifndef JACO_DRIVER_JACO_TRAJECTORY_ACTION_H
#define JACO_DRIVER_JACO_TRAJECTORY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "jaco_driver/jaco_comm.h"

namespace jaco {

    class JacoTrajectoryActionServer {
    public:
        JacoTrajectoryActionServer(JacoComm &, const ros::NodeHandle &n);
        ~JacoTrajectoryActionServer();

        void actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &);

    private:
        ros::NodeHandle node_handle_;
        JacoComm &arm_comm_;

        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;
        
        void convertDHAnglesToPhysical(AngularInfo &angles);
        void convertPhysicalAnglesToDH(AngularInfo &angles);
        AngularInfo computeError(AngularInfo &goal, AngularInfo &current);
        float getShortestAngleDistance(float &reference, float &current);
        void normalizeAngles(AngularInfo &angles);
        void printAngles(const char* desc, AngularInfo &angles);
        double normalize(const double value, const double start, const double end);
        bool areValuesClose(float first, float second, float tolerance);

        void checkCurrentWayPointError(float &error, bool &stop);
        void setPControllerJointVelocity(float & actuator, float error, bool &reached_goal, bool &exit_now);


        
    };
}
#endif  // JACO_DRIVER_JACO_TRAJECTORY_ACTION_H
