/**
 *  File: jaco_trajectory_angles_action.cpp
 *  Desc: Class for interfacing moveIt with the jaco arm.
 *  Auth: Plinio Moreno
 *
 *
 */


#include <kinova/KinovaTypes.h>

#include "jaco_driver/jaco_trajectory_angles_action.h"

#include "jaco_driver/jaco_types.h"
using namespace std;
namespace jaco
{

JacoTrajectoryAnglesActionServer::JacoTrajectoryAnglesActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh, "joint_trajectory_angles"),
      action_server_(node_handle_, "joint_velocity_controller",
                     boost::bind(&JacoTrajectoryAnglesActionServer::actionCallback, this, _1), false)
{
    double tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
    node_handle_.param<double>("rate_hz", rate_hz_, 600.0);
    node_handle_.param<double>("tolerance", tolerance, 2.0);
    tolerance_ = (float)tolerance;
    j6o_ = arm_comm.robotType() == 2 ? 270.0 : 260.0;
    max_curvature_= 20.0;
    action_server_.start();
}


JacoTrajectoryAnglesActionServer::~JacoTrajectoryAnglesActionServer()
{
}


static inline double avoid_mul_loops_first_point(double actual_angle, double trajectory_angle)
{
  double revolutions = fabs((actual_angle-trajectory_angle)/(1.0 * M_PI));
  
  if (revolutions/round(revolutions)>0.99 && revolutions/round(revolutions)<1.01 && revolutions > 0.99)
    return (actual_angle-trajectory_angle);
  else
    return 0.0;
}
void JacoTrajectoryAnglesActionServer::actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    JacoAngles current_joint_angles;
    double current_joint_pos[NUM_JACO_JOINTS];
    double delta_for_trajectory[NUM_JACO_JOINTS];


	if (arm_comm_.isStopped())
        {
	    control_msgs::FollowJointTrajectoryResult result;
            ROS_INFO("Could not complete joint angle action because the arm is 'stopped'.");
            result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            action_server_.setSucceeded(result);
            return;
        }
  float trajectoryPoints[NUM_JACO_JOINTS][goal->trajectory.points.size()];
  int numPoints = goal->trajectory.points.size();

  // Converting the actual arm angles to the DH angles
  arm_comm_.getJointAnglesCommand(current_joint_angles);
  current_joint_pos[0] = (180 - current_joint_angles.Actuator1) * DEG_TO_RAD;
  current_joint_pos[1] = (current_joint_angles.Actuator2 - j6o_) * DEG_TO_RAD;
  current_joint_pos[2] = (90 - current_joint_angles.Actuator3) * DEG_TO_RAD;
  current_joint_pos[3] = (180 - current_joint_angles.Actuator4) * DEG_TO_RAD;
  current_joint_pos[4] = (180 - current_joint_angles.Actuator5) * DEG_TO_RAD;
  current_joint_pos[5] = (270 - current_joint_angles.Actuator6) * DEG_TO_RAD;
  // Checking if the first point of the trajectory of each joint commands for more than 180 deg rotation
  // If that is the case, the angles are shifted for the controller to work properly
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    delta_for_trajectory[i]= avoid_mul_loops_first_point(current_joint_pos[i], goal->trajectory.points.at(0).positions.at(i));
  }
  // Converting the DH angles of the trajectory to the actual arm angles
  for (unsigned int i = 0; i < numPoints; i++)
  {
    trajectoryPoints[0][i] = PI - goal->trajectory.points.at(i).positions.at(0)-delta_for_trajectory[0];
    trajectoryPoints[1][i] = j6o_*DEG_TO_RAD + goal->trajectory.points.at(i).positions.at(1)+delta_for_trajectory[1];
    trajectoryPoints[2][i] = PI/2.0 - goal->trajectory.points.at(i).positions.at(2)-delta_for_trajectory[2];
    trajectoryPoints[3][i] = PI - goal->trajectory.points.at(i).positions.at(3)-delta_for_trajectory[3];
    trajectoryPoints[4][i] = PI - goal->trajectory.points.at(i).positions.at(4)-delta_for_trajectory[4];
    trajectoryPoints[5][i] = 3.0*PI/2.0 - goal->trajectory.points.at(i).positions.at(5)-delta_for_trajectory[5];

  //initialize arrays needed to fit a smooth trajectory to the given points
  ecl::Array<double> timePoints(numPoints);
  timePoints[0] = 0.0;
  vector<ecl::Array<double> > jointPoints;
  jointPoints.resize(NUM_JACO_JOINTS);
  float prevPoint[NUM_JACO_JOINTS];
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    jointPoints[i].resize(numPoints);
    jointPoints[i][0] = trajectoryPoints[i][0];
    prevPoint[i] = trajectoryPoints[i][0];
  }

  //determine time component of trajectories for each joint
  for (unsigned int i = 1; i < numPoints; i++)
  {
    float maxTime = 0.0;
    for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      //calculate approximate time required to move to the next position
      float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
      if (j <= 2)
        time /= LARGE_ACTUATOR_VELOCITY;
      else
        time /= SMALL_ACTUATOR_VELOCITY;

      if (time > maxTime)
        maxTime = time;

      jointPoints[j][i] = trajectoryPoints[j][i];
      prevPoint[j] = trajectoryPoints[j][i];
    }

    timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
  }

  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(6);
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
  {
    ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], max_curvature_);
    splines.at(i) = tempSpline;
  }

//control loop
  bool trajectoryComplete = false;
  double startTime = ros::Time::now().toSec();
  double t = 0;
  float error[NUM_JACO_JOINTS];
  float totalError;
  float prevError[NUM_JACO_JOINTS] = {0};
  float currentPoint;
  AngularPosition position_data;
  AngularInfo trajPoint;
  trajPoint.InitStruct();
  ros::Rate rate(rate_hz_);

while (!trajectoryComplete)
  {
    if (arm_comm_.isStopped())
    {
      control_msgs::FollowJointTrajectoryResult result;
            ROS_INFO("Could not complete joint angle action because the arm is 'stopped'.");
            result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            action_server_.setSucceeded(result);
            return;
    }

    //check for preempt requests from clients
    if (action_server_.isPreemptRequested())
    {
      //stop gripper control
      trajPoint.Actuator1 = 0.0;
      trajPoint.Actuator2 = 0.0;
      trajPoint.Actuator3 = 0.0;
      trajPoint.Actuator4 = 0.0;
      trajPoint.Actuator5 = 0.0;
      trajPoint.Actuator6 = 0.0;
      arm_comm_.setJointVelocities(trajPoint);
      //executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      action_server_.setPreempted();
      ROS_INFO("Joint trajectory server preempted by client");

      return;
    }

    //get time for trajectory
    t = ros::Time::now().toSec() - startTime;
    if (t > timePoints.at(timePoints.size() - 1))
    {
      //use final trajectory point as the goal to calculate error until the error
      //is small enough to be considered successful
      arm_comm_.getJointAnglesCommand(current_joint_angles);
      current_joint_pos[0] = current_joint_angles.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = current_joint_angles.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = current_joint_angles.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = current_joint_angles.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = current_joint_angles.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = current_joint_angles.Actuator6 * DEG_TO_RAD;

      totalError = 0;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
	error[i] = (splines.at(i))(timePoints.at(timePoints.size() - 1)) - current_joint_pos[i];
        totalError += fabs(error[i]);
      }

      if (totalError < .03)
      {
        trajPoint.Actuator1 = 0.0;
        trajPoint.Actuator2 = 0.0;
        trajPoint.Actuator3 = 0.0;
        trajPoint.Actuator4 = 0.0;
        trajPoint.Actuator5 = 0.0;
        trajPoint.Actuator6 = 0.0;
	arm_comm_.setJointVelocities(trajPoint);
        trajectoryComplete = true;
        ROS_INFO("Trajectory complete!");
        break;
      }
    }
    else
    {
      //calculate error
      arm_comm_.getJointAnglesCommand(current_joint_angles);
      current_joint_pos[0] = current_joint_angles.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = current_joint_angles.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = current_joint_angles.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = current_joint_angles.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = current_joint_angles.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = current_joint_angles.Actuator6 * DEG_TO_RAD;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
	error[i] = (splines.at(i))(t) - current_joint_pos[i];
      }
    }

    //calculate control input
    //populate the velocity command
    trajPoint.Actuator1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
    trajPoint.Actuator2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
    trajPoint.Actuator3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
    trajPoint.Actuator4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajPoint.Actuator5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajPoint.Actuator6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);

    //for debugging:
    //cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

    //send the velocity command
    arm_comm_.setJointVelocities(trajPoint);//AngularInfo &joint_vel
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      prevError[i] = error[i];
    }

    rate.sleep();
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  action_server_.setSucceeded(result);

}

}  // namespace jaco
