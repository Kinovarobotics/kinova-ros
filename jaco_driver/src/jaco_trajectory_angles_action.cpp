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

/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}
void JacoTrajectoryAnglesActionServer::actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    /*control_msgs::FollowJointTrajectoryActionFeedback feedback;
    control_msgs::FollowJointTrajectoryActionResult result;
    JacoAngles current_joint_angles;
    ros::Time current_time = ros::Time::now();*/
    JacoAngles current_joint_angles;


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

  //get trajectory data
  //convert the data!!!!
    /*joint_state.position[0] = (180- jaco_angles.joint1) * (PI / 180);
    joint_state.position[1] = (jaco_angles.joint2 - j6o) * (PI / 180);
    joint_state.position[2] = (90-jaco_angles.joint3) * (PI / 180);
    joint_state.position[3] = (180-jaco_angles.joint4) * (PI / 180);
    joint_state.position[4] = (180-jaco_angles.joint5) * (PI / 180);
    joint_state.position[5] = (270-jaco_angles.joint6) * (PI / 180);*/
  for (unsigned int i = 0; i < numPoints; i++)
  {
    // Trajectory points in degrees
    /*
    trajectoryPoints[0][i] = 180.0 - goal->trajectory.points.at(i).positions.at(0)*RAD_TO_DEG;
    trajectoryPoints[1][i] = j6o_ + goal->trajectory.points.at(i).positions.at(1)*RAD_TO_DEG;
    trajectoryPoints[2][i] = 90.0 - goal->trajectory.points.at(i).positions.at(2)*RAD_TO_DEG;
    trajectoryPoints[3][i] = 180.0 - goal->trajectory.points.at(i).positions.at(3)*RAD_TO_DEG;
    trajectoryPoints[4][i] = 180.0 - goal->trajectory.points.at(i).positions.at(4)*RAD_TO_DEG;
    trajectoryPoints[5][i] = 270.0 - goal->trajectory.points.at(i).positions.at(5)*RAD_TO_DEG;*/

    // Trajectory points in radians
    
    trajectoryPoints[0][i] = PI - goal->trajectory.points.at(i).positions.at(0);
    trajectoryPoints[1][i] = j6o_*DEG_TO_RAD + goal->trajectory.points.at(i).positions.at(1);
    trajectoryPoints[2][i] = PI/2.0 - goal->trajectory.points.at(i).positions.at(2);
    trajectoryPoints[3][i] = PI - goal->trajectory.points.at(i).positions.at(3);
    trajectoryPoints[4][i] = PI - goal->trajectory.points.at(i).positions.at(4);
    trajectoryPoints[5][i] = 3.0*PI/2.0 - goal->trajectory.points.at(i).positions.at(5);
    /*trajectoryPoints[0][i] = goal->trajectory.points.at(i).positions.at(0);
    trajectoryPoints[1][i] = goal->trajectory.points.at(i).positions.at(1);
    trajectoryPoints[2][i] = goal->trajectory.points.at(i).positions.at(2);
    trajectoryPoints[3][i] = goal->trajectory.points.at(i).positions.at(3);
    trajectoryPoints[4][i] = goal->trajectory.points.at(i).positions.at(4);
    trajectoryPoints[5][i] = goal->trajectory.points.at(i).positions.at(5);*/
    /*for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
    {
      trajectoryPoints[j][i] = goal->trajectory.points.at(i).positions.at(j);
    }*/
  }

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
  double current_joint_pos[NUM_JACO_JOINTS];
  AngularPosition position_data;
//  TrajectoryPoint trajPoint;
  AngularInfo trajPoint;
  trajPoint.InitStruct();
  //trajPoint.Position.Type = ANGULAR_VELOCITY;
  //trajPoint.Position.HandMode = HAND_NOMOVEMENT;

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
      {
        //boost::recursive_mutex::scoped_lock lock(api_mutex);
        //GetAngularPosition(position_data);
	arm_comm_.getJointAngles(current_joint_angles);
      }
      current_joint_pos[0] = current_joint_angles.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = current_joint_angles.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = current_joint_angles.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = current_joint_angles.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = current_joint_angles.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = current_joint_angles.Actuator6 * DEG_TO_RAD;
      /*current_joint_pos[0] = (180 - current_joint_angles.Actuator1) * DEG_TO_RAD;
      current_joint_pos[1] = (current_joint_angles.Actuator2 - j6o_) * DEG_TO_RAD;
      current_joint_pos[2] = (90 - current_joint_angles.Actuator3) * DEG_TO_RAD;
      current_joint_pos[3] = (180 - current_joint_angles.Actuator4) * DEG_TO_RAD;
      current_joint_pos[4] = (180 - current_joint_angles.Actuator5) * DEG_TO_RAD;
      current_joint_pos[5] = (270 - current_joint_angles.Actuator6) * DEG_TO_RAD;*/
/*joint_state.position[0] = (180- jaco_angles.joint1) * (PI / 180);
    joint_state.position[1] = (jaco_angles.joint2 - j6o) * (PI / 180);
    joint_state.position[2] = (90-jaco_angles.joint3) * (PI / 180);
    joint_state.position[3] = (180-jaco_angles.joint4) * (PI / 180);
    joint_state.position[4] = (180-jaco_angles.joint5) * (PI / 180);
    joint_state.position[5] = (270-jaco_angles.joint6) * (PI / 180);
    joint_state.position[6] = finger_conv_ratio_ * fingers.Finger1;
    joint_state.position[7] = finger_conv_ratio_ * fingers.Finger2;*/

      totalError = 0;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
	if 
        //currentPoint = simplify_angle(current_joint_pos[i]);
        /*error[i] = nearest_equivalent(simplify_angle((splines.at(i))(timePoints.at(timePoints.size() - 1))),
                                      currentPoint) - currentPoint;*/
	error[i] = (splines.at(i))(timePoints.at(timePoints.size() - 1)) - current_joint_pos[i];
        totalError += fabs(error[i]);
      }

      if (totalError < .06)
      {
        trajPoint.Actuator1 = 0.0;
        trajPoint.Actuator2 = 0.0;
        trajPoint.Actuator3 = 0.0;
        trajPoint.Actuator4 = 0.0;
        trajPoint.Actuator5 = 0.0;
        trajPoint.Actuator6 = 0.0;
        //executeAngularTrajectoryPoint(trajPoint, true);
	arm_comm_.setJointVelocities(trajPoint);
        trajectoryComplete = true;
        ROS_INFO("Trajectory complete!");
        break;
      }
    }
    else
    {
      //calculate error
      {
        //boost::recursive_mutex::scoped_lock lock(api_mutex);
        //GetAngularPosition(position_data);
	arm_comm_.getJointAngles(current_joint_angles);
      }

      current_joint_pos[0] = current_joint_angles.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = current_joint_angles.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = current_joint_angles.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = current_joint_angles.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = current_joint_angles.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = current_joint_angles.Actuator6 * DEG_TO_RAD;
      /*current_joint_pos[0] = (180 - current_joint_angles.Actuator1) * DEG_TO_RAD;
      current_joint_pos[1] = (current_joint_angles.Actuator2 - j6o_) * DEG_TO_RAD;
      current_joint_pos[2] = (90 - current_joint_angles.Actuator3) * DEG_TO_RAD;
      current_joint_pos[3] = (180 - current_joint_angles.Actuator4) * DEG_TO_RAD;
      current_joint_pos[4] = (180 - current_joint_angles.Actuator5) * DEG_TO_RAD;
      current_joint_pos[5] = (270 - current_joint_angles.Actuator6) * DEG_TO_RAD;*/
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        //currentPoint = simplify_angle(current_joint_pos[i]);
        //error[i] = nearest_equivalent(simplify_angle((splines.at(i))(t)), currentPoint) - currentPoint;
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
    //executeAngularTrajectoryPoint(trajPoint, true);
    //AngularInfo target(goal->angles);
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
