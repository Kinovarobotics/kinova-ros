#include <gripper_command_action_server.h>

using namespace kinova;




GripperCommandActionController::GripperCommandActionController(ros::NodeHandle &n):
    nh_(n),
    has_active_goal_(false)
{
    action_server_gripper_command_.reset(new GCAS(nh_, "/j2n6s300_gripper/gripper_command", boost::bind(&GripperCommandActionController::goalCBFollow, this, _1), boost::bind(&GripperCommandActionController::cancelCBFollow, this, _1), false));
    action_client_set_finger_.reset(new SFPAC(nh_, "/j2n6s300_driver/fingers_action/finger_positions"));

    ros::NodeHandle pn("~");
    // unit of GripperCommand is meter. Gripper_command_goal from "moveit.rviz" is in unit of radians.
    nh_.param("gripper_command_goal_constraint_", gripper_command_goal_constraint_, 0.01);
    nh_.param("finger_max_turn_", finger_max_turn_, 6400.0);
    nh_.param("finger_conv_ratio_", finger_conv_ratio_, 1.4 / 6400.0);

    gripper_joint_num_ = 3;
    gripper_joint_names_.resize(gripper_joint_num_);

    for (uint i = 0; i<gripper_joint_names_.size(); i++)
    {
        gripper_joint_names_[i] = "j2n6s300_joint_finger_" + boost::lexical_cast<std::string>(i+1);
    }

    sub_fingers_state_ = nh_.subscribe("/j2n6s300_driver/out/finger_position", 1, &GripperCommandActionController::controllerStateCB, this);


    ros::Time started_waiting_for_controller = ros::Time::now();
    while (ros::ok() && !last_finger_state_)
    {
        ros::spinOnce();
        if (started_waiting_for_controller != ros::Time(0) &&
                ros::Time::now() > started_waiting_for_controller + ros::Duration(30.0))
        {
            ROS_WARN("Waited for the kinova driver for 30 seconds, but it never showed up. Continue waiting the finger state on topic /j2n6s300_driver/out/finger_position ...");
            started_waiting_for_controller = ros::Time(0);
        }
        ros::WallDuration(0.1).sleep();
    }


    action_server_gripper_command_->start();
    action_client_set_finger_->waitForActionServerToStart();
    ROS_INFO("Start Gripper_Command_Trajectory_Action server!");

}


GripperCommandActionController::~GripperCommandActionController()
{
    sub_fingers_state_.shutdown();
}


static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
        if (count(b.begin(), b.end(), a[i]) != 1)
            return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
        if (count(a.begin(), a.end(), b[i]) != 1)
            return false;
    }

    return true;
}


void GripperCommandActionController::goalCBFollow(GCAS::GoalHandle gh)
{
    ROS_INFO("Gripper_command_action_server received goal!");

    // Cancels the currently active goal.
    if (has_active_goal_)
    {
        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        ROS_INFO("A new gripper command goal is comming, so the previous one is cancelled.");
        has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    // gripper_gap 0 is open, 1.4 is close. Here command.position from Moveit.rviz is finger radian value, rather than Cartesian gap of gripper in meter.
    double gripper_gap = active_goal_.getGoal()->command.position;
    ROS_INFO("Gripper_command_action_server accepted goal!");


    // send the goal to fingers position action server
    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = std::min( finger_max_turn_ , gripper_gap/finger_conv_ratio_);
    goal.fingers.finger2 = std::min( finger_max_turn_ , gripper_gap/finger_conv_ratio_);
    if (gripper_joint_num_ == 3)
        goal.fingers.finger3 = std::min( finger_max_turn_ , gripper_gap/finger_conv_ratio_);
    else
        goal.fingers.finger3 = 0.0;

    if(action_client_set_finger_->isServerConnected())
        action_client_set_finger_->sendGoal(goal);

    ROS_INFO("Gripper_command_action_server published goal via command publisher!");
}


void GripperCommandActionController::cancelCBFollow(GCAS::GoalHandle gh)
{
    if (active_goal_ == gh)
    {
        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }
}


void GripperCommandActionController::controllerStateCB(const kinova_msgs::FingerPositionConstPtr &msg)
{
    ROS_INFO_ONCE("Gripper_command__action_server receive feedback of trajectory state from topic: /trajectory_controller/state");

    last_finger_state_ = msg;

    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
        return;

    // Checks that we have ended inside the goal constraints, FingerPosition has 3 values
    double abs_error1 = fabs(active_goal_.getGoal()->command.position - msg->finger1 * finger_conv_ratio_);
    double abs_error2 = fabs(active_goal_.getGoal()->command.position - msg->finger2 * finger_conv_ratio_);
    double abs_error3 = fabs(active_goal_.getGoal()->command.position - msg->finger3 * finger_conv_ratio_);

    if (abs_error1<gripper_command_goal_constraint_  && abs_error2<gripper_command_goal_constraint_ && abs_error3<gripper_command_goal_constraint_)
    {
        ROS_INFO("Gripper command goal succeeded!");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_command_action_server");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::GripperCommandActionController gcac(node);

    ros::spin();
    return 0;
}

