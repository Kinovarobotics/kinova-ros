#include <kinova_driver/kinova_joint_trajectory_controller.h>
#include <angles/angles.h>
#include <ros/console.h>

using namespace kinova;

JointTrajectoryController::JointTrajectoryController(kinova::KinovaComm &kinova_comm, ros::NodeHandle& n):
    kinova_comm_(kinova_comm),
    nh_(n)
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
    ros::NodeHandle pn("~");
    std::string robot_type;
    nh_.param<std::string>("robot_name",prefix_,"j2n6s300");
    nh_.param<std::string>("robot_type",robot_type,"j2n6s300");
    number_joint_ =robot_type[3] - '0';

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    pub_joint_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("trajectory_controller/state", 1);

    traj_frame_id_ = "world";   
    joint_names_.resize(number_joint_);
    
    //std::cout << "joint names in feedback of trajectory state are: " << std::endl;
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = prefix_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
        std::cout << joint_names_[i] << " ";
    }
    std::cout << std::endl;

    terminate_thread_ = false;

    thread_update_state_ = new boost::thread(boost::bind(&JointTrajectoryController::update_state, this));

    traj_feedback_msg_.joint_names.resize(joint_names_.size());
    traj_feedback_msg_.desired.positions.resize(joint_names_.size());
    traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
    traj_feedback_msg_.actual.positions.resize(joint_names_.size());
    traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
    traj_feedback_msg_.error.positions.resize(joint_names_.size());
    traj_feedback_msg_.error.velocities.resize(joint_names_.size());
    traj_feedback_msg_.joint_names = joint_names_;

    // counter in the timer to publish joint velocity command: pub_joint_vel()
    traj_command_points_index_ = 0;

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

JointTrajectoryController::~JointTrajectoryController()
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
    ROS_WARN("destruction entered!");
    {
        boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
        terminate_thread_ = true;
    }

    sub_command_.shutdown();
    pub_joint_feedback_.shutdown();

    thread_update_state_->join();
    delete thread_update_state_;

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

void JointTrajectoryController::update_state()
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    ros::Rate update_rate(10);
    previous_pub_ = ros::Time::now();
    while (nh_.ok())
    {
        // check if terminate command is sent from main thread
        {
            boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
            if (terminate_thread_)
            {
                break;
            }
        }

        traj_feedback_msg_.header.frame_id = traj_frame_id_;
        traj_feedback_msg_.header.stamp = ros::Time::now();
        KinovaAngles current_joint_angles;
        KinovaAngles current_joint_velocity;
        AngularPosition current_joint_command;

        kinova_comm_.getAngularCommand(current_joint_command);
        kinova_comm_.getJointAngles(current_joint_angles);
        kinova_comm_.getJointVelocities(current_joint_velocity);


        traj_feedback_msg_.desired.velocities[0] = current_joint_command.Actuators.Actuator1 *M_PI/180;
        traj_feedback_msg_.desired.velocities[1] = current_joint_command.Actuators.Actuator2 *M_PI/180;
        traj_feedback_msg_.desired.velocities[2] = current_joint_command.Actuators.Actuator3 *M_PI/180;
        traj_feedback_msg_.desired.velocities[3] = current_joint_command.Actuators.Actuator4 *M_PI/180;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.desired.velocities[4] = current_joint_command.Actuators.Actuator5 *M_PI/180;
            traj_feedback_msg_.desired.velocities[5] = current_joint_command.Actuators.Actuator6 *M_PI/180;
            if (number_joint_==7)
                traj_feedback_msg_.desired.velocities[5] = current_joint_command.Actuators.Actuator7 *M_PI/180;
        }

        traj_feedback_msg_.actual.positions[0] = current_joint_angles.Actuator1 *M_PI/180;
        traj_feedback_msg_.actual.positions[1] = current_joint_angles.Actuator2 *M_PI/180;
        traj_feedback_msg_.actual.positions[2] = current_joint_angles.Actuator3 *M_PI/180;
        traj_feedback_msg_.actual.positions[3] = current_joint_angles.Actuator4 *M_PI/180;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.actual.positions[4] = current_joint_angles.Actuator5 *M_PI/180;
            traj_feedback_msg_.actual.positions[5] = current_joint_angles.Actuator6 *M_PI/180;
            if (number_joint_==7)
                traj_feedback_msg_.actual.positions[5] = current_joint_angles.Actuator7 *M_PI/180;
        }

        traj_feedback_msg_.actual.velocities[0] = current_joint_velocity.Actuator1 *M_PI/180;
        traj_feedback_msg_.actual.velocities[1] = current_joint_velocity.Actuator2 *M_PI/180;
        traj_feedback_msg_.actual.velocities[2] = current_joint_velocity.Actuator3 *M_PI/180;
        traj_feedback_msg_.actual.velocities[3] = current_joint_velocity.Actuator4 *M_PI/180;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.actual.velocities[4] = current_joint_velocity.Actuator5 *M_PI/180;
            traj_feedback_msg_.actual.velocities[5] = current_joint_velocity.Actuator6 *M_PI/180;
            if (number_joint_==7)
                traj_feedback_msg_.actual.velocities[5] = current_joint_velocity.Actuator7 *M_PI/180;
        }

        for (size_t j = 0; j<joint_names_.size(); j++)
        {
            traj_feedback_msg_.error.velocities[j] = traj_feedback_msg_.actual.velocities[j] - traj_feedback_msg_.desired.velocities[j];
        }

        //        ROS_WARN_STREAM("I'm publishing after second: " << (ros::Time::now() - previous_pub_).toSec());
        pub_joint_feedback_.publish(traj_feedback_msg_);
        previous_pub_ = ros::Time::now();
        update_rate.sleep();
    }
    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}
