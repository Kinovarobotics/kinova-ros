#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <kinova/KinovaTypes.h>
#include "kinova_driver/kinova_ros_types.h"
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"

#include <actionlib/client/simple_action_client.h>

#include <algorithm>
#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;

// Create actionlib client
typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ArmJoint_actionlibClient;
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ArmPose_actionlibClient;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> armPose_interMark_server;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> armJoint_interMark_server;
interactive_markers::MenuHandler menu_handler;
sensor_msgs::JointState current_joint_state;
std::string tf_prefix_;
std::string kinova_robotType_;
char robot_category_;
int robot_category_version_;
char wrist_type_;
int arm_joint_number_;
char robot_mode_;
int finger_number_;
int joint_total_number_;

bool mouse_was_up = true;
// %EndTag(vars)%

// declare processFeedback(), action when interative marker is clicked.
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.25;
    marker.scale.y = msg.scale * 0.25;
    marker.scale.z = msg.scale * 0.25;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(1DOF)%
void make1DofMarker(const std::string& frame_id, const std::string& axis, unsigned int interaction_mode, const tf::Vector3& position, const std::string& description, const std::string& name)
{

    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.08;
    int_marker.name = name;
    int_marker.description = description;

    // insert a box
    makeBoxControl(int_marker);
    InteractiveMarkerControl control;

    if (interaction_mode == InteractiveMarkerControl::ROTATE_AXIS)
        control.name = "rotate";
    else if (interaction_mode == InteractiveMarkerControl::MOVE_AXIS)
        control.name = "move";
    else
        ROS_INFO("interactive mode should be eigher ROTATE_AXIS or MOVE_AXIS");

    if (axis == "x")
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name += "_x";
    }
    else if (axis == "y")
    {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name += "_y";
    }
    else if (axis == "z")
    {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name += "_z";
    }
    else
        ROS_INFO("\n The rotation axis must be x, y or z. \n");

    control.interaction_mode = interaction_mode;
    int_marker.controls.push_back(control);

    armJoint_interMark_server->insert(int_marker);
//    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback);
    //  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    //    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%position

void sendArmJointGoal(const std::string marker_name, double joint_offset)
{
    ArmJoint_actionlibClient client("/"+tf_prefix_+"_driver/joints_action/joint_angles", true);
    kinova_msgs::ArmJointAnglesGoal goal;

    goal.angles.joint1 = current_joint_state.position[0]*180/M_PI;
    goal.angles.joint2 = current_joint_state.position[1]*180/M_PI;
    goal.angles.joint3 = current_joint_state.position[2]*180/M_PI;
    goal.angles.joint4 = current_joint_state.position[3]*180/M_PI;
    if(arm_joint_number_==4)
    {
        goal.angles.joint5 = 0.0;
        goal.angles.joint6 = 0.0;
    }
    else if(arm_joint_number_==6)
    {
        goal.angles.joint5 = current_joint_state.position[4]*180/M_PI;
        goal.angles.joint6 = current_joint_state.position[5]*180/M_PI;
    }

    client.waitForServer();

    // map marker position to the position of arm joint.
    switch(*marker_name.rbegin()-'0')
    {
    case 1:
        goal.angles.joint1 += joint_offset;
        break;
    case 2:
        goal.angles.joint2 += joint_offset;
        break;
    case 3:
        goal.angles.joint3 += joint_offset;
        break;
    case 4:
        goal.angles.joint4 += joint_offset;
        break;
    case 5:
        if(arm_joint_number_==6)
        {
            goal.angles.joint5 += joint_offset;
        }
        break;
    case 6:
        if(arm_joint_number_==6)
        {
            goal.angles.joint6 += joint_offset;
        }
        break;
    }

    if (arm_joint_number_==6)
    {
        ROS_INFO( " current joint is as : %f, %f, %f, %f, %f, %f (degree)\n",  current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5]);
    }
    else if(arm_joint_number_==4)
    {
        ROS_INFO( " current joint is as : %f, %f, %f, %f (degree)\n",  current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3]);
    }
    ROS_INFO( " joint goal is set as : %f, %f, %f, %f, %f, %f (degree)\n",  goal.angles.joint1, goal.angles.joint2, goal.angles.joint3, goal.angles.joint4, goal.angles.joint5, goal.angles.joint6);

    client.sendGoal(goal);

}


// %EndTag(send actionlib goals)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = tf_prefix_+"_link_base";
    tf::Quaternion quaternion_mousedown, quaternion_mouseup;

    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    double roll_mousedown, pitch_mousedown, yaw_mousedown;
    double roll_mouseup, pitch_mouseup, yaw_mouseup;
    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        // the moment mouse is clicked down, not the moment keeps down.
        if(mouse_was_up=true)
        {
            tf::quaternionMsgToTF(feedback->pose.orientation, quaternion_mousedown);
            tf::Matrix3x3(quaternion_mousedown).getRPY(roll_mousedown, pitch_mousedown, yaw_mousedown);
            if (feedback->marker_name == "cartesian_6dof")
            {
                ROS_INFO_STREAM("cartesian_6dof control mode.");
                ROS_INFO_STREAM( s.str() << ": mouse DOWN (refers to current status): "
                                 << "\nposition = "
                                 << feedback->pose.position.x
                                 << ", " << feedback->pose.position.y
                                 << ", " << feedback->pose.position.z
                                 << "\norientation = "
                                 << "w: " << feedback->pose.orientation.w
                                 << ", x: " << feedback->pose.orientation.x
                                 << ", y: " << feedback->pose.orientation.y
                                 << ", z: " << feedback->pose.orientation.z
                                 << "\nRollPitchYaw = "
                                 << ": Roll: " << roll_mousedown
                                 << ", Pitch: " << pitch_mousedown
                                 << ", Yaw: " << yaw_mousedown
                                 << "\nframe: " << feedback->header.frame_id
                                 << " time: " << feedback->header.stamp.sec << "sec, "
                                 << feedback->header.stamp.nsec << " nsec" );
            }
            else
            {
                ROS_INFO_STREAM("Joint control is mode.");
                ROS_INFO_STREAM( s.str() << ": mouse DOWN (refers to current status): "
                                 << feedback->marker_name
                                 << ": " << yaw_mousedown
                                 << "\nframe: " << feedback->header.frame_id
                                 << " time: " << feedback->header.stamp.sec << "sec, "
                                 << feedback->header.stamp.nsec << " nsec" );
            }
        }

        mouse_was_up = false;
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        tf::quaternionMsgToTF(feedback->pose.orientation, quaternion_mouseup);
        tf::Matrix3x3(quaternion_mouseup).getRPY(roll_mouseup, pitch_mouseup, yaw_mouseup);
        if (feedback->marker_name == "cartesian_6dof")
        {
            tf::TransformListener transform_listener;
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped pose_endeffector;
            pose_endeffector.header.frame_id = tf_prefix_+"_link_base";
            pose_endeffector.header.stamp = ros::Time::now();

            try{
                ros::Time now = ros::Time::now();
                transform_listener.waitForTransform(tf_prefix_+"_link_base",tf_prefix_+"_end_effector", now, ros::Duration(3.0));
                transform_listener.lookupTransform(tf_prefix_+"_link_base",tf_prefix_+"_end_effector", ros::Time(0), transform);
            }
            catch(tf::TransformException exception)
            {
                ROS_ERROR("Error occured during transformation from end_effector to link_base: %s", exception.what());
                ros::Duration(1.0).sleep();
            }
            pose_endeffector.pose.position.x = transform.getOrigin().x();
            pose_endeffector.pose.position.y = transform.getOrigin().y();
            pose_endeffector.pose.position.z = transform.getOrigin().z();
            tf::quaternionTFToMsg(transform.getRotation().normalize(), pose_endeffector.pose.orientation);

            ROS_INFO_STREAM( s.str() << ": mouse UP (refers to command): "
                             << "\nposition = "
                             << feedback->pose.position.x
                             << ", " << feedback->pose.position.y
                             << ", " << feedback->pose.position.z
                             << "\norientation = "
                             << "w: " << feedback->pose.orientation.w
                             << ", x: " << feedback->pose.orientation.x
                             << ", y: " << feedback->pose.orientation.y
                             << ", z: " << feedback->pose.orientation.z
                             << "\nRollPitchYaw = "
                             << ": Roll: " << roll_mouseup
                             << ", Pitch: " << pitch_mouseup
                             << ", Yaw: " << yaw_mouseup
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec" );

            ROS_INFO_STREAM("POSE_endeffector parent frame is : " << pose_endeffector.header.frame_id);
//            sendArmPoseGoal(pose_endeffector);
            // armPose_interMark_server->applyChanges();
        }
        else
        {
            ROS_INFO_STREAM( s.str() << ": mouse UP (refers to command): "
                             << feedback->marker_name
                             << ": " << yaw_mouseup
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec");
            sendArmJointGoal(feedback->marker_name, (yaw_mouseup-yaw_mousedown)*180/M_PI);
            //   armJoint_interMark_server->applyChanges();
        }
        //reset flag for MOUSE_DOWN
        mouse_was_up = true;

        // sendFingerGoal(feedback);


        break;
    }


}
// %EndTag(processFeedback)%


// %Tag(CurrentJoint)%
void currentJointsFeedback(const sensor_msgs::JointStateConstPtr joint_state)
{
    std::vector<std::string> joint_names_;
    joint_names_.resize(joint_total_number_);

    joint_names_[0] =  tf_prefix_+"_joint_1";
    joint_names_[1] =  tf_prefix_+"_joint_2";
    joint_names_[2] =  tf_prefix_+"_joint_3";
    joint_names_[3] =  tf_prefix_+"_joint_4";
    if(arm_joint_number_==6)
    {
        joint_names_[4] =  tf_prefix_+"_joint_5";
        joint_names_[5] =  tf_prefix_+"_joint_6";
    }

    if (finger_number_ ==2)
    {
        joint_names_[joint_total_number_-2] =  tf_prefix_+"_joint_finger_1";
        joint_names_[joint_total_number_-1] =  tf_prefix_+"_joint_finger_2";
    }
    else if(finger_number_ ==3)
    {
        joint_names_[joint_total_number_-3] =  tf_prefix_+"_joint_finger_1";
        joint_names_[joint_total_number_-2] =  tf_prefix_+"_joint_finger_2";
        joint_names_[joint_total_number_-1] =  tf_prefix_+"_joint_finger_3";
    }
    current_joint_state.name = joint_names_;

    current_joint_state.header.stamp = ros::Time::now();
    current_joint_state.position.resize(joint_total_number_);
    current_joint_state.position = joint_state->position;
    current_joint_state.velocity = joint_state->velocity;
    current_joint_state.effort = joint_state->effort;

}
// %EndTag(CurrentJoint)%

////////////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
///
int main(int argc, char** argv)
{
    // Retrieve the (non-option) argument:
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robotType provided in the argument!" << std::endl;
        return -1;
    }
    else // there is an input...
    {
//        if (valid_kinovaRobotType(kinova_robotType_) == false)
//        {
//            ROS_WARN("Invalid kinova_robotType error! Obtained: %s.", kinova_robotType_.c_str());
//            return -1;
//        }
        kinova_robotType_ = argv[argc-1];
        ROS_INFO("kinova_robotType is %s.", kinova_robotType_.c_str());
    }

//    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_;

    // Maximum number of joints on Kinova-like robots:
    robot_category_ = kinova_robotType_[0];
    robot_category_version_ = kinova_robotType_[1]-'0';
    wrist_type_ = kinova_robotType_[2];
    arm_joint_number_ = kinova_robotType_[3]-'0';
    robot_mode_ = kinova_robotType_[4];
    finger_number_ = kinova_robotType_[5]-'0';
    joint_total_number_ = arm_joint_number_ + finger_number_;


    ros::init(argc, argv, tf_prefix_+"_interactive_control");
    ros::NodeHandle nh("~");

    ros::Subscriber armJoint_sub = nh.subscribe("/"+tf_prefix_+"_driver/out/joint_state", 1, &currentJointsFeedback);

    armJoint_interMark_server.reset( new interactive_markers::InteractiveMarkerServer(tf_prefix_+"_interactive_control_Joint","",false) );

    ros::Duration(0.1).sleep();


    // %Tag(CreatInteractiveMarkers)%
    tf::Vector3 position;
//    position = tf::Vector3(0, 0, 0); // Simple 6-DOF Control
//    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );

    position = tf::Vector3(0, 0, 0);
    make1DofMarker(tf_prefix_+"_link_1", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "1st Axis", "marker_joint1");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker(tf_prefix_+"_link_2", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "2nd Axis", "marker_joint2");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker(tf_prefix_+"_link_3", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "3rd Axis", "marker_joint3");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker(tf_prefix_+"_link_4", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "4th Axis", "marker_joint4");
    if(arm_joint_number_==6)
    {
        position = tf::Vector3(0, 0, 0);
        make1DofMarker(tf_prefix_+"_link_5", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "5th Axis", "marker_joint5");    position = tf::Vector3(0, 0, 0);
        make1DofMarker(tf_prefix_+"_link_hand", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "6th Axis", "marker_joint6");
    }
    // %EndTag(CreatInteractiveMarkers)%

    armJoint_interMark_server->applyChanges();

    ros::spin();

    armJoint_interMark_server.reset();

    return 0;

}


