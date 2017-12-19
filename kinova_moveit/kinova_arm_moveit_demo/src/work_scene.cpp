#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "work_scene");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Define publisher to update work scene
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Define table plane
    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 1.6; // x
    table.dimensions[1] = 0.8; // y
    table.dimensions[2] = 0.03; // z
    // Define table position
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = table.dimensions[0]/2.0 - 0.15;
    table_pose.position.y = table.dimensions[1]/2.0 - 0.08;
    table_pose.position.z =  -table.dimensions[2]/2.0;

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.id = "table";
    collision_objects.header.frame_id = "root";
    collision_objects.primitives.push_back(table);
    collision_objects.primitive_poses.push_back(table_pose);
    collision_objects.operation = collision_objects.ADD;



    // Define A Cylinder
    shape_msgs::SolidPrimitive coca_can;
    coca_can.type = coca_can.CYLINDER;
    coca_can.dimensions.resize(2);
    coca_can.dimensions[0] = 0.13; // height
    coca_can.dimensions[1] = 0.036; // radius
    // Define coca can position
    geometry_msgs::Pose coca_can_pose;
    coca_can_pose.orientation.w = 1.0;
    coca_can_pose.position.x = 0.7;
    coca_can_pose.position.y = 0.0;
    coca_can_pose.position.z = coca_can.dimensions[0]/2;


    // Define attached objects
    moveit_msgs::AttachedCollisionObject attached_objects;
    attached_objects.link_name = "j2n6s300_end_effector";
    attached_objects.object.header.frame_id = "root";
    attached_objects.object.id = "cylinder";
    attached_objects.object.primitives.push_back(coca_can);
    attached_objects.object.primitive_poses.push_back(coca_can_pose);
    attached_objects.object.operation = attached_objects.object.ADD;


    // Add all objects to environment
    ROS_INFO("Adding the all objects to the work scene.");
    moveit_msgs::PlanningScene work_scene;
    work_scene.world.collision_objects.push_back(attached_objects.object);
    work_scene.world.collision_objects.push_back(collision_objects);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);
    ros::WallDuration(1).sleep();


    ros::shutdown();
    return 0;
}
