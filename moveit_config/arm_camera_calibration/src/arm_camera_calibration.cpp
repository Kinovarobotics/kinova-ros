#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

tf::Transform transform;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}





bool computeMatrix(PointCloud::Ptr target,
                   PointCloud::Ptr world,
                   std::string target_name,
                   std::string world_name,
                   const bool broadcast)
{
    if ((!world_name.empty()) && (!target_name.empty()) &&
            (target->points.size() > 2) && (world->points.size() == target->points.size()))
    {
        Eigen::Matrix4f trMatrix;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;

        svd.estimateRigidTransformation(*target, *world, trMatrix);

        ROS_INFO("Registration completed and Registration Matrix is being broadcasted");

         transform=tf::Transform(tf::Matrix3x3(trMatrix(0, 0), trMatrix(0, 1), trMatrix(0, 2),
                                              trMatrix(1, 0), trMatrix(1, 1), trMatrix(1, 2),
                                              trMatrix(2, 0), trMatrix(2, 1), trMatrix(2, 2)),
                                tf::Vector3(trMatrix(0, 3), trMatrix(1, 3), trMatrix(2, 3)));

        Eigen::Vector3d origin(transform.getOrigin());
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
	std::cout << std::endl << "#################################################" << std::endl; 
	std::cout << std::endl << "########### TRANSFORMATION PARAMETERS ###########" << std::endl; 
	std::cout << std::endl << "#################################################" << std::endl; 
        std::cout << "origin: "<<origin.transpose() << std::endl;
        std::cout << "rpy: " << roll << " " << pitch << " " << yaw << std::endl;


    }

    return true;
}




int main(int argc, char *argv[])
{
    ros::init (argc, argv, "marker_detect");
    ros::NodeHandle n;

    ros::NodeHandle n_priv("~");
    double min_x,min_y,min_z;
    double max_x,max_y,max_z;
    double roll_angle_range, pitch_angle_range, yaw_angle_range;
    double roll_angle_offset, pitch_angle_offset, yaw_angle_offset;
    int number_of_points;

    std::string marker_link;
    std::string end_effector_link;
    std::string camera_link;
    std::string base_link;

    n_priv.param<int>("number_of_points",number_of_points, 6);

    n_priv.param<double>("roll_angle_range",    roll_angle_range,  0.2);
    n_priv.param<double>("roll_angle_offset",   roll_angle_offset,  0.2);
    n_priv.param<double>("pitch_angle_range",   pitch_angle_range, 0.2);
    n_priv.param<double>("pitch_angle_offset",  pitch_angle_offset,  0.2);
    n_priv.param<double>("yaw_angle_range",     yaw_angle_range,   0.2);
    n_priv.param<double>("yaw_angle_offset",    yaw_angle_offset,  0.2);

    n_priv.param<double>("min_x",min_x, 0.2);
    n_priv.param<double>("max_x",max_x, 1.0);

    n_priv.param<double>("min_y",min_y, -0.2);
    n_priv.param<double>("max_y",max_y, -1.0);

    n_priv.param<double>("min_z",min_z, 0.2);
    n_priv.param<double>("max_z",max_z, 1.0);

    n_priv.param<std::string>("base_link",base_link, "base_link");
    n_priv.param<std::string>("camera_link",camera_link, "camera_link");
    n_priv.param<std::string>("end_effector_link",end_effector_link, "end_effector");
    n_priv.param<std::string>("marker_link",marker_link, "ar_marker_4");


    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("arm");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.

    group.setPoseReferenceFrame(base_link);
    //std::cout << group.getPlanningFrame() << std::endl;
    group.setEndEffectorLink(end_effector_link);
  //  min_y=-1*min_y;
   // max_y=-1*max_y;
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group.setWorkspace(min_x,min_y,min_z,max_x,max_y,max_z);
    

    group.setGoalTolerance(0.03);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    // specify that our target will be a random one
    geometry_msgs::PoseStamped random_pose;
    random_pose.header.frame_id=base_link;

    std::cout << "get end effector link:"<< group.getEndEffectorLink()<<std::endl;
    PointCloud::Ptr arm_cloud(new PointCloud);
    PointCloud::Ptr camera_cloud(new PointCloud);

    while(ros::ok() &&
          camera_cloud->points.size()!=number_of_points &&
          arm_cloud->points.size()!=number_of_points)
    {
        ROS_INFO_STREAM("Samples acquired so far: "<<camera_cloud->points.size()<< " out of "<<number_of_points);

	// Keep trying to generate possible end effector pose
        bool success;
        do
        {
            tf::Quaternion quat_tf=tf::createQuaternionFromRPY(roll_angle_offset +RandomFloat(-roll_angle_range,roll_angle_range),
							       pitch_angle_offset+RandomFloat(-pitch_angle_range,pitch_angle_range),
                                                               yaw_angle_offset  +RandomFloat(-yaw_angle_range,yaw_angle_range) );
            geometry_msgs::Quaternion quat_msg;
            tf::quaternionTFToMsg(quat_tf,quat_msg);
            //random_pose=group.getRandomPose();
            random_pose.pose.orientation=quat_msg;
	    
            random_pose.pose.position.x=RandomFloat(min_x,max_x);
            random_pose.pose.position.y=RandomFloat(min_y,max_y);
            random_pose.pose.position.z=RandomFloat(min_z,max_z);

	//    random_pose.pose.position.x=0.55555555;
	 //   random_pose.pose.position.y=-0.433334;
	//   random_pose.pose.position.z=0.3333334;
	    
	     
            group.setPoseTarget(random_pose);
            success = group.plan(my_plan);
            std::cout << random_pose.pose.position << std::endl;
	    std::cout << min_x <<" "<<max_x<<" "<<min_y<<" "<<max_y<<" "<<min_z<<" "<<max_z<<" "<< std::endl;
        }
        while(!success && ros::ok());

        // Move Arm
        if(!group.execute(my_plan))
            ROS_WARN_STREAM("Trajectory did not succeed.");

        //sleep(5.0);


        tf::TransformListener listener;
        // Get some point correspondences
        try
        {
            /////////////////////////////////////
            // Point in the end effector frame //
            /////////////////////////////////////

            tf::StampedTransform marker_end_effector_tf;
            listener.waitForTransform(base_link, end_effector_link, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(base_link, end_effector_link, ros::Time(0), marker_end_effector_tf);

            ///////////////////////////
            // Point in camera frame //
            ///////////////////////////

            tf::StampedTransform marker_camera_tf;
            listener.waitForTransform(camera_link, marker_link, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(camera_link, marker_link, ros::Time(0), marker_camera_tf);

            PointT arm_point;
            arm_point.x=marker_end_effector_tf.getOrigin().x();
            arm_point.y=marker_end_effector_tf.getOrigin().y();
            arm_point.z=marker_end_effector_tf.getOrigin().z();
            arm_cloud->points.push_back(arm_point);

            //            std::cout << "arm_point:" << arm_point  << std::endl;

            PointT camera_point;
            camera_point.x=marker_camera_tf.getOrigin().x();
            camera_point.y=marker_camera_tf.getOrigin().y();
            camera_point.z=marker_camera_tf.getOrigin().z();
            camera_cloud->points.push_back(camera_point);

            //            std::cout << "camera_point:" << camera_point  << std::endl;

        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }



    PointCloud::Ptr source, target;
    Eigen::Matrix4f pairTransform;

    source = arm_cloud;
    target = camera_cloud;

    // Add visualization data
    //showCloudsLeft(source,  target);

    computeMatrix(target,
                      source,
                      camera_link,
                      base_link,
                      true);
    ros::Rate r(100.0);
    while(ros::ok)
    {


            static tf::TransformBroadcaster br;
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                  base_link, camera_link));


	r.sleep();
    }

    return 1;
}


