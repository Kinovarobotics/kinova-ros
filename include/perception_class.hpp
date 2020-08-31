// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.hpp
//
// Defines a Perception class to handle visualization for operating a robotic manipulator
// utilizing an Intel Realsense D435i sensor 
//
// ********************************************************************************************

#ifndef PERCEPTION_CLASS_HPP
#define PERCEPTION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <cstring>
#include <ctime>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>				// saving pointclouds
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

using namespace pcl;
using namespace std;
using namespace tf;

class Perception
{
  private:
  
    //Publishers & Subscribers
    ros::Publisher combined_cloud_pub;
    ros::Subscriber wrist_camera_sub;

    //Functions
    void wrist_camera_callback(const sensor_msgs::PointCloud2 msg);
    PointCloud<PointXYZRGB> voxelgrid_filter(PointCloud<PointXYZRGB>::Ptr cloud);
    PointCloud<PointXYZRGB> sac_segmentation(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers_plane);
    PointCloud<PointNormal> move_least_squares(PointCloud<PointXYZRGB>::Ptr cloud);
    void computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals);
    void extractNormals(PointCloud<Normal>::Ptr cloud_normals, PointIndices::Ptr inliers_plane);

  public:

    //Constructor

    //Members
    PointCloud<PointXYZRGB> combined_cloud;
    PointCloud<PointXYZRGB> current_cloud;
    PointCloud<PointXYZRGB> left_cloud;
    PointCloud<PointXYZRGB> right_cloud;
    PointCloud<PointXYZRGB> top_cloud;
    PointCloud<PointXYZRGB> front_cloud;
    TransformListenerPtr transform_listener_ptr;

    bool points_not_found;

    //Functions
    void init_subscriber(ros::NodeHandle nodeHandle);
    Perception(ros::NodeHandle nodeHandle);
    void publish_combined_cloud();
    void concatenate_clouds();
    void take_snapshot_left();
    void take_snapshot_right();
    void take_snapshot_top();
    void take_snapshot_front();

};  

#endif // PERCEPTION_CLASS
