// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.cpp
//
// perception_class function
// ********************************************************************************************

#include "perception_class.hpp"

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// When you create functions, this is where you will put them
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// WRIST CAMERA CALLBACK FUNCTION
// initialize cloud right with the information from wrist_camera
// converts pointcloud from sensor_msgs::PointCloud2 to pcl::PointXYZRGB
void Perception::wrist_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->current_cloud);
 
    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->current_cloud.header.stamp);
    this->transform_listener_ptr->waitForTransform("/world", this->current_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->current_cloud, this->current_cloud, *this->transform_listener_ptr);
}

// ********************************************************************************************
// Public Functions
// ********************************************************************************************

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate "constructor" for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::init_subscriber(ros::NodeHandle nodeHandle)
{
  this->wrist_camera_sub = nodeHandle.subscribe("/pcl_filters/wrist_camera_xyz_filter/output", 1, &Perception::wrist_camera_callback, this);
}

// PERCEPTION CLASS CONSTRUCTOR
// Create instance of Perception class and instantiate publisher for combined cloud 
// and subscriber for pointcloud from wrist camera sensor
Perception::Perception(ros::NodeHandle handle)
{
  this->combined_cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
  this->points_not_found = true;
}

// PUBLISH COMBINED CLOUD FUNCTION
// Publish combined (concatenated) point cloud
// Convert combined_cloud (pcl::PointXYZRGB) to sensor_msgs::PointCloud2
// and then publish
void Perception::publish_combined_cloud()
{

  sensor_msgs::PointCloud2 cloud;
  toROSMsg(this->combined_cloud, cloud);

  this->combined_cloud_pub.publish(cloud);
}

// take_snapshot_ function list
// --------------------------------
// take snapshots (save the current_cloud into different member variables) of the pointcloud in 
// different robot positions in order to get a more complete pointcloud after concatenation

// TAKE SNAPSHOT LEFT FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_left()
{
  this->left_cloud = this->current_cloud;
}

// TAKE SNAPSHOT RIGHT FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_right()
{
  this->right_cloud = this->current_cloud;
}

// TAKE SNAPSHOT TOP FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_top()
{
  this->top_cloud = this->current_cloud;
}

// TAKE SNAPSHOT FRONT FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_front()
{
  this->front_cloud = this->current_cloud;
}

// CONCATENATE CLOUDS FUNCTION
// concatenates the points of all the cloud members into the combined pointcloud
// then performs downsampling (voxel_filter) and noise reduction (move_least_squares)
// to clean up resulting published pointcloud
void Perception::concatenate_clouds() 
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    
  *temp_cloud = this->left_cloud;
  *temp_cloud+= this->right_cloud;
  *temp_cloud+= this->top_cloud;

  // Uncomment to save concatenated pointcloud if desired
  // pcl::io::savePCDFileASCII("single_workstation_object_sample.pcd", *temp_cloud);

  // Passthrough filter to limit to work area
  PassThrough<PointXYZRGB> pass_w;
  pass_w.setInputCloud (temp_cloud);
  pass_w.setFilterFieldName ("x");
  pass_w.setFilterLimits (-0.9, -0.3);
  pass_w.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_y;
  pass_y.setInputCloud (temp_cloud);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.45, 0.45);
  pass_y.filter(*temp_cloud);
  
  PassThrough<PointXYZRGB> pass_z;
  pass_z.setInputCloud (temp_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (-0.02, 0.3);
  pass_z.filter(*temp_cloud);

  this->combined_cloud = *temp_cloud;
}

// Generic PCL Filters

// COMPUTE NORMALS FUNCTION
// TODO: add description
void Perception::computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

// EXTRACT NORMALS FUNCTION
// TODO: add description
void Perception::extractNormals(PointCloud<Normal>::Ptr cloud_normals, PointIndices::Ptr inliers_plane)
{
  ExtractIndices<Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

// VOXEL FILTER FUNCTION
// downsamples point cloud to make the resulting model cleaner
PointCloud<PointXYZRGB> Perception::voxelgrid_filter(PointCloud<PointXYZRGB>::Ptr cloud)
{
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);

  VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01, 0.01, 0.01); 
  sor.filter(*filtered_cloud);

  return *filtered_cloud;
}

// SAC (PLANAR) SEGMENTATION FUNCTION
// remove largest planar surface from pointcloud
PointCloud<PointXYZRGB> Perception::sac_segmentation(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers_plane)
{
  // Create the segmentation object
  SACSegmentation<PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);
  ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
  seg.segment (*inliers_plane, *coefficients_plane);

  // Create the filtering object
  ExtractIndices<PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);

  return *cloud;
}

// MOVE LEAST SQUARES FUNCTION
// aligns the surface normals to eliminate noise
PointCloud<PointNormal> Perception::move_least_squares(PointCloud<PointXYZRGB>::Ptr cloud)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
  PointCloud<PointNormal> mls_points;
  MovingLeastSquares<PointXYZRGB, PointNormal> mls;
 
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(4);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.1);
  mls.process(mls_points);

  return mls_points;
}
