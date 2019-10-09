/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "pointcloud_mapper/point_cloud_filter.h"

PointCloudFilter::PointCloudFilter() {}
PointCloudFilter::~PointCloudFilter() {}

bool PointCloudFilter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudFilter");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudFilter::LoadParameters(const ros::NodeHandle& n) {
  // Load filtering parameters.
  // Apply a voxel grid filter on incoming point clouds.
  n.param<bool>("filtering/grid_filter", params_.grid_filter, false);
  // Resolution of voxel grid filter.
  n.param<double>("filtering/grid_res", params_.grid_res, 0.2);

  // Apply a random downsample filter on incoming point clouds.
  n.param<bool>("filtering/random_filter", params_.random_filter, true);
  // Percentage of points to discard. Must be between 0.0 and 1.0.
  n.param<double>("filtering/decimate_percentage", params_.decimate_percentage, 0.9);

  // Apply a statistical outlier filter on incoming point clouds.
  n.param<bool>("filtering/outlier_filter", params_.outlier_filter, false);
  // Standard deviation threshold in distance to neighbors for outlier removal.
  n.param<double>("filtering/outlier_std", params_.outlier_std, 1.0);
  // Number of nearest neighbors for outlier removal filter.
  n.param<int>("filtering/outlier_knn", params_.outlier_knn, 10);

  // Apply a radius filter on incoming point clouds.
  n.param<bool>("filtering/radius_filter", params_.radius_filter, false);
  // Size of the radius outlier filter.
  n.param<double>("filtering/radius", params_.radius, 0.15);
  // If this number of neighbors are not found within a radius of a point, remove
  // the point.
  n.param<int>("filtering/radius_knn", params_.radius_knn, 3);

  // Cap to [0.0, 1.0].
  params_.decimate_percentage =
      std::min(1.0, std::max(0.0, params_.decimate_percentage));

  return true;
}

bool PointCloudFilter::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  return true;
}

bool PointCloudFilter::Filter(const PointCloud::ConstPtr& points,
                              PointCloud::Ptr points_filtered) const {
  if (points_filtered == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Copy input points.
  *points_filtered = *points;

  // Apply a random downsampling filter to the incoming point cloud.
  if (params_.random_filter) {
    const int n_points = static_cast<int>((1.0 - params_.decimate_percentage) *
                                          points_filtered->size());
    pcl::RandomSample<pcl::PointXYZ> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(points_filtered);
    random_filter.filter(*points_filtered);
  }

  // Apply a voxel grid filter to the incoming point cloud.
  if (params_.grid_filter) {
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(params_.grid_res, params_.grid_res, params_.grid_res);
    grid.setInputCloud(points_filtered);
    grid.filter(*points_filtered);
  }

  // Remove statistical outliers in incoming the point cloud.
  if (params_.outlier_filter) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(points_filtered);
    sor.setMeanK(params_.outlier_knn);
    sor.setStddevMulThresh(params_.outlier_std);
    sor.filter(*points_filtered);
  }

  // Remove points without a threshold number of neighbors within a specified
  // radius.
  if (params_.radius_filter) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> rad;
    rad.setInputCloud(points_filtered);
    rad.setRadiusSearch(params_.radius);
    rad.setMinNeighborsInRadius(params_.radius_knn);
    rad.filter(*points_filtered);
  }

  return true;
}
