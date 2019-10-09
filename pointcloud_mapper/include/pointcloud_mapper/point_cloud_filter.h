/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

class PointCloudFilter {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  PointCloudFilter();
  ~PointCloudFilter();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Filter an incoming point cloud
  bool Filter(const PointCloud::ConstPtr& points,
              PointCloud::Ptr points_filtered) const;

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // The node's name.
  std::string name_;

  struct Parameters {
    // Apply a voxel grid filter.
    bool grid_filter;

    // Resolution of voxel grid filter.
    double grid_res;

    // Apply a random downsampling filter.
    bool random_filter;

    // Percentage of points to discard. Must be between 0.0 and 1.0;
    double decimate_percentage;

    // Apply a statistical outlier filter.
    bool outlier_filter;

    // Standard deviation threshold in distance to neighbors for outlier
    // removal.
    double outlier_std;

    // Number of nearest neighbors to use for outlier filter.
    int outlier_knn;

    // Apply a radius outlier filter.
    bool radius_filter;

    // Size of the radius filter.
    double radius;

    // If this number of neighbors are not found within a radius around each
    // point, remove that point.
    int radius_knn;
  } params_;
};

#endif // POINT_CLOUD_FILTER_H
