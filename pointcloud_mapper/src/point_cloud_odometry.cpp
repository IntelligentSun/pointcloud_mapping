/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#include "pointcloud_mapper/point_cloud_odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/registration/gicp.h>

using pcl::copyPointCloud;
using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;

PointCloudOdometry::PointCloudOdometry() : initialized_(false) {
  query_.reset(new PointCloud);
  reference_.reset(new PointCloud);
}

PointCloudOdometry::~PointCloudOdometry() {}

bool PointCloudOdometry::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudOdometry");

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

bool PointCloudOdometry::LoadParameters(const ros::NodeHandle& n) {

  // Load frame ids.
  n.param<std::string>("frame_id/fixed", fixed_frame_id_,"world");
  n.param<std::string>("frame_id/odometry", odometry_frame_id_, "odom");

  // Load initial position.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  n.getParam("init/position/x", init_x);
  n.getParam("init/position/y", init_y);
  n.getParam("init/position/z", init_z);
  n.getParam("init/orientation/roll", init_roll);
  n.getParam("init/orientation/pitch", init_pitch);
  n.getParam("init/orientation/yaw", init_yaw);

  geometry_utils::Transform3 init;
  init.translation = geometry_utils::Vec3(init_x, init_y, init_z);
  init.rotation = geometry_utils::Rot3(init_roll, init_pitch, init_yaw);
  integrated_estimate_ = init;

  // Load algorithm parameters.
  // Stop ICP if the transformation from the last iteration was this small.
  n.param<double>("icp/tf_epsilon", params_.icp_tf_epsilon,0.0000000001);
  // During ICP, two points won't be considered a correspondence if they are at
  // least this far from one another.
  n.param<double>("icp/corr_dist", params_.icp_corr_dist, 2.0);
  // Iterate ICP this many times.
  n.param<int>("icp/iterations", params_.icp_iterations, 10);

  // Maximum acceptable incremental rotation and translation.
  n.param<bool>("icp/transform_thresholding", transform_thresholding_, false);
  n.param<double>("icp/max_translation", max_translation_, 0.09);
  n.param<double>("icp/max_rotation", max_rotation_, 0.1);

  return true;
}

bool PointCloudOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  query_pub_ = nl.advertise<PointCloud>("odometry_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloud>("odometry_reference_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_integrated_estimate", 10, false);

  return true;
}

bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {
  // Store input point cloud's time stamp for publishing.
  stamp_.fromNSec(points.header.stamp*1e3);

  // If this is the first point cloud, store it and wait for another.
  if (!initialized_) {
    copyPointCloud(points, *query_);
    initialized_ = true;
    return false;
  }

  // Move current query points (acquired last iteration) to reference points.
  copyPointCloud(*query_, *reference_);

  // Set the incoming point cloud as the query point cloud.
  copyPointCloud(points, *query_);
#if 1
  // Update pose estimate via ICP.
  return UpdateICP();
#else
  return UpdateNDT();
#endif
}

const geometry_utils::Transform3& PointCloudOdometry::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const geometry_utils::Transform3& PointCloudOdometry::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

bool PointCloudOdometry::GetLastPointCloud(PointCloud::Ptr& out) const {
  if (!initialized_ || query_ == NULL) {
    ROS_WARN("%s: Not initialized.", name_.c_str());
    return false;
  }

  out = query_;
  return true;
}

bool PointCloudOdometry::UpdateNDT()
{
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (50);

  // Setting point cloud to be aligned.
  ndt.setInputSource (query_);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (reference_);

  /// Set initial alignment estimate found using robot odometry.
  /// "/odom"
  Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ()); // TODO
  Eigen::Translation3f init_translation (0.0, 0.0, 0);  // TODO
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  auto T = Eigen::Matrix4f();
  T <<  0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
  if(ndt.getFitnessScore() < 0.1) {
    // Retrieve transformation and estimate and update.
    T = ndt.getFinalTransformation();
  }

  // Update pose estimates.
  incremental_estimate_.translation = geometry_utils::Vec3(T(0, 3), T(1, 3), T(2, 3));
  incremental_estimate_.rotation = geometry_utils::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));

  // Only update if the incremental transform is small enough.
  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        geometry_utils::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
    ROS_WARN(
        "%s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
        name_.c_str(), incremental_estimate_.translation.Norm(),
        incremental_estimate_.rotation.ToEulerZYX().Norm());
  }

  // Convert pose estimates to ROS format and publish.
  PublishPose(incremental_estimate_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, integrated_estimate_pub_);

  // Publish point clouds for visualization.
  PublishPoints(query_, query_pub_);
  PublishPoints(reference_, reference_pub_);

  // Convert transform between fixed frame and odometry frame.
  geometry_msgs::TransformStamped tf;
  tf.transform = geometry_utils::ros::ToRosTransform(integrated_estimate_);
  tf.header.stamp = stamp_;
  tf.header.frame_id = fixed_frame_id_;
  tf.child_frame_id = odometry_frame_id_;
  tfbr_.sendTransform(tf);

  return true;
}

bool PointCloudOdometry::UpdateICP() {
  // Compute the incremental transformation.
  GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icp;
  icp.setTransformationEpsilon(params_.icp_tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.icp_corr_dist);
  icp.setMaximumIterations(params_.icp_iterations);
  icp.setRANSACIterations(0);

  icp.setInputSource(query_);
  icp.setInputTarget(reference_);

  PointCloud unused_result;
  icp.align(unused_result);

  const Eigen::Matrix4f T = icp.getFinalTransformation();

  // Update pose estimates.
  incremental_estimate_.translation = geometry_utils::Vec3(T(0, 3), T(1, 3), T(2, 3));
  incremental_estimate_.rotation = geometry_utils::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));

  // Only update if the incremental transform is small enough.
  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        geometry_utils::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
    ROS_WARN(
        "%s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
        name_.c_str(), incremental_estimate_.translation.Norm(),
        incremental_estimate_.rotation.ToEulerZYX().Norm());
  }

  // Convert pose estimates to ROS format and publish.
  PublishPose(incremental_estimate_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, integrated_estimate_pub_);

  // Publish point clouds for visualization.
  PublishPoints(query_, query_pub_);
  PublishPoints(reference_, reference_pub_);

  // Convert transform between fixed frame and odometry frame.
  geometry_msgs::TransformStamped tf;
  tf.transform = geometry_utils::ros::ToRosTransform(integrated_estimate_);
  tf.header.stamp = stamp_;
  tf.header.frame_id = fixed_frame_id_;
  tf.child_frame_id = odometry_frame_id_;
  tfbr_.sendTransform(tf);

  return true;
}

void PointCloudOdometry::PublishPoints(const PointCloud::Ptr& points,
                                       const ros::Publisher& pub) {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() > 0) {
    PointCloud out;
    out = *points;
    out.header.frame_id = odometry_frame_id_;
    pub.publish(out);
  }
}

void PointCloudOdometry::PublishPose(const geometry_utils::Transform3& pose,
                                     const ros::Publisher& pub) {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() == 0)
    return;

  // Convert from geometry_utils::Transform3 to ROS's PoseStamped type and publish.
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = geometry_utils::ros::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;
  pub.publish(ros_pose);
}
