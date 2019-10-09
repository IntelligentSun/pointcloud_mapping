/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/registration/gicp.h>

#include "pointcloud_mapper/point_cloud_localization.h"

namespace gu = geometry_utils;
namespace gr = gu::ros;

using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::transformPointCloud;

PointCloudLocalization::PointCloudLocalization() {}
PointCloudLocalization::~PointCloudLocalization() {}

bool PointCloudLocalization::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudLocalization");

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

bool PointCloudLocalization::LoadParameters(const ros::NodeHandle& n) {
  // Load frame ids.
  n.param<std::string>("frame_id/fixed", fixed_frame_id_,"world");
  n.param<std::string>("frame_id/base", base_frame_id_, "base_link");

  // Load initial position.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  n.getParam("init/position/x", init_x);
  n.getParam("init/position/y", init_y);
  n.getParam("init/position/z", init_z);
  n.getParam("init/orientation/roll", init_roll);
  n.getParam("init/orientation/pitch", init_pitch);
  n.getParam("init/orientation/yaw", init_yaw);

  integrated_estimate_.translation = gu::Vec3(init_x, init_y, init_z);
  integrated_estimate_.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);

  // Load algorithm parameters.
  // Stop ICP if the transformation from the last iteration was this small.
  n.param<double>("localization/tf_epsilon", params_.tf_epsilon, 0.0000000001);
  // During ICP, two points won't be considered a correspondence if they are
  // at least this far from one another.
  n.param<double>("localization/corr_dist", params_.corr_dist, 1.0);
  // Iterate ICP this many times.
  n.param<int>("localization/iterations", params_.iterations, 10);

  // Maximum acceptable incremental rotation and translation.
  n.param<bool>("localization/transform_thresholding", transform_thresholding_, false);
  n.param<double>("localization/max_translation", max_translation_, 0.05);
  n.param<double>("localization/max_rotation", max_rotation_, 0.1);

  return true;
}

bool PointCloudLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  query_pub_ = nl.advertise<PointCloud>("localization_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloud>("localization_reference_points", 10, false);
  aligned_pub_ =
      nl.advertise<PointCloud>("localization_aligned_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "localization_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "localization_integrated_estimate", 10, false);

  return true;
}

const gu::Transform3& PointCloudLocalization::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudLocalization::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

void PointCloudLocalization::SetIntegratedEstimate(
    const gu::Transform3& integrated_estimate) {
  integrated_estimate_ = integrated_estimate;

  // Publish transform between fixed frame and localization frame.
  geometry_msgs::TransformStamped tf;
  tf.transform = gr::ToRosTransform(integrated_estimate_);
  tf.header.stamp = stamp_;
  tf.header.frame_id = fixed_frame_id_;
  tf.child_frame_id = base_frame_id_;
  tfbr_.sendTransform(tf);
}

bool PointCloudLocalization::MotionUpdate(
    const gu::Transform3& incremental_odom) {
  // Store the incremental transform from odometry.
  incremental_estimate_ = incremental_odom;
  return true;
}

bool PointCloudLocalization::TransformPointsToFixedFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Compose the current incremental estimate (from odometry) with the
  // integrated estimate, and transform the incoming point cloud.
  const gu::Transform3 estimate =
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::TransformPointsToSensorFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Compose the current incremental estimate (from odometry) with the
  // integrated estimate, then invert to go from world to sensor frame.
  const gu::Transform3 estimate = gu::PoseInverse(
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_));
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::MeasurementUpdate(const PointCloud::Ptr& query,
                                               const PointCloud::Ptr& reference,
                                               PointCloud* aligned_query) {
  if (aligned_query == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Store time stamp.
  stamp_.fromNSec(query->header.stamp*1e3);

#if 1
  // ICP-based alignment. Generalized ICP does (roughly) plane-to-plane
  // matching, and is much more robust than standard ICP.
  GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(params_.tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.corr_dist);
  icp.setMaximumIterations(params_.iterations);
  icp.setRANSACIterations(0);
  icp.setMaximumOptimizerIterations(20); // default 20

  icp.setInputSource(query);
  icp.setInputTarget(reference);

  PointCloud unused;
  icp.align(unused);

  // Retrieve transformation and estimate and update.
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  pcl::transformPointCloud(*query, *aligned_query, T);

#else
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
  ndt.setInputSource (query);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (reference);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  auto T = Eigen::Matrix4f();
  T <<  0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
  if(ndt.getFitnessScore() < 0.1) {
    // Transforming unfiltered, input cloud using found transform.
    // Retrieve transformation and estimate and update.
    T = ndt.getFinalTransformation();
  }
  pcl::transformPointCloud (*query, *aligned_query, T);

#endif

  gu::Transform3 pose_update;
  pose_update.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  pose_update.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                  T(1, 0), T(1, 1), T(1, 2),
                                  T(2, 0), T(2, 1), T(2, 2));

  // Only update if the transform is small enough.
  if (!transform_thresholding_ ||
      (pose_update.translation.Norm() <= max_translation_ &&
       pose_update.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    incremental_estimate_ = gu::PoseUpdate(incremental_estimate_, pose_update);
  } else {
    ROS_WARN(
        " %s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
        name_.c_str(), pose_update.translation.Norm(),
        pose_update.rotation.ToEulerZYX().Norm());
  }

  integrated_estimate_ =
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_);

  // Convert pose estimates to ROS format and publish.
  PublishPose(incremental_estimate_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, integrated_estimate_pub_);

  // Publish point clouds for visualization.
  PublishPoints(*query, query_pub_);
  PublishPoints(*reference, reference_pub_);
  PublishPoints(*aligned_query, aligned_pub_);

  // Publish transform between fixed frame and localization frame.
  geometry_msgs::TransformStamped tf;
  tf.transform = gr::ToRosTransform(integrated_estimate_);
  tf.header.stamp = stamp_;
  tf.header.frame_id = fixed_frame_id_;
  tf.child_frame_id = base_frame_id_;
  tfbr_.sendTransform(tf);

  return true;
}

void PointCloudLocalization::PublishPoints(const PointCloud& points,
                                           const ros::Publisher& pub) const {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() > 0) {
    PointCloud out;
    out = points;
    out.header.frame_id = base_frame_id_;
    pub.publish(out);
  }
}

void PointCloudLocalization::PublishPose(const gu::Transform3& pose,
                                         const ros::Publisher& pub) const {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() == 0)
   return;

  // Convert from gu::Transform3 to ROS's PoseStamped type and publish.
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = gr::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;
  pub.publish(ros_pose);
}
