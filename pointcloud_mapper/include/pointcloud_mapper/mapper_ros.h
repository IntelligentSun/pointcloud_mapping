/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#ifndef MAPPER_ROS_H
#define MAPPER_ROS_H

#include <pcl_ros/point_cloud.h>

#include "point_cloud_mapper.h"
#include "measurement_synchronizer.h"
#include "point_cloud_odometry.h"
#include "point_cloud_localization.h"
#include "loop_closure.h"
#include "point_cloud_filter.h"

namespace pointcloud_mapper {

class MapperROS
{
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  MapperROS();
  ~MapperROS();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  // The from_log argument specifies whether to run SLAM online (subscribe to
  // topics) or by loading messages from a bag file.
  bool Initialize(const ros::NodeHandle& nh,const ros::NodeHandle& local_nh, bool from_log);

  // Sensor message processing.
  void ProcessPointCloudMessage(const PointCloud::ConstPtr& msg);

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // Sensor callbacks.
  void PointCloudCallback(const PointCloud::ConstPtr& msg);

  // Timer callbacks.
  void EstimateTimerCallback(const ros::TimerEvent& ev);
  void VisualizationTimerCallback(const ros::TimerEvent& ev);

  // Loop closing. Returns true if at least one loop closure was found. Also
  // output whether or not a new keyframe was added to the pose graph.
  bool HandleLoopClosures(const PointCloud::ConstPtr& scan, bool* new_keyframe);

  // The node's name.
  std::string name_;

  // Update rates and callback timers.
  double estimate_update_rate_;
  double visualization_update_rate_;
  ros::Timer estimate_update_timer_;
  ros::Timer visualization_update_timer_;

  // Subscribers.
  ros::Subscriber pcld_sub_;

  // Publishers
  ros::Publisher base_frame_pcld_pub_;

  // Names of coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Class objects
  MeasurementSynchronizer synchronizer_;
  PointCloudOdometry odometry_;
  LaserLoopClosure loop_closure_;
  PointCloudLocalization localization_;
  PointCloudMapper mapper_;
  PointCloudFilter filter_;
};

}//namespace

#endif // MAPPER_ROS_H
