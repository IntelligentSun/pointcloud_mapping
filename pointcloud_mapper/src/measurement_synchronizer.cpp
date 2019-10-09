/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#include "pointcloud_mapper/measurement_synchronizer.h"

MeasurementSynchronizer::MeasurementSynchronizer() : pending_index_(0) {}
MeasurementSynchronizer::~MeasurementSynchronizer() {}

void MeasurementSynchronizer::SortMessages() {
  sensor_ordering_.clear();

  // Accumulate all new messages in a single list.
  unsigned int ii = 0;
  for (pcld_queue::const_iterator it = pending_pclds_.begin();
       it != pending_pclds_.end(); ++it, ++ii) {
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType((*it)->msg->header.stamp.toSec(), POINTCLOUD, ii));
    sensor_ordering_.push_back(p);
  }

  ii = 0;
  for (pcl_pcld_queue::const_iterator it = pending_pcl_pclds_.begin();
       it != pending_pcl_pclds_.end(); ++it, ++ii) {
    ros::Time stamp;
    stamp.fromNSec((*it)->msg->header.stamp*1e3);
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType(stamp.toSec(), PCL_POINTCLOUD, ii));
    sensor_ordering_.push_back(p);
  }

  // Sort the list by time.
  std::sort(sensor_ordering_.begin(), sensor_ordering_.end(),
            MeasurementSynchronizer::CompareTimestamps);

  pending_index_ = 0;
}

bool MeasurementSynchronizer::GetNextMessage(sensor_type* type,
                                             unsigned int* index) {
  if (type == NULL || index == NULL) {
    ROS_WARN("[MeasurementSynchronizer.cc]: Type or index is null.");
    return false;
  }

  if (pending_index_ >= sensor_ordering_.size()) {
    return false;
  }

  *type = sensor_ordering_[pending_index_]->type;
  *index = sensor_ordering_[pending_index_]->index;

  pending_index_++;
  return true;
}

bool MeasurementSynchronizer::NextMessageExists() {
  return pending_index_ < sensor_ordering_.size();
}

void MeasurementSynchronizer::ClearMessages() {
  pending_pclds_.clear();
  pending_pcl_pclds_.clear();
}

const MeasurementSynchronizer::pcld_queue&
MeasurementSynchronizer::GetPointCloudMessages() {
  return pending_pclds_;
}

const MeasurementSynchronizer::pcl_pcld_queue&
MeasurementSynchronizer::GetPCLPointCloudMessages() {
  return pending_pcl_pclds_;
}

const MeasurementSynchronizer::Message<sensor_msgs::PointCloud2>::ConstPtr&
MeasurementSynchronizer::GetPointCloudMessage(unsigned int index) {
  return pending_pclds_[index];
}

const MeasurementSynchronizer::Message<
    pcl::PointCloud<pcl::PointXYZ>>::ConstPtr&
MeasurementSynchronizer::GetPCLPointCloudMessage(unsigned int index) {
  return pending_pcl_pclds_[index];
}

void MeasurementSynchronizer::AddPointCloudMessage(
    const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& tag) {
  Message<sensor_msgs::PointCloud2>::Ptr p(
      new Message<sensor_msgs::PointCloud2>(msg, tag));
  pending_pclds_.push_back(p);
}

void MeasurementSynchronizer::AddPCLPointCloudMessage(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg,
    const std::string& tag) {
  Message<pcl::PointCloud<pcl::PointXYZ>>::Ptr p(
      new Message<pcl::PointCloud<pcl::PointXYZ>>(msg, tag));
  pending_pcl_pclds_.push_back(p);
}
