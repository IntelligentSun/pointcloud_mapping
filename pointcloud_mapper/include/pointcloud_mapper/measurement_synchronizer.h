/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#ifndef MEASUREMENT_SYNCHRONIZER_H
#define MEASUREMENT_SYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <memory>
#include <vector>

class MeasurementSynchronizer {
 public:
  MeasurementSynchronizer();
  ~MeasurementSynchronizer();

  // Enums for all valid sensor message types.
  typedef enum {
    POINTCLOUD,
    PCL_POINTCLOUD
  } sensor_type;

  // Basic sorting, querying, clearing.
  void SortMessages();
  bool GetNextMessage(sensor_type* type, unsigned int* index);
  bool NextMessageExists();
  void ClearMessages();

  // Templated message type for holding generic sensor messages.
  template<typename T>
  struct Message {
    typename T::ConstPtr msg;
    std::string tag;
    typedef std::shared_ptr<Message<T>> Ptr;
    typedef std::shared_ptr<const Message<T>> ConstPtr;
    Message(const typename T::ConstPtr m, const std::string& t)
        : msg(m), tag(t) {}
  };

  // Typedefs for queues of all sensor types.
  typedef std::vector<Message<sensor_msgs::PointCloud2>::ConstPtr> pcld_queue;
  typedef std::vector<Message<pcl::PointCloud<pcl::PointXYZ>>::ConstPtr>
      pcl_pcld_queue;

  // Methods for accessing entire queues of accumulated measurements.
  const pcld_queue& GetPointCloudMessages();
  const pcl_pcld_queue& GetPCLPointCloudMessages();

  // Methods for accessing a single measurement by index.
  const Message<sensor_msgs::PointCloud2>::ConstPtr& GetPointCloudMessage(
      unsigned int index);
  const Message<pcl::PointCloud<pcl::PointXYZ>>::ConstPtr&
      GetPCLPointCloudMessage(unsigned int index);

  // Methods for adding sensor measurements of specific types.
  void AddPointCloudMessage(const sensor_msgs::PointCloud2::ConstPtr& msg,
                            const std::string& tag = std::string());
  void AddPCLPointCloudMessage(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg,
      const std::string& tag = std::string());

  // Static enum to string conversion.
  static std::string GetTypeString(const sensor_type& type) {
    switch(type) {
      case POINTCLOUD:
        return std::string("POINTCLOUD");
      case PCL_POINTCLOUD:
        return std::string("PCL_POINTCLOUD");
      // No default to force compile-time error.
    }
  }

 private:
  // Templated message for sorting generic sensor messages by timestamp.
  struct TimestampedType {
    double time;
    sensor_type type;
    unsigned int index;
    TimestampedType(double t, sensor_type s, unsigned int i)
        : time(t), type(s), index(i) {}
    typedef std::shared_ptr<TimestampedType> Ptr;
    typedef std::shared_ptr<const TimestampedType> ConstPtr;
  };

  // Sorting function.
  static bool CompareTimestamps(const TimestampedType::ConstPtr& lhs,
                                const TimestampedType::ConstPtr& rhs) {
    return ((lhs->time < rhs->time) ||
            ((lhs->time == rhs->time) && (lhs->type < rhs->type)));
  }

  // Queues of sensor messages.
  pcld_queue pending_pclds_;
  pcl_pcld_queue pending_pcl_pclds_;

  unsigned int pending_index_;
  std::vector<TimestampedType::ConstPtr> sensor_ordering_;
};

#endif // MEASUREMENT_SYNCHRONIZER_H
