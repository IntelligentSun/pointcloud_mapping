/*****************************************************************
 *
 * This file is part of the HighFormWork project
 *
 * HighFormWork 2019, from BLAM
 * Sun Jiayuan
 *
 *****************************************************************/

#include "pointcloud_mapper/mapper_ros.h"

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"pointcloud_mapper");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  pointcloud_mapper::MapperROS mapper;

  if (!mapper.Initialize(nh, private_nh, false)) {
    ROS_ERROR("initialize throw error");
    return EXIT_FAILURE;
  }

  ros::spin();
  return EXIT_SUCCESS;
}
