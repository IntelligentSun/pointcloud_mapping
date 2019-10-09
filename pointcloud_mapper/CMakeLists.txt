cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_mapper)

add_compile_options(-std=c++11)

find_package(PCL REQUIRED COMPONENTS common octree)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif(NOT PCL_FOUND)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        tf
        tf2_ros
        geometry_utils
        pose_graph_msgs
        )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pcl_ros
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

#add_executable(mapper
#        src/mapper_ros_node.cpp
#        src/mapper_ros.cpp
#        src/point_cloud_mapper.cpp
#        src/measurement_synchronizer.cpp
#        src/point_cloud_odometry.cpp
#        src/point_cloud_localization.cpp
#        src/loop_closure.cpp
#        src/point_cloud_filter.cpp
#        )

#target_link_libraries(mapper
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        gtsam
#        tbb
#        )
