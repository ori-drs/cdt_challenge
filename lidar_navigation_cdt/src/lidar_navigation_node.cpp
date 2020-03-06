#include "lidar_navigation_cdt/lidar_navigation.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_navigation_node");
  ros::NodeHandle nodeHandle("~");
  bool success;
  grid_map_demos::LidarNavigation LidarNavigation(nodeHandle, success);
  if (success) ros::spin();
  return 0;
}
