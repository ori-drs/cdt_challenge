#include "grid_map_cdt/lidar_navigation.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_navigation_node");
  ros::NodeHandle nodeHandle("~");
  bool success;
  grid_map_demos::NavigationDemo navigationDemo(nodeHandle, success);
  if (success) ros::spin();
  return 0;
}
