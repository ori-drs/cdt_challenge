
#include "goal_placer.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_placer_node");
  ros::NodeHandle nodeHandle("~");
  
  GoalPlacer goal_placer(nodeHandle);
  
  ros::Time prev_time = ros::Time::now();
  ros::Duration wait_period(0.5);
  while (ros::ok()){
    if (ros::Time::now() - prev_time > wait_period){
      goal_placer.publishGoal();
      prev_time = ros::Time::now();
    }
  }
  return 0;
}
