
#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Point.h>

class GoalPlacer{
public: 
  GoalPlacer(ros::NodeHandle& nodeHandle);
  ~GoalPlacer();

  void publishGoal(void);
private:
  ros::NodeHandle& nodeHandle_;

  ros::Publisher goalPub_;

  tf::TransformListener* listener_;

  int current_goal_index_;
  std::vector<double> goals_x_;
  std::vector<double> goals_y_;
};
