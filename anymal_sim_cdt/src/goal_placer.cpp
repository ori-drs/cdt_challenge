
#include "goal_placer.h"

GoalPlacer::GoalPlacer(ros::NodeHandle& nodeHandle) : 
                      nodeHandle_(nodeHandle), 
                      current_goal_index_(0)
{
  ROS_INFO("Class GoalPlacer constructed. ");

  listener_ = new tf::TransformListener();

  goalPub_ = nodeHandle_.advertise<geometry_msgs::Point>("/cdt_challange/goal", 10);

  goals_x_.push_back(0.0);
  goals_y_.push_back(0.0);

  goals_x_.push_back(1.0);
  goals_y_.push_back(1.0);

  goals_x_.push_back(14.5);
  goals_y_.push_back(4.0);
}

GoalPlacer::~GoalPlacer(){
  ROS_INFO("Class GoalPlacer destructed. ");
}

void GoalPlacer::publishGoal(void){

  tf::StampedTransform transform;
  try {
      listener_->waitForTransform("/odom", "/base", ros::Time(0), ros::Duration(10.0) );
      listener_->lookupTransform("/odom", "/base", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }

  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  tf::transformTFToEigen(transform, pose_robot);

  Eigen::Vector2d goal(goals_x_[current_goal_index_], goals_y_[current_goal_index_]);
  Eigen::Vector2d pos_robot_2d = pose_robot.translation().head(2);
  double distance_to_goal = (goal - pos_robot_2d).norm();

  if (distance_to_goal < 0.5){
    current_goal_index_++;
  }

  if (current_goal_index_ >= goals_x_.size()){
    ROS_INFO("This is the last goal, no further goal will be placed. ");
    return;
  }

  geometry_msgs::Point msg;
  msg.x = goals_y_[current_goal_index_];
  msg.y = goals_y_[current_goal_index_];
  msg.z = 0.0;
  goalPub_.publish(msg);
}
