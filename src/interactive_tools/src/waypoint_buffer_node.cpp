/** waypoint_buffer_node.cpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * ROS Node for robot to patrol a given list of waypoints
 */

#include "interactive_tools/waypoint_buffer_node.hpp"

namespace me5413_world
{

// Dynamic Parameters
bool PARAMS_UPDATED;

// void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
// {
//   // Common Params
//   MAP_FRAME = config.map_frame;
// }

WaypointBufferNode::WaypointBufferNode() : tf2_listener_(tf2_buffer_)
{
  // f = boost::bind(&dynamicParamCallback, _1, _2);
  // server.setCallback(f);

  this->timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointBufferNode::timerCallback, this);

  // this->sub_robot_odom_ = nh_.subscribe("/odom", 1, &WaypointBufferNode::robotOdomCallback, this);
  this->sub_waypoint_ = nh_.subscribe("/move_base_simple/goal", 1, &WaypointBufferNode::waypointCallback, this);
  this->sub_waypoint_cmd_ = nh_.subscribe("/rviz_panel/waypoint_cmd", 1, &WaypointBufferNode::waypointCmdCallback, this);
  
  this->pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/waypoint_buffer/current_goal", 1);
  this->pub_waypoints_all_ = nh_.advertise<nav_msgs::Path>("/waypoint_buffer/waypoints", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->mode_ == "";
  this->ptr_curr_ = 0;
  this->goal_tolerance_ = 0.6;
  
}

void WaypointBufferNode::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
{
  // Calculate absolute errors (wrt to world frame)
  if (this->mode_ == "Add")
  {
    this->waypoints_.push_back(*goal_pose.get());
  }
  
  publishWaypoints();
  return;
}

void WaypointBufferNode::waypointCmdCallback(const std_msgs::String& waypoint_cmd)
{
  if (waypoint_cmd.data == "Add")
  {
    this->mode_ = waypoint_cmd.data;
  }
  else if (waypoint_cmd.data == "Remove")
  {
    if (this->waypoints_.empty())
    {
      ROS_WARN("waypoint is currently empty");
      this->ptr_curr_ = 0;
    }
    else
    {
      this->waypoints_.pop_back();
      if (this->ptr_curr_ >= this->waypoints_.size())
      {
        this->ptr_curr_ = this->waypoints_.size() - 1;
      }
    }
    publishWaypoints();
    this->mode_ = waypoint_cmd.data;
  }
  else if (waypoint_cmd.data == "Start")
  {
    this->mode_ = waypoint_cmd.data;
  }
  else if (waypoint_cmd.data == "Clear")
  {
    this->waypoints_.clear();
    this->ptr_curr_ = 0;
    publishWaypoints();
    this->mode_ = waypoint_cmd.data;
  }
  else if (waypoint_cmd.data == "Skip")
  {
    if (this->ptr_curr_ + 1 < this->waypoints_.size())
    {
      this->ptr_curr_++;
    }
    publishWaypoints();
    // this->mode_ = waypoint_cmd.data;
  }
  else if (waypoint_cmd.data == "Loop")
  {
    this->ptr_curr_ = 0;
    publishWaypoints();
    // this->mode_ = waypoint_cmd.data;
  }
  else
  {
    ROS_WARN("Waypoint command not supported");
  }

  return;
}

bool WaypointBufferNode::publishWaypoints()
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = this->map_frame_;
  path_msg.header.stamp = ros::Time::now();
  path_msg.poses = this->waypoints_;
  pub_waypoints_all_.publish(path_msg);

  return true;
}

void WaypointBufferNode::timerCallback(const ros::TimerEvent &)
{
  if (this->mode_ == "Start" && !this->waypoints_.empty() && this->ptr_curr_ < this->waypoints_.size())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tf2_buffer_.lookupTransform(this->map_frame_, this->robot_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      // ros::Duration(1.0).sleep();
    }

    const double dist = std::hypot(transformStamped.transform.translation.x - this->waypoints_[this->ptr_curr_].pose.position.x, 
                                   transformStamped.transform.translation.y - this->waypoints_[this->ptr_curr_].pose.position.y);
    ROS_INFO("Distance to current Goal is %.2f m", dist);
    ROS_INFO("Current Goal is at (%.2f, %.2f)", this->waypoints_[this->ptr_curr_].pose.position.x, this->waypoints_[this->ptr_curr_].pose.position.y);
    ROS_INFO("Current Robot is at (%.2f, %.2f)", transformStamped.transform.translation.x, transformStamped.transform.translation.y);

    if (dist <= this->goal_tolerance_)
    {
      ROS_INFO("Waypoint %d has been reached!", this->ptr_curr_);
      this->ptr_curr_++;
    }

    if (this->ptr_curr_ < this->waypoints_.size())
    {
      geometry_msgs::PoseStamped pose = this->waypoints_[this->ptr_curr_];
      pose.header.frame_id = this->map_frame_;
      pose.header.stamp = ros::Time::now();
      this->pub_goal_.publish(pose);
    }
  }
  
  return;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_buffer_node");
  me5413_world::WaypointBufferNode waypoint_buffer_node;
  ros::spin();  // spin the ros node.
  return 0;
}
