/** waypoint_buffer_node.hpp
 *
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * MIT License
 *
 * Declarations for WaypointBufferNode class
 */

#ifndef WAYPOINT_BUFFER_H_
#define WAYPOINT_BUFFER_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
// #include <me5413_world/path_trackerConfig.h>

namespace me5413_world
{

class WaypointBufferNode
{
 public:
  WaypointBufferNode();
  virtual ~WaypointBufferNode() {};

 private:
  void timerCallback(const ros::TimerEvent &);
  // void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  void waypointCmdCallback(const std_msgs::String& waypoint_cmd);
  // 
  bool updateCurrentWaypoint();
  bool publishWaypoints();

  // ROS declaration
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Subscriber sub_waypoint_;
  ros::Subscriber sub_waypoint_cmd_;
  ros::Subscriber sub_robot_odom_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_waypoints_all_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;
  // dynamic_reconfigure::Server<me5413_world::path_trackerConfig> server;
  // dynamic_reconfigure::Server<me5413_world::path_trackerConfig>::CallbackType f;

  // Robot pose
  std::string map_frame_;
  std::string robot_frame_;
  // nav_msgs::Odometry odom_world_robot_;
  // geometry_msgs::Pose pose_world_goal_;

  std::string mode_;

  // Waypoint Queue
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  int ptr_curr_;

  // Params
  double goal_tolerance_;
};

} // namespace me5413_world

#endif // WAYPOINT_BUFFER_H_
