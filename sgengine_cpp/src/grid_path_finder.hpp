#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sgengine_messages/msg/rpyxyz.hpp>

class PathFinderNode : public rclcpp::Node
{
public:
  PathFinderNode();

private:
  void odom_callback(const sgengine_messages::msg::RPYXYZ & msg);
  void obstacle_map_callback(const nav_msgs::msg::OccupancyGrid & msg);
  rclcpp::Subscription<sgengine_messages::msg::RPYXYZ>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_subscription_;

  sgengine_messages::msg::RPYXYZ robot_position;
};