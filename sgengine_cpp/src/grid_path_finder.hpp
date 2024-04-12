#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sgengine_messages/msg/aruco.hpp>
#include <sgengine_messages/msg/aruco_array.hpp>
#include <sgengine_messages/msg/rpyxyz.hpp>
#include <visualization_msgs/msg/marker.hpp>

class PathFinderNode : public rclcpp::Node
{
public:
  PathFinderNode();

private:
  void odom_callback(const sgengine_messages::msg::RPYXYZ & msg);
  void aruco_callback(const sgengine_messages::msg::ArucoArray & msg);
  void obstacle_map_callback(const nav_msgs::msg::OccupancyGrid & msg);

  rclcpp::Subscription<sgengine_messages::msg::RPYXYZ>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sgengine_messages::msg::ArucoArray>::SharedPtr aruco_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_subscription_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_point_publisher_;

  std::optional<sgengine_messages::msg::RPYXYZ> robot_position = std::nullopt;
  std::optional<sgengine_messages::msg::Aruco> aruco_marker = std::nullopt;
};