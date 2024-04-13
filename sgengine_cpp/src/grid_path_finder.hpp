#pragma once

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sgengine_messages/msg/aruco.hpp>
#include <sgengine_messages/msg/aruco_array.hpp>
#include <sgengine_messages/msg/rpyxyz.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PathFinderNode : public rclcpp::Node
{
public:
  PathFinderNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry & msg);
  void aruco_callback(const sgengine_messages::msg::ArucoArray & msg);
  void obstacle_map_callback(const nav_msgs::msg::OccupancyGrid & msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sgengine_messages::msg::ArucoArray>::SharedPtr aruco_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_subscription_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_point_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_markers_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_grid_publisher_;

  std::optional<nav_msgs::msg::Odometry> odom_position = std::nullopt;
  std::pair<float, float> odom_offset = std::pair(0.0, 0.0);
  Eigen::Matrix3d rotation_matrix_;
  std::optional<sgengine_messages::msg::Aruco> aruco_marker = std::nullopt;
};