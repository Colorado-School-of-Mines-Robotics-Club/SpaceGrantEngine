#include "grid_path_finder.hpp"

#include <cmath>
#include <map>
#include <queue>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

PathFinderNode::PathFinderNode() : Node("obstacle_map_node")
{
  odom_subscription_ = this->create_subscription<sgengine_messages::msg::RPYXYZ>(
    "odom/rpy_xyz", 10, std::bind(&PathFinderNode::odom_callback, this, std::placeholders::_1));
  obstacle_map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "obstacle_map", 10,
    std::bind(&PathFinderNode::obstacle_map_callback, this, std::placeholders::_1));
  aruco_subscription_ = this->create_subscription<sgengine_messages::msg::ArucoArray>(
    "oak/aruco", 10, std::bind(&PathFinderNode::aruco_callback, this, std::placeholders::_1));

  target_point_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("target_point", 10);
  path_markers_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("path_to_target_markers", 10);
  debug_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("debug_grid", 10);

  RCLCPP_INFO(this->get_logger(), "Running Grid Path Finder Node");
}

// Function to create a rotation matrix from roll, pitch, and yaw
Eigen::Matrix3d calc_rotation_matrix(double roll, double pitch, double yaw)
{
  // Precompute cosine and sine of angles
  double cy = std::cos(yaw);
  double sy = std::sin(yaw);
  double cp = std::cos(pitch);
  double sp = std::sin(pitch);
  double cr = std::cos(roll);
  double sr = std::sin(roll);

  Eigen::Matrix3d Rz;
  Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;

  Eigen::Matrix3d Ry;
  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;

  Eigen::Matrix3d Rx;
  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;

  return Rz * Ry * Rx;  // Matrix multiplication
}

void PathFinderNode::odom_callback(const sgengine_messages::msg::RPYXYZ & msg)
{
  if (
    msg != *odom_position && msg.x != 0 && msg.y != 0 && msg.z != 0 && msg.roll != 0 &&
    msg.pitch != 0 && msg.yaw != 0) {
    odom_position = msg;
    RCLCPP_INFO(
      this->get_logger(), "Odom: %f, %f, %f", odom_position->x, odom_position->y, odom_position->z);

    rotation_matrix_ =
      calc_rotation_matrix(odom_position->roll, odom_position->pitch, odom_position->yaw);
  }
}

void PathFinderNode::aruco_callback(const sgengine_messages::msg::ArucoArray & msg)
{
  if (!msg.markers.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Found %li Aruco Markers", msg.markers.size());
    for (auto & marker : msg.markers) {
      // id of specific marker that we're using
      if (marker.id == 18) {
        if (odom_position != std::nullopt) {
          odom_offset.first = odom_position->x;
          odom_offset.second = odom_position->y;
          odom_position = std::nullopt;
        }
        aruco_marker = marker;
      }
    }
  }
}

std::vector<std::pair<int, int>> find_path_bfs(
  const nav_msgs::msg::OccupancyGrid & grid, std::pair<int, int> start, std::pair<int, int> goal)
{
  int width = grid.info.width;
  int height = grid.info.height;
  std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
  std::vector<std::vector<std::pair<int, int>>> parent(
    height, std::vector<std::pair<int, int>>(width, {-1, -1}));

  std::queue<std::pair<int, int>> q;
  q.push(start);
  visited[start.second][start.first] = true;

  int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};  // Down, Right, Up, Left

  bool path_found = false;

  while (!q.empty()) {
    auto [x, y] = q.front();
    q.pop();

    if (x == goal.first && y == goal.second) {
      path_found = true;
      break;  // Reached the goal
    }

    for (auto & dir : directions) {
      int nx = x + dir[0], ny = y + dir[1];

      if (
        nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx] &&
        grid.data[ny * width + nx] == 0) {  // Check if it's free space
        visited[ny][nx] = true;
        parent[ny][nx] = {x, y};
        q.push({nx, ny});
      }
    }
  }

  // If no path was found, return an empty path
  if (!path_found) {
    return {};
  }

  // Backtrack from the goal to start to find the path
  std::vector<std::pair<int, int>> path;
  for (std::pair<int, int> at = goal; at != start; at = parent[at.second][at.first]) {
    if (at == std::make_pair(-1, -1)) return {};  // Safety check, should not happen
    path.push_back(at);
  }
  path.push_back(start);  // Add the start position
  std::reverse(path.begin(), path.end());
  return path;
}

void PathFinderNode::obstacle_map_callback(const nav_msgs::msg::OccupancyGrid & msg)
{
  if (odom_position == std::nullopt)  // || aruco_marker == std::nullopt)
    return;

  std::pair<int, int> start_point(-msg.info.origin.position.x / msg.info.resolution, 0);
  // std::pair<float, float> offset_from_last_marker_detection = std::pair<float, float>(
  //   odom_position->x - odom_offset.first, odom_position->y - odom_offset.second);
  // // Calculate the position of the aruco marker relative to the current robot position based on where it was when the marker was last detected
  // std::pair<float, float> target_position = std::pair<float, float>(
  //   aruco_marker->marker.translation.x - offset_from_last_marker_detection.first,
  //   aruco_marker->marker.translation.y - offset_from_last_marker_detection.second);

  // X & Y from odom are flipped?
  Eigen::Vector3d translation(odom_position->y, odom_position->x, odom_position->z);
  Eigen::Vector3d p_final(0.0, 2.0, 0);

  Eigen::Vector3d p_rotated = rotation_matrix_ * p_final;
  Eigen::Vector3d p_new = p_rotated + translation;
  std::pair<float, float> target_position = std::pair<float, float>(p_new[0], p_new[1]);

  // End point may be outside of the grid, need to find closest point in the grid
  std::pair<uint32_t, uint32_t> target_cell = std::pair<uint32_t, uint32_t>(
    std::max(
      (uint32_t)0,
      std::min(
        msg.info.width - 1,
        (uint32_t)(-msg.info.origin.position.x / msg.info.resolution + target_position.first / msg.info.resolution))),
    std::max(
      (uint32_t)0,
      std::min(
        msg.info.height - 1,
        (uint32_t)(-msg.info.origin.position.y / msg.info.resolution + target_position.second / msg.info.resolution))));

  std::vector<std::pair<int, int>> target_path = find_path_bfs(msg, start_point, target_cell);
  // if (target_path.empty()) {
  //   RCLCPP_WARN(this->get_logger(), "Could not find path to target");
  // }

  auto debug_grid = msg;
  RCLCPP_DEBUG(
    this->get_logger(), "Origin cell is %f, %f", msg.info.origin.position.x / msg.info.resolution,
    msg.info.origin.position.y / msg.info.resolution);
  RCLCPP_DEBUG(this->get_logger(), "Grid size is %i, %i", msg.info.width, msg.info.height);
  RCLCPP_DEBUG(
    this->get_logger(), "Target cell is %i, %i", target_cell.first + 1, target_cell.second + 1);
  int i = target_cell.first + target_cell.second * msg.info.width;
  debug_grid.data[i] = 100;
  debug_grid_publisher_->publish(debug_grid);

  // Target Marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "obstacle_map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "marker_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = target_position.first;
  marker.pose.position.y = target_position.second;
  marker.pose.position.z = 0.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  target_point_publisher_->publish(marker);

  // Path to end marker
  visualization_msgs::msg::MarkerArray marker_array;
  for (int i = 0; i < (int)target_path.size(); i++) {
    auto & step = target_path[i];
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "obstacle_map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "marker_namespace";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = step.first * msg.info.resolution + msg.info.origin.position.x;
    marker.pose.position.y = step.second * msg.info.resolution + msg.info.origin.position.y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
  path_markers_publisher_->publish(marker_array);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFinderNode>());
  rclcpp::shutdown();
  return 0;
}
