#include "grid_path_finder.hpp"

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

  RCLCPP_INFO(this->get_logger(), "Running Grid Path Finder Node");
}

void PathFinderNode::odom_callback(const sgengine_messages::msg::RPYXYZ & msg)
{
  robot_position = msg;
}

void PathFinderNode::aruco_callback(const sgengine_messages::msg::ArucoArray & msg)
{
  if (!msg.markers.empty()) {
    aruco_marker = msg.markers[0];
    RCLCPP_INFO(this->get_logger(), "Found %i Aruco Markers", msg.markers.size());
  }
}

std::vector<std::pair<int, int>> find_path_bfs(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & grid, std::pair<int, int> start,
  std::pair<int, int> goal)
{
  int width = grid->info.width;
  int height = grid->info.height;
  std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
  std::vector<std::vector<std::pair<int, int>>> parent(
    height, std::vector<std::pair<int, int>>(width, {-1, -1}));

  std::queue<std::pair<int, int>> q;
  q.push(start);
  visited[start.second][start.first] = true;

  int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};  // Down, Right, Up, Left

  while (!q.empty()) {
    auto [x, y] = q.front();
    q.pop();

    if (x == goal.first && y == goal.second) break;  // Reached the goal

    for (auto & dir : directions) {
      int nx = x + dir[0], ny = y + dir[1];

      if (
        nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx] &&
        grid->data[ny * width + nx] == 0) {  // Check if it's free space
        visited[ny][nx] = true;
        parent[ny][nx] = {x, y};
        q.push({nx, ny});
      }
    }
  }

  // Backtrack from the goal to start to find the path
  std::vector<std::pair<int, int>> path;
  for (std::pair<int, int> at = goal; at != start; at = parent[at.second][at.first]) {
    if (at == std::make_pair(-1, -1)) return {};  // No path found
    path.push_back(at);
  }
  path.push_back(start);  // Add the start position
  std::reverse(path.begin(), path.end());
  return path;
}

void PathFinderNode::obstacle_map_callback(const nav_msgs::msg::OccupancyGrid & msg)
{
  if (robot_position == std::nullopt) {
    return;
  }
  std::pair<int, int> start_point(msg.info.origin.position.x, 0);
  std::pair<float, float> target_position = std::pair<float, float>(0.0, 1.0);
  std::pair<int, int> target_cell;
  // End point may be outside of the grid, need to find closest point in the grid
  target_cell.first = std::max(
    (uint32_t)0,
    std::min(
      msg.info.width,
      (uint32_t)(target_position.first / msg.info.resolution + msg.info.origin.position.x)));
  target_cell.second = std::max(
    (uint32_t)0,
    std::min(
      msg.info.height,
      (uint32_t)(target_position.second / msg.info.resolution + msg.info.origin.position.y)));

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
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFinderNode>());
  rclcpp::shutdown();
  return 0;
}
