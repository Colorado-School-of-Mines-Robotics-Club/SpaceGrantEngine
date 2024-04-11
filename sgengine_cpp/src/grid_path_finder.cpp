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

  std::cout << "Running Grid Path Finder Node" << std::endl;
}

void PathFinderNode::odom_callback(const sgengine_messages::msg::RPYXYZ & msg)
{
  robot_position = msg;
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
  // End point may be outside of the grid, need to find closest point in the grid
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFinderNode>());
  rclcpp::shutdown();
  return 0;
}
