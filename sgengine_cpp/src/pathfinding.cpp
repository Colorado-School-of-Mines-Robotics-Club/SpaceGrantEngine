#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "pathfinding.hpp"

using std::placeholders::_1;

PathfindingNode::PathfindingNode()
  : Node("pathfinding_node")
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/full_point_cloud", rclcpp::SensorDataQoS(), std::bind(&PathfindingNode::point_cloud_callback, this, std::placeholders::_1));
  down_sampled_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/downsampled_point_cloud", 10);
}

void PathfindingNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2& msg) {
  // Convert ROS2 PointCloud2 message to PCL PointCloud
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(msg, *cloud);

  // Perform downsampling using VoxelGrid
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);  // Leaf size parameters
  sor.filter(*cloud_filtered);

  counter += 1;
  if (counter % 20 != 0) {
    return;
  }

  // Convert back to ROS2 message and publish
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud_filtered, output);
  output.header.frame_id = "down_pcl";
  down_sampled_point_cloud_publisher_->publish(output);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathfindingNode>());
  rclcpp::shutdown();
  return 0;
}
