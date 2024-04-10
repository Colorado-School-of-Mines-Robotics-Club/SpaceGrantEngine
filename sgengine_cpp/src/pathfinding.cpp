#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "pathfinding.hpp"

using std::placeholders::_1;

PathfindingNode::PathfindingNode()
  : Node("pathfinding_node")
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/full_point_cloud", rclcpp::SensorDataQoS(), std::bind(&PathfindingNode::point_cloud_callback, this, std::placeholders::_1));
  down_sampled_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/downsampled_point_cloud", 10);
}

void PathfindingNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2& msg) {
  // Convert to PCL
  pcl::PCLPointCloud2::Ptr original_cloud(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(msg, *original_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(*original_cloud, *pcl_cloud);

  // Downsample points
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_sor;
  voxel_sor.setInputCloud(pcl_cloud);
  voxel_sor.setLeafSize(0.03f, 0.03f, 0.03f);
  voxel_sor.filter(*voxel_filtered);

  // Rotate points to correct orrientation
  double rotation_angle = M_PI * 1.5;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(rotation_angle, Eigen::Vector3f::UnitX()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*voxel_filtered, *transformed_cloud, transform);

  // Convert to strip
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_strip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> strip_pass;
  strip_pass.setInputCloud(transformed_cloud);
  strip_pass.setFilterFieldName("z");
  strip_pass.setFilterLimits(0.0, 0.35); // Set the allowable height range
  strip_pass.filter(*cloud_strip);

  // Convert back to PointCloud2 msg and publish
  pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*cloud_strip, *output_cloud);
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(*output_cloud, output);
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
