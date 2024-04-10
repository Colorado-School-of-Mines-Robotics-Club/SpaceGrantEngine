#include "pathfinding.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;

PathfindingNode::PathfindingNode() : Node("pathfinding_node")
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "full_point_cloud", rclcpp::SensorDataQoS(),
    std::bind(&PathfindingNode::point_cloud_callback, this, std::placeholders::_1));
  cost_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);
  down_sampled_point_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("downsampled_point_cloud", 10);
}

void PathfindingNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
{
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
  const double rotation_angle = M_PI * 1.5;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(rotation_angle, Eigen::Vector3f::UnitX()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*voxel_filtered, *transformed_cloud, transform);

  // Filter outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(transformed_cloud);
  sor.setMeanK(50);  // Set the number of nearest neighbors to use for mean distance estimation
  sor.setStddevMulThresh(1.0);  // Set the standard deviation multiplier threshold
  sor.filter(*outliers_filtered);

  // Convert to strip
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_strip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> strip_pass;
  strip_pass.setInputCloud(outliers_filtered);
  strip_pass.setFilterFieldName("z");
  strip_pass.setFilterLimits(0.0, 0.6);  // Set the allowable height range
  strip_pass.filter(*cloud_strip);

  // Translate to 2D grid
  const float resolution = 0.03;
  const float min_distance = 0.5;

  std::map<int32_t, std::map<int32_t, uint32_t>> grid_counter;

  int32_t min_x = 0;
  int32_t max_x = 0;
  int32_t min_y = 0;
  int32_t max_y = 0;
  for (const pcl::PointXYZ & point : cloud_strip->points) {
    if (point.y < min_distance) {
      continue;
    }
    int32_t x_index = static_cast<int32_t>(point.x / resolution);
    int32_t y_index = static_cast<int32_t>(point.y / resolution);

    if (x_index < min_x) min_x = x_index;
    if (x_index > max_x) max_x = x_index;
    if (y_index < min_y) min_y = y_index;
    if (y_index > max_y) max_y = y_index;

    grid_counter[y_index][x_index] += 1;
  }

  // Translate to OccupancyGrid type
  auto grid = nav_msgs::msg::OccupancyGrid();
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "cost_map";

  grid.info.resolution = resolution;     // meters/cell
  grid.info.width = max_x - min_x + 1;   // cells
  grid.info.height = max_y - min_y + 1;  // cells

  // Origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
  grid.info.origin.position.x = -(float)(grid_counter[0].find(0)->first - min_x) * resolution;
  grid.info.origin.position.y = 0.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height);

  for (auto row = grid_counter.begin(); row != grid_counter.end(); row++) {
    uint32_t y_index = row->first;
    for (auto cell = row->second.begin(); cell != row->second.end(); cell++) {
      uint32_t x_index = cell->first;
      auto count = cell->second;

      int i = (x_index - min_x) + (y_index - min_y) * grid.info.width;

      if (count < 3) {
        grid.data[i] = 0;
      } else {
        grid.data[i] = 100;
      }
    }
  }

  cost_map_publisher_->publish(grid);

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
