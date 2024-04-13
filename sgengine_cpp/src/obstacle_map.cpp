#include "obstacle_map.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;

ObstacleMapNode::ObstacleMapNode() : Node("obstacle_map_node")
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "full_point_cloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleMapNode::point_cloud_callback, this, std::placeholders::_1));
  cost_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);
  down_sampled_point_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("downsampled_point_cloud", 10);

  RCLCPP_INFO(this->get_logger(), "Running Obstacle Map Node");
}

cv::Mat occupancyGridToMat(const nav_msgs::msg::OccupancyGrid & grid)
{
  cv::Mat mat(
    grid.info.height, grid.info.width, CV_8UC1);  // Ensure 8-bit unsigned single-channel matrix

  for (unsigned int y = 0; y < grid.info.height; y++) {
    for (unsigned int x = 0; x < grid.info.width; x++) {
      int i = x + y * grid.info.width;  // Correct index in linear array
      int8_t grid_value = grid.data[i];
      uint8_t mat_value = (grid_value == -1) ? 255 : static_cast<uint8_t>(grid_value);
      mat.at<uint8_t>(y, x) = mat_value;
    }
  }
  return mat;
}

void applyGaussianBlur(
  cv::Mat & input, cv::Mat & output, int kernel_size, double sigmaX, double sigmaY)
{
  if (kernel_size % 2 == 0) kernel_size++;  // Kernel size must be odd
  cv::GaussianBlur(input, output, cv::Size(kernel_size, kernel_size), sigmaX, sigmaY);
}

void matToOccupancyGrid(const cv::Mat & mat, nav_msgs::msg::OccupancyGrid & grid)
{
  for (unsigned int y = 0; y < grid.info.height; y++) {
    for (unsigned int x = 0; x < grid.info.width; x++) {
      int i = x + y * grid.info.width;
      uint8_t value = mat.at<uint8_t>(y, x);
      grid.data[i] = value;  // Assign back to the grid, handling unknowns if necessary
    }
  }
}

void ObstacleMapNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
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

  // Filter distance
  pcl::PointCloud<pcl::PointXYZ>::Ptr distance_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> distance_strip_pass;
  distance_strip_pass.setInputCloud(outliers_filtered);
  distance_strip_pass.setFilterFieldName("y");
  distance_strip_pass.setFilterLimits(0.5, 3.0);  // Set the allowable distance range
  distance_strip_pass.filter(*distance_filtered);

  // Convert to strip
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_strip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> height_strip_pass;
  height_strip_pass.setInputCloud(distance_filtered);
  height_strip_pass.setFilterFieldName("z");
  height_strip_pass.setFilterLimits(0.0, 0.6);  // Set the allowable height range
  height_strip_pass.filter(*cloud_strip);

  // Translate to 2D grid
  const float resolution = 0.03;

  std::map<int32_t, std::map<int32_t, uint32_t>> grid_counter;

  int32_t min_x = 0;
  int32_t max_x = 0;
  int32_t min_y = 0;
  int32_t max_y = 0;
  for (const pcl::PointXYZ & point : cloud_strip->points) {
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
  grid.header.frame_id = "obstacle_map";

  grid.info.resolution = resolution;     // meters/cell
  grid.info.width = max_x - min_x + 1;   // cells
  grid.info.height = max_y - min_y + 1;  // cells

  // Origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
  grid.info.origin.position.x = -(double)(grid_counter[0].find(0)->first - min_x) * resolution;
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

  cv::Mat original = occupancyGridToMat(grid);
  cv::Mat blurred;
  applyGaussianBlur(original, blurred, 10, 2.0, 2.0);
  matToOccupancyGrid(blurred, grid);

  for (auto & cell : grid.data) {
    if (cell > 0) cell = 25;
  }

  cost_map_publisher_->publish(grid);

  // Convert back to PointCloud2 msg and publish
  pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*cloud_strip, *output_cloud);
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(*output_cloud, output);
  output.header.frame_id = "obstacle_map";
  down_sampled_point_cloud_publisher_->publish(output);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleMapNode>());
  rclcpp::shutdown();
  return 0;
}
