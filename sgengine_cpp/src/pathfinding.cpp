#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>

#include "pathfinding.hpp"

using std::placeholders::_1;

PathfindingNode::PathfindingNode()
  : Node("pathfinding_node")
{
  move_command_publisher_ = this->create_publisher<sgengine_messages::msg::TwoFloat>("pico/move_command", 10);
  depth_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/odom/depth", 10, std::bind(&PathfindingNode::depth_image_callback, this, _1));
  RCLCPP_INFO(this->get_logger(), "Running Pathfinding");
}

template <typename T>
cv::Mat where(cv::Mat& src, double condition, T x, std::optional<T> y = std::nullopt)
{
  cv::Mat modified;
  src.copyTo(modified);

  uint8_t* pixel_ptr  = (uint8_t*)modified.data;
  int cn = modified.channels();
  for (int r = 0; r < modified.rows; r++)
  {
    for (int c = 0; c < modified.cols; c++)
    {
      uint8_t pixel = pixel_ptr[r * modified.cols * cn + c * cn];
      if (pixel < condition) {
        pixel_ptr[r * modified.cols * cn + c * cn] = x;
      } else if (y.has_value()) {
        pixel_ptr[r * modified.cols * cn + c * cn] = y.value();
      }
    }
  }

  return modified;
}

void PathfindingNode::depth_image_callback(const sensor_msgs::msg::Image& depth_map) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth_map, depth_map.encoding);
  
  // cv::Rect roi;
  // int border_size = 50;
  // roi.x = border_size;
  // roi.y = border_size;
  // roi.width = cv_ptr->image.size().width - (border_size*2);
  // roi.height = cv_ptr->image.size().height - (border_size*2);
  // cv::Mat cropped_image = cv_ptr->image(roi);

  double min_depth;
  double max_depth;
  cv::minMaxLoc(cv_ptr->image, &min_depth, &max_depth, nullptr, nullptr);
  // std::cout << min_depth << " " << max_depth << '\n';

  cv::Mat gausian;
  // cv::filter2D(cv_ptr->image, gausian, -1, )
  cv::GaussianBlur(cv_ptr->image, gausian, cv::Size(7,7), 0);

  cv::Mat laplacian;
  cv::Laplacian(gausian, laplacian, CV_32F);

  double n = cv::norm(laplacian);
  laplacian /= n;
  laplacian *= 255.0;

  double min_laplacian;
  double max_laplacian;
  cv::minMaxLoc(cv_ptr->image, &min_laplacian, &max_laplacian, nullptr, nullptr);

  gausian -= min_depth;
  gausian /= (max_depth - min_depth);

  laplacian += 1 + gausian;

  laplacian = where(laplacian, min_laplacian + 0.3 * (max_laplacian - min_laplacian), 0);

  cv::Mat binary = where<uint8_t>(laplacian, 0.8, 0, 1);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathfindingNode>());
  rclcpp::shutdown();
  return 0;
}
