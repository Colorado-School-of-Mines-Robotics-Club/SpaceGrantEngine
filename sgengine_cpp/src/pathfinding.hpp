#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PathfindingNode : public rclcpp::Node {
public:
    PathfindingNode();
private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2& msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr down_sampled_point_cloud_publisher_;

    int counter = 0;
};