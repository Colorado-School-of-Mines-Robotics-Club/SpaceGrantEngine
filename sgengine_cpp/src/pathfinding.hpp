#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sgengine_messages/msg/two_float.hpp"

class PathfindingNode : public rclcpp::Node {
public:
    PathfindingNode();
private:
    void depth_image_callback(const sensor_msgs::msg::Image& msg);
    rclcpp::Publisher<sgengine_messages::msg::TwoFloat>::SharedPtr move_command_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
};