// Copyright 2025 Simon Sagmeister
#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
class TimeDelayPublisher : public rclcpp::Node
{
public:
  // Constructor that accepts rclcpp::NodeOptions
  explicit TimeDelayPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("time_delay_publisher", options)
  {
    // Create publishers for each topic
    no_delay_publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("/no_delay", 1);
    fixed_delay_1_publisher_ =
      this->create_publisher<builtin_interfaces::msg::Time>("/fixed_delay_1", 1);
    fixed_delay_2_publisher_ =
      this->create_publisher<builtin_interfaces::msg::Time>("/fixed_delay_2", 1);
    measured_delay_publisher_ =
      this->create_publisher<builtin_interfaces::msg::Time>("/measured_delay", 1);
    measured_delay_with_skew_publisher_ =
      this->create_publisher<builtin_interfaces::msg::Time>("/measured_delay_with_skew", 1);

    // Create a timer to publish messages at 2 Hz
    timer_ = this->create_timer(
      std::chrono::milliseconds(100), std::bind(&TimeDelayPublisher::publish_time, this));
  }

private:
  void publish_time()
  {
    // Get the current ROS 2 time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto now = this->get_clock()->now();
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = now.seconds();
    time_msg.nanosec = now.nanoseconds() % 1'000'000'000;

    // Publish the timestamp to all topics
    no_delay_publisher_->publish(time_msg);
    fixed_delay_1_publisher_->publish(time_msg);
    fixed_delay_2_publisher_->publish(time_msg);
    measured_delay_publisher_->publish(time_msg);
    measured_delay_with_skew_publisher_->publish(time_msg);
  }
  // Publishers
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr no_delay_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr fixed_delay_1_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr fixed_delay_2_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr measured_delay_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr measured_delay_with_skew_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};