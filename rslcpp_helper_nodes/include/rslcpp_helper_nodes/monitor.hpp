// Copyright 2025 Simon Sagmeister
#pragma once
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
// Helper nodes
namespace rslcpp::helper_nodes
{
class SimulationMonitor : public rclcpp::Node
{
public:
  explicit SimulationMonitor(
    std::function<void(std::uint8_t exit_code)> abort_callback, rclcpp::NodeOptions options)
  : Node("SimulationMonitorHelperNode", options), abort_simulation_callback_(abort_callback)
  {
    declare_parameter("timeout_s", 3);
  }

private:
  void shutdown_callback(const std_msgs::msg::Bool::SharedPtr msg)
  { 
    if (msg->data) abort_simulation_callback_(0); 
  }
  void timer_callback()
  {
    if (initial_cycle_) {
      start_time_ = this->get_clock()->now();
      initial_cycle_ = false;
    }
    auto current_time = this->get_clock()->now();
    auto timeout = this->get_parameter("timeout_s").as_int();
    if ((current_time - start_time_).seconds() >= timeout) {
      std::cout << "Simulation timeouted!" << std::endl;
      abort_simulation_callback_(1);
    }
  }

private:
  bool initial_cycle_ = true;
  rclcpp::Time start_time_;
  std::function<void(std::uint8_t exit_code)> abort_simulation_callback_;

private:
  rclcpp::TimerBase::SharedPtr timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::seconds(1),
    std::bind(&SimulationMonitor::timer_callback, this));

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_error_ =
    this->create_subscription<std_msgs::msg::Bool>(
      "/rslcpp/shutdown", 1,
      std::bind(&SimulationMonitor::shutdown_callback, this, std::placeholders::_1));
};
}  // namespace rslcpp::helper_nodes
