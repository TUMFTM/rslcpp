// Copyright 2025 Simon Sagmeister
#pragma once

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
namespace rslcpp
{
/// @brief Overwrite the ros2 clock of node with a given time.
/// @param time current simulation time
/// @param clock node clock to be updated
inline void set_clock(rclcpp::Time const & time, rclcpp::Clock::SharedPtr clock)
{
  std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());

  if (!clock->ros_time_is_active()) {
    auto ret = rcl_enable_ros_time_override(clock->get_clock_handle());
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to enable ros_time_override_status");
    }
  }

  auto ret = rcl_set_ros_time_override(clock->get_clock_handle(), time.nanoseconds());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to set ros_time_override_status");
  }
}
}  // namespace rslcpp