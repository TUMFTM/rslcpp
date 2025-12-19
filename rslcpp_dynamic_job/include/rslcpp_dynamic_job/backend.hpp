// Copyright 2025 Simon Sagmeister
#pragma once

#include <rclcpp/rclcpp.hpp>
namespace rslcpp::dynamic_job
{
/// @brief Set the initial time for the job
/// @note You have to call this function before the ros2 executor
/// @note is runnning. So you cannot call this in a timer or subscription
void set_initial_time(rclcpp::Time const & initial_time);

/// @brief Set the exit code for the simulation
/// @note This automatically aborts the simulation
void set_exit_code(std::uint8_t const & exit_code);
namespace internal
{
/// @brief Get the initial time for the job
/// @note This is used internally to get the initial time for the job
rclcpp::Time get_initial_time();
/// @brief Get the exit code for the job
/// @note This is used internally to get the exit code for the job
std::uint8_t get_exit_code();
/// @brief Get the finished state of the job
/// @note This is used internally to get the finished state of the job
bool get_finished();
}  // namespace internal
}  // namespace rslcpp::dynamic_job
