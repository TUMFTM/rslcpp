// Copyright 2025 Simon Sagmeister
#include "rslcpp_dynamic_job/backend.hpp"
// Implement a static storage and the functions here
static struct
{
  // Add your static variables here
  rclcpp::Time initial_time{
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count(),
    RCL_SYSTEM_TIME};
  rclcpp::Duration time_step_size{std::chrono::milliseconds(1)};  // Default to 10 milliseconds
  std::uint8_t exit_code{0};
  bool finished{false};
} __backend_storage__;
namespace rslcpp::dynamic_job
{
void set_initial_time(rclcpp::Time const & initial_time)
{
  __backend_storage__.initial_time = initial_time;
}
void set_exit_code(std::uint8_t const & exit_code)
{
  __backend_storage__.exit_code = exit_code;
  __backend_storage__.finished = true;  // Automatically set finished to true when exit code is set
}
void set_time_step_size(rclcpp::Duration const & duration)
{
  __backend_storage__.time_step_size = duration;
}
namespace internal
{
rclcpp::Time get_initial_time() { return __backend_storage__.initial_time; }
std::uint8_t get_exit_code() { return __backend_storage__.exit_code; }
bool get_finished() { return __backend_storage__.finished; }
rclcpp::Duration get_time_step_size() { return __backend_storage__.time_step_size; }
}  // namespace internal
}  // namespace rslcpp::dynamic_job
