// Copyright 2025 Simon Sagmeister
#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rslcpp_time_delay_backend/callback_backend.hpp>
#include <string>
#include <vector>

#include "rslcpp/utilities.hpp"
namespace rslcpp
{
typedef std::uint8_t exit_code_t;
class Job
{
public:
  typedef std::shared_ptr<Job> SharedPtr;
  typedef std::unique_ptr<Job> UniquePtr;

public:
  /// Methods will be called in the order they are defined here
  virtual ~Job() = default;

  /// @brief Create all nodes that are part of the simulation and return them in a vector
  virtual std::vector<rclcpp::Node::SharedPtr> create_and_get_nodes() = 0;
  /// @brief Get the initial simulation time
  virtual rclcpp::Time get_initial_time() = 0;
  /// @brief  Get the time step size. This is the clock "tick" of the simulation
  virtual rclcpp::Duration get_time_step_size() = 0;
  /// @brief Return true to stop the simulation. As a result, all the nodes created above will be
  /// destroyed.
  virtual bool get_finished() = 0;
  /// @brief Return the exit code of the simulation. This will be called after stopping the job.
  virtual exit_code_t get_exit_code() = 0;
};
/// @brief Run a job. This will block until the job is finished and will then return its exit code
exit_code_t run_job(int argc, char ** argv, Job::SharedPtr job);

}  // namespace rslcpp
