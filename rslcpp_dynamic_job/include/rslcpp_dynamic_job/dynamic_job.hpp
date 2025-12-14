// Copyright 2025 Simon Sagmeister
#pragma once
#include <functional>
#include <memory>
#include <rslcpp/rslcpp.hpp>
#include <rslcpp_dynamic_node_composition/composition.hpp>
#include <utility>
#include <vector>

#include "rslcpp_dynamic_job/backend.hpp"
namespace rslcpp::dynamic_job
{
struct DynamicJob : public rslcpp::Job
{
public:
  typedef std::shared_ptr<Job> SharedPtr;
  typedef std::unique_ptr<Job> UniquePtr;

  std::vector<rclcpp::Node::SharedPtr> create_and_get_nodes() override;
  rclcpp::Time get_initial_time() override;
  rclcpp::Duration get_time_step_size() override;
  bool get_finished() override;
  exit_code_t get_exit_code() override;

public:
  DynamicJob(
    std::vector<rslcpp::dynamic_composition::ComponentDescription> const & component_descriptions,
    rclcpp::NodeOptions const & options = rclcpp::NodeOptions())
  : rslcpp::Job{}, component_descriptions_{component_descriptions}, node_options_{options}
  {
  }
  ~DynamicJob() override = default;

public:
  // Create setter function for setting the created callbacks
  void set_pre_node_creation_callback(std::function<void()> && callback);
  void set_post_node_creation_callback(std::function<void()> && callback);
  void set_create_fixed_nodes_pre_callback(
    std::function<std::vector<rclcpp::Node::SharedPtr>()> && callback);
  void set_create_fixed_nodes_post_callback(
    std::function<std::vector<rclcpp::Node::SharedPtr>()> && callback);

private:
  rslcpp::dynamic_composition::ComponentLoader component_loader_{};
  std::vector<rslcpp::dynamic_composition::ComponentDescription> component_descriptions_;
  rclcpp::NodeOptions node_options_;

private:
  std::function<void()> arbitrary_callback_pre_;
  std::function<std::vector<rclcpp::Node::SharedPtr>()> create_fixed_nodes_pre_;
  std::function<std::vector<rclcpp::Node::SharedPtr>()> create_fixed_nodes_post_;
  std::function<void()> arbitrary_callback_post_;

private:
  void set_default_configuration_from_ros_parameters();
};
}  // namespace rslcpp::dynamic_job
