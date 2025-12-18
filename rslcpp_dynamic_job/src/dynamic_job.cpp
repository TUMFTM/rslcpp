// Copyright 2025 Simon Sagmeister
#include "rslcpp_dynamic_job/dynamic_job.hpp"

#include "rslcpp_dynamic_job/backend.hpp"
namespace rslcpp::dynamic_job
{
std::vector<rclcpp::Node::SharedPtr> DynamicJob::create_and_get_nodes()
{
  // Apply default configuration before creating the nodes
  set_default_configuration_from_ros_parameters();

  using nv_T = std::vector<rclcpp::Node::SharedPtr>;

  nv_T return_val;

  if (arbitrary_callback_pre_) arbitrary_callback_pre_();

  // Fixed nodes before components are loaded
  if (create_fixed_nodes_pre_) {
    auto fixed_nodes = create_fixed_nodes_pre_();
    return_val.insert(return_val.end(), fixed_nodes.begin(), fixed_nodes.end());
  }

  // Load dynamically loaded nodes
  nv_T dynamically_loaded_components =
    component_loader_.load_and_create_nodes(component_descriptions_, node_options_);
  return_val.insert(
    return_val.end(), dynamically_loaded_components.begin(), dynamically_loaded_components.end());

  // Fixed nodes after components are loaded
  if (create_fixed_nodes_post_) {
    auto fixed_nodes = create_fixed_nodes_post_();
    return_val.insert(return_val.end(), fixed_nodes.begin(), fixed_nodes.end());
  }

  if (arbitrary_callback_post_) arbitrary_callback_post_();
  // Return the concatenated vector
  return return_val;
}
rclcpp::Time DynamicJob::get_initial_time() { return internal::get_initial_time(); }
bool DynamicJob::get_finished() { return internal::get_finished(); }
exit_code_t DynamicJob::get_exit_code() { return internal::get_exit_code(); }
void DynamicJob::set_pre_node_creation_callback(std::function<void()> && callback)
{
  arbitrary_callback_pre_ = std::move(callback);
}
void DynamicJob::set_post_node_creation_callback(std::function<void()> && callback)
{
  arbitrary_callback_post_ = std::move(callback);
}
void DynamicJob::set_create_fixed_nodes_pre_callback(
  std::function<std::vector<rclcpp::Node::SharedPtr>()> && callback)
{
  create_fixed_nodes_pre_ = std::move(callback);
}
void DynamicJob::set_create_fixed_nodes_post_callback(
  std::function<std::vector<rclcpp::Node::SharedPtr>()> && callback)
{
  create_fixed_nodes_post_ = std::move(callback);
}
void DynamicJob::set_default_configuration_from_ros_parameters()
{
  rclcpp::NodeOptions param_loader_opts;
  param_loader_opts.use_intra_process_comms(true);
  param_loader_opts.start_parameter_event_publisher(false);
  param_loader_opts.enable_rosout(false);
  param_loader_opts.enable_topic_statistics(false);
  param_loader_opts.use_clock_thread(false);
  param_loader_opts.automatically_declare_parameters_from_overrides(
    true);  // Allow automatic declaration of parameters from overrides

  // Create a temporary node to load parameters
  auto param_loader_node =
    std::make_shared<rclcpp::Node>("DynamicJobConfiguration", param_loader_opts);

  if (param_loader_node->has_parameter("initial_time_ns_since_epoch")) {
    auto initial_time = param_loader_node->get_parameter("initial_time_ns_since_epoch").as_int();
    set_initial_time(rclcpp::Time(initial_time));
    std::cout << "Dynamic Job | Initial time set via parameter to: " << initial_time
              << " ns since epoch" << std::endl;
  }

  param_loader_node = nullptr;  // Ensure the node is destroyed to free resources
}
}  // namespace rslcpp::dynamic_job
