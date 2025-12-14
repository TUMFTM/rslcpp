// Copyright 2025 Simon Sagmeister
#include "rslcpp_helper_nodes/monitor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rslcpp_dynamic_job/backend.hpp>

// Create the required default constructor to have a composable node
using SimulationMonitorGeneric = rslcpp::helper_nodes::SimulationMonitor;
namespace rslcpp_helper_nodes
{
struct SimulationMonitor : public SimulationMonitorGeneric
{
  explicit SimulationMonitor(const rclcpp::NodeOptions & options)
  : SimulationMonitorGeneric(
      [](std::uint8_t exit_code) { rslcpp::dynamic_job::set_exit_code(exit_code); }, options)
  {
  }
};
}  // namespace rslcpp_helper_nodes
RCLCPP_COMPONENTS_REGISTER_NODE(rslcpp_helper_nodes::SimulationMonitor)
