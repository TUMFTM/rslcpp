// Copyright 2025 Simon Sagmeister
#include "rslcpp_helper_nodes/recorder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Create the required default constructor to have a composable node
using BagRecorderGeneric = rslcpp::helper_nodes::BagRecorder;
namespace rslcpp_helper_nodes
{
struct BagRecorder : public BagRecorderGeneric
{
  explicit BagRecorder(const rclcpp::NodeOptions & options) : BagRecorderGeneric(options) {}
};
}  // namespace rslcpp_helper_nodes
RCLCPP_COMPONENTS_REGISTER_NODE(rslcpp_helper_nodes::BagRecorder)
