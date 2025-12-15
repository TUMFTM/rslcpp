// Copyright 2025 Simon Sagmeister
#include "rslcpp_helper_nodes/player.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Create the required default constructor to have a composable node
using BagPlayerGeneric = rslcpp::helper_nodes::BagPlayer;
namespace rslcpp_helper_nodes
{
struct BagPlayer : public BagPlayerGeneric
{
  explicit BagPlayer(const rclcpp::NodeOptions & options) : BagPlayerGeneric(options) {}
};
}  // namespace rslcpp_helper_nodes
RCLCPP_COMPONENTS_REGISTER_NODE(rslcpp_helper_nodes::BagPlayer)
