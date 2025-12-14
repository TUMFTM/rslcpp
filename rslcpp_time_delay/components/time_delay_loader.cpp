// Copyright 2025 Simon Sagmeister
#include "rslcpp_time_delay/time_delay_loader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Create the required default constructor to have a composable node
using TimeDelayLoaderGeneric = rslcpp::time_delay::TimeDelayLoader;
namespace rslcpp_time_delay
{
struct TimeDelayLoader : public TimeDelayLoaderGeneric
{
  explicit TimeDelayLoader(const rclcpp::NodeOptions & options) : TimeDelayLoaderGeneric(options) {}
};
}  // namespace rslcpp_time_delay
RCLCPP_COMPONENTS_REGISTER_NODE(rslcpp_time_delay::TimeDelayLoader)
