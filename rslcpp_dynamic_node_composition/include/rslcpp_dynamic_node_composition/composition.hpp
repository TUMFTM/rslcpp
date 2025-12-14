// Copyright 2025 Simon Sagmeister
#pragma once
#include <class_loader/class_loader.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "rslcpp_dynamic_node_composition/types.hpp"
#include "rslcpp_dynamic_node_composition/utils.hpp"
namespace rslcpp::dynamic_composition
{

/// @brief Parse a list of component descriptions from command line arguments
/// @note The syntax for the command line arguments is:
/// @note --component <package_name1> <component_name1> --component-ros-args <c1_ros_arg1> ...
/// <c1_ros_argN> --component <package_name2> <component_name2>
/// @param argc Number of command line arguments
/// @param argv Arguments
/// @warning This function modifies the argc and argv parameters to remove the component arguments
/// @return A parsed list of component descriptions
std::vector<ComponentDescription> parse_components_from_command_line_arguments(
  int & argc, char **& argv);
/// @brief Clear the global storage to prevent dangling references
/// @note This is necessary to prevent dangling references to nodes and class loaders
/// @note This function should be called before the program exits or when the nodes are no longer
/// needed.
class ComponentLoader
{
  std::unordered_map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

public:
  /// @brief Load rclcpp components and create the corresponding nodes
  /// @param components A vector of component descriptions
  /// @return A list of node shared ptr to the created nodes
  std::vector<rclcpp::Node::SharedPtr> load_and_create_nodes(
    const std::vector<rslcpp::dynamic_composition::ComponentDescription> & components,
    rclcpp::NodeOptions const & options = rclcpp::NodeOptions());
  /// @brief Load an rclcpp component and create the corresponding node
  /// @param comp The component description
  /// @return rclcpp::Node::SharedPtr The ptr to the created node
  std::shared_ptr<rclcpp::Node> load_component(
    rslcpp::dynamic_composition::ComponentDescription const & component,
    rclcpp::NodeOptions const & options = rclcpp::NodeOptions());
  /// @brief Load rclcpp components and create the corresponding nodes from cli inputs.
  /// @note For a detailed documentation of parsing the cli-args, see the function
  /// `parse_components_from_command_line_arguments`
  /// @param argc Number of command line arguments
  /// @param argv Arguments
  /// @warning This function modifies the argc and argv parameters to remove the component arguments
  /// @return A list of node shared ptr to the created nodes
  std::vector<rclcpp::Node::SharedPtr> load_and_create_nodes(
    int & argc, char **& argv, rclcpp::NodeOptions const & options = rclcpp::NodeOptions());
};
}  // namespace rslcpp::dynamic_composition
