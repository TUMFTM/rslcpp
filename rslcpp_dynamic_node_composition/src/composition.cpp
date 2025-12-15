// Copyright 2025 Simon Sagmeister
#include "rslcpp_dynamic_node_composition/composition.hpp"
namespace rslcpp::dynamic_composition
{
std::vector<rclcpp::Node::SharedPtr> ComponentLoader::load_and_create_nodes(
  const std::vector<rslcpp::dynamic_composition::ComponentDescription> & components,
  rclcpp::NodeOptions const & options)
{
  std::vector<rclcpp::Node::SharedPtr> nodes;
  for (const auto & component : components) {
    nodes.emplace_back(load_component(component, options));
  }
  return nodes;
}
/// @brief Load an rclcpp component and create the corresponding node
/// @param comp The component description
/// @return rclcpp::Node::SharedPtr The ptr to the created node
std::shared_ptr<rclcpp::Node> ComponentLoader::load_component(
  rslcpp::dynamic_composition::ComponentDescription const & component,
  rclcpp::NodeOptions const & options)
{
  std::cout << ">> Loading Component | " << component.component_name << std::endl;
  std::string library_path = rslcpp::dynamic_composition::utils::get_shared_library_path(component);
  if (loaders.find(library_path) == loaders.end()) {
    loaders[library_path] = std::make_unique<class_loader::ClassLoader>(library_path);
  }

  std::string class_name =
    "rclcpp_components::NodeFactoryTemplate<" + component.component_name + ">";

  auto node_factory =
    loaders[library_path]->createInstance<rclcpp_components::NodeFactory>(class_name);

  // Modify the options to override the component arguments
  rclcpp::NodeOptions modified_options = options;
  modified_options.arguments(component.component_arguments);
  // Load the node
  auto wrapper = node_factory->create_node_instance(modified_options);
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
  node_wrappers.push_back(wrapper);

  return node;
}
std::vector<ComponentDescription> parse_components_from_command_line_arguments(
  int & argc, char **& argv)
{
  std::vector<ComponentDescription> components;

  std::size_t i = 0;
  while (i < static_cast<std::size_t>(argc)) {
    // break the loop if we reach the ros args

    if (std::string(argv[i]) == "--ros-args") {
      break;
    }
    if (std::string(argv[i]) == "--component") {
      // Find the subset of arguments specifying the components
      std::size_t k = i + 1;
      while (k < static_cast<std::size_t>(argc)) {
        auto str_arg = std::string(argv[k]);
        if (str_arg == "--ros-args" || str_arg == "--component") {
          break;
        }
        k++;
      }
      std::size_t num_comp_sub_args = k - i - 1;
      if (num_comp_sub_args < 2) {
        throw std::runtime_error(
          "DynamicComposition | '--component' specified without package and class arguments.");
      }
      // Extract the component description
      std::string package_name = argv[i + 1];
      std::string component_name = argv[i + 2];
      std::vector<std::string> component_arguments;
      if (num_comp_sub_args >= 3) {
        if (argv[i + 3] == std::string("--component-ros-args")) {
          // If the first argument after the component is --ros-args, we add it to the component
          // arguments
          component_arguments.push_back("--ros-args");
        } else {
          throw std::runtime_error(
            "DynamicComposition | If you want to add arguments per component, use "
            "'--component-ros-args' after the component name");
        }
      }
      // Add the component arguments
      for (std::size_t j = 4; j <= num_comp_sub_args; ++j) {
        component_arguments.push_back(argv[i + j]);
      }
      components.emplace_back(package_name, component_name, component_arguments);
      i += num_comp_sub_args + 1;  // Move to the next component
      continue;
    }
    i++;
  }
  // Remove the parsed components from the command line arguments
  argv += i;
  argc -= i;
  if (argc <= 0) {
    argc = 0;
    argv = nullptr;  // Reset argv if no arguments left
  }
  return components;
}
}  // namespace rslcpp::dynamic_composition
