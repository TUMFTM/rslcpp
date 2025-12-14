// Copyright 2025 Simon Sagmeister
#include "rslcpp_dynamic_node_composition/utils.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/has_resource.hpp>
#include <filesystem>
#include <rcpputils/split.hpp>

#include "rslcpp_dynamic_node_composition/types.hpp"
namespace rslcpp::dynamic_composition::utils
{
std::string get_shared_library_path(ComponentDescription const & comp)
{
  std::string amend_index_content;
  std::string base_path;
  // check_package_name_is_valid(package_name);
  ament_index_cpp::get_resource(
    "rclcpp_components", comp.package_name, amend_index_content, &base_path);
  if (amend_index_content.find(comp.component_name) == std::string::npos) {
    throw exceptions::ComponentNotFound(comp);
  }

  std::vector<std::string> lines = rcpputils::split(amend_index_content, '\n', true);
  std::filesystem::path library_path;
  for (const auto & line : lines) {
    std::vector<std::string> parts = rcpputils::split(line, ';');
    // check_split_result<std::string>(parts, 2);
    if (parts[0] == comp.component_name) library_path = parts[1];
  }
  if (!library_path.is_absolute()) {
    return base_path / library_path;
  }
  return library_path;
}
}  // namespace rslcpp::dynamic_composition::utils
