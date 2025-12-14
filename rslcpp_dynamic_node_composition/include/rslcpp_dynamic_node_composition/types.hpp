// Copyright 2025 Simon Sagmeister
#pragma once
#include <stdexcept>
#include <string>
#include <vector>
namespace rslcpp::dynamic_composition
{
struct ComponentDescription
{
  std::string package_name;
  std::string component_name;
  std::vector<std::string> component_arguments{};
  ComponentDescription(
    std::string const & pkg_name, std::string const & cmpnt_name,
    std::vector<std::string> const & args)
  : package_name(pkg_name), component_name(cmpnt_name), component_arguments(args)
  {
  }
};
namespace exceptions
{
struct ComponentNotFound : public std::runtime_error
{
  explicit ComponentNotFound(ComponentDescription const & cmpnt)
  : std::runtime_error(
      std::string("DynamicComposition | Component not found! | Package: " + cmpnt.package_name) +
      " -> Component: " + cmpnt.component_name)
  {
  }
};
}  // namespace exceptions
}  // namespace rslcpp::dynamic_composition
