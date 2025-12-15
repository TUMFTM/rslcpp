// Copyright 2025 Simon Sagmeister
#pragma once

#include <string>

#include "rslcpp_dynamic_node_composition/types.hpp"
namespace rslcpp::dynamic_composition::utils
{
std::string get_shared_library_path(ComponentDescription const & comp);
}  // namespace rslcpp::dynamic_composition::utils
