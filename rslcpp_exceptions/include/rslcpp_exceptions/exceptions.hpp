// Copyright 2025 Simon Sagmeister
#pragma once

#include <stdexcept>
#include <string>

#define RSLCPP_ACTIVATE_EXCEPTIONS
#define RSLCPP_DEACTIVATE_NONIPC
// #define RSLCPP_DEACTIVATE_UNSUPPORTED

#ifdef RSLCPP_ACTIVATE_EXCEPTIONS
#define RSLCPP_INTERNAL_THROW_EXCEPTION(exception) throw exception
#else
#define RSLCPP_INTERNAL_THROW_EXCEPTION(exception)
#endif
namespace rslcpp::exceptions
{
struct NonIntraProcessCommunication : public std::runtime_error
{
  explicit NonIntraProcessCommunication(std::string const & msg) : std::runtime_error(msg) {}
  void conditional_throw()
  {
#ifndef RSLCPP_DEACTIVATE_NONIPC
    RSLCPP_INTERNAL_THROW_EXCEPTION(*this);
#endif
  }
};
struct IntraProcessPublishDisabled : public NonIntraProcessCommunication
{
  explicit IntraProcessPublishDisabled(std::string const & publisher_topic)
  : NonIntraProcessCommunication(
      std::string("Intra process publish is disabled for publisher: ") + publisher_topic)

  {
  }
};
struct UnsupportedTimeDelayFeature : public std::runtime_error
{
  explicit UnsupportedTimeDelayFeature(std::string const & msg) : std::runtime_error(msg) {}
  void conditional_throw()
  {
#ifndef RSLCPP_DEACTIVATE_UNSUPPORTED
    RSLCPP_INTERNAL_THROW_EXCEPTION(*this);
#endif
  }
};
struct NodeNotUsingSimTime : public std::runtime_error
{
  explicit NodeNotUsingSimTime(std::string const & node_name)
  : std::runtime_error(
      std::string("Node '") + node_name +
      "' is not using sim time. Please enable sim time by setting the parameter 'use_sim_time' "
      "to "
      "true.\nYou can set this parameter to true for all nodes by appending '--ros-args -p use_sim_time:=true' to your command")
  {
  }
};
}  // namespace rslcpp::exceptions
