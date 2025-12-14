// Copyright 2025 Simon Sagmeister
#include "rslcpp_dynamic_job/dynamic_job.hpp"

#include <rslcpp/rslcpp.hpp>
#include <rslcpp_dynamic_node_composition/composition.hpp>
int main(int argc, char ** argv)
{
  auto components_to_load =
    rslcpp::dynamic_composition::parse_components_from_command_line_arguments(argc, argv);

  // Specify the node options
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.enable_rosout(false);
  options.start_parameter_event_publisher(false);
  options.enable_topic_statistics(false);
  options.use_clock_thread(false);

  auto job = std::make_shared<rslcpp::dynamic_job::DynamicJob>(components_to_load, options);
  auto exit_code = rslcpp::run_job(argc, argv, job);

  return exit_code;
}