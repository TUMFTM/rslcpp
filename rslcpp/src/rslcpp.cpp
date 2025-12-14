// Copyright 2024 Simon Sagmeister

#include "rslcpp/rslcpp.hpp"

#include <rslcpp_exceptions/exceptions.hpp>
namespace rslcpp
{
exit_code_t run_job(int argc, char ** argv, Job::SharedPtr job)
{
  // Get the time delay backend
  time_delay::CallbackBackend & time_delay_backend = time_delay::CallbackBackend::getInstance();

  // Set domain ID
  rclcpp::InitOptions init_options;

  // Init rclcpp
  rclcpp::init(argc, argv, init_options);

  auto nodes = job->create_and_get_nodes();

  // Check that all nodes correctly use sim time
  for (auto & node : nodes) {
    // node->set_parameter(sim_time_param);
    if (!node->get_parameter("use_sim_time").as_bool()) {
      throw exceptions::NodeNotUsingSimTime(node->get_fully_qualified_name());
    }
  }

  auto context = rclcpp::contexts::get_global_default_context();
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;

  rclcpp::experimental::executors::EventsExecutor executor{
    std::make_unique<rclcpp::experimental::executors::SimpleEventsQueue>(), false,
    executor_options};

  for (auto & node : nodes) {
    executor.add_node(node);
  }

  auto sim_time = job->get_initial_time();
  auto time_step = job->get_time_step_size();

  // Do the simulation loop
  // The second conditions allows to manually break the loop on ctrl + c
  while (!job->get_finished() && rclcpp::ok(context)) {
    // Set the time delay backend
    time_delay_backend.set_time(sim_time.nanoseconds());
    // Set node clocks
    for (auto & node : nodes) {
      rslcpp::set_clock(sim_time, node->get_clock());
    }
    // Execute ready delayed publishers
    time_delay_backend.execute_ready_publishers();

    // Execute all callbacks until none is ready
    // Since we only have a single thread with IPC communication only, it is garantueed that
    // the callbacks are executed in the order they get ready.
    // It is also garantueed that all callbacks that would ever be ready at this time step are
    // executed.
    executor.spin_all(std::chrono::hours(
      200 * 365 * 24));  // Large timeout of 200 years to ensure all callbacks are executed since
                         // setting 0 ns to imply infinite timeout does not work with the current
                         // implementation of the events executor.
    // Advance the simulation time
    sim_time += time_step;
    // Set the simulation time for all clocks
  }
  auto exit_code = job->get_exit_code();
  rclcpp::shutdown();
  return exit_code;
}
}  // namespace rslcpp
