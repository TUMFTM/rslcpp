// Copyright 2024 Simon Sagmeister

#include "rslcpp/rslcpp.hpp"

#include <limits>
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
    executor.spin_all(
      std::chrono::hours(
        200 * 365 * 24));  // Large timeout of 200 years to ensure all callbacks are executed since
                           // setting 0 ns to imply infinite timeout does not work with the current
                           // implementation of the events executor.

/// Get the time until the the next timer.
#ifdef RSLCPP__CUSTOM__RCLCPP
    std::chrono::nanoseconds time_until_next_timer = executor.get_time_until_next_timer();
#else
    // Allow for a fallback of 1 ms if we are not compiled with the custom rclcpp version
    std::chrono::nanoseconds time_until_next_timer = std::chrono::milliseconds(1);
#endif

    if (time_until_next_timer == std::chrono::nanoseconds::max()) {
      // No more timers are scheduled, we can end the simulation
      break;
    }

    // Get the time until the next delayed callback
    rslcpp::time_delay::Duration time_until_next_delayed_callback =
      time_delay_backend.get_time_until_next_callback();

    // Use ternary operator to get the minimum duration for the next time step
    rclcpp::Duration time_step_duration =
      (time_until_next_delayed_callback < time_until_next_timer.count())
        ? std::chrono::nanoseconds(time_until_next_delayed_callback)
        : time_until_next_timer;

    // Advance the simulation time by the time until the next timer
    sim_time += time_step_duration;
    // Set the simulation time for all clocks
  }
  auto exit_code = job->get_exit_code();
  rclcpp::shutdown();
  return exit_code;
}
}  // namespace rslcpp
