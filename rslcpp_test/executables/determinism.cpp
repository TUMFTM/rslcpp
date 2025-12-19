// Copyright 2025 Simon Sagmeister
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rslcpp/rslcpp.hpp>
#include <rslcpp_helper_nodes/monitor.hpp>

#include "rslcpp_test/determinism/determinism_check.hpp"
#include "rslcpp_test/determinism/nodes.hpp"
// ============================================================================
// Region: Using rslcpp
// ============================================================================

class TestJob : public rslcpp::Job
{
public:
  std::uint8_t get_exit_code() override { return exit_code_; }
  bool get_finished() override { return finished_; }
  rclcpp::Time get_initial_time() override { return rclcpp::Time(10, 0, RCL_ROS_TIME); }
  std::vector<rclcpp::Node::SharedPtr> create_and_get_nodes() override
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto node_a = std::make_shared<NodeA>(options);
    auto node_b = std::make_shared<NodeB>(options);
    auto node_c = std::make_shared<NodeC>(options);
    auto node_d = std::make_shared<NodeD>(options);

    auto monitor_node = std::make_shared<rslcpp::helper_nodes::SimulationMonitor>(
      [this](rslcpp::exit_code_t exit_code) {
        finished_ = true;
        exit_code_ = exit_code;
      },
      options);
    monitor_node->set_parameter(rclcpp::Parameter("timeout_s", 100));

    return {node_a, node_b, node_c, node_d, monitor_node};
  }

private:
  bool finished_ = false;
  rslcpp::exit_code_t exit_code_ = 0;
};
// Run the job determinsitically using rslcpp
int main(int argc, char ** argv)
{
  // Use an extra job to deallocate everything.
  {
    auto job = std::make_shared<TestJob>();
    rslcpp::run_job(argc, argv, job);
  }

  // Check the determinism results
  std::cout << std::endl << "================ Determinism Check ================" << std::endl;
  check_final_states();

  // Print a report of the callback execution order
  std::cout << std::endl
            << "================ Callback Execution Report ================" << std::endl;
  print_report();

  // Write the callback execution order to a file in the current directory
  {
    std::ofstream file("callback_execution_order.txt");
    for (const auto & seed : __callback_execution_order__) {
      file << seed << std::endl;
    }
  }

  return 0;
}
