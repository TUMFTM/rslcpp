// Copyright 2025 Simon Sagmeister
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rslcpp/rslcpp.hpp>
#include <rslcpp_helper_nodes/monitor.hpp>

#include "rslcpp_test/communication_delay/receiver.hpp"
#include "rslcpp_test/communication_delay/sender.hpp"
/// @brief Define your own job object to create a an rslcpp simulation
class TestJob : public rslcpp::Job
{
  /// @brief Return the exit code of the simulation. This will be called after stopping the job.
  std::uint8_t get_exit_code() override { return exit_code_; }
  /// @brief Get whether the simulation is finished
  bool get_finished() override { return finished_; }
  /// @brief Return the initial time of the simulation
  rclcpp::Time get_initial_time() override { return rclcpp::Time(10, 0, RCL_ROS_TIME); }
  std::vector<rclcpp::Node::SharedPtr> create_and_get_nodes() override
  {
    rclcpp::NodeOptions options;
    // Use intra process communication
    options.use_intra_process_comms(true);
    // Create the sender and receiver nodes
    auto sender_node = std::make_shared<CommunicationDelaySender>(options);
    auto receiver_node = std::make_shared<ReceiverNode>(options);
    // Use the monitor node from the helper nodes pkg to abort the simulation after 100s
    auto monitor_node = std::make_shared<rslcpp::helper_nodes::SimulationMonitor>(
      [this](rslcpp::exit_code_t exit_code) {
        finished_ = true;
        exit_code_ = exit_code;
      },
      options);
    monitor_node->set_parameter(rclcpp::Parameter("timeout_s", 100));

    // Return the shared ptr to all of the created nodes
    return {sender_node, monitor_node, receiver_node};
  }

private:
  bool finished_ = false;
  rslcpp::exit_code_t exit_code_ = 0;
};
// Run the job determinsitically using rslcpp
int main(int argc, char ** argv)
{
  // Create the job
  auto job = std::make_shared<TestJob>();
  // Run the job using rslcpp
  auto exit_code = rslcpp::run_job(argc, argv, job);
  std::cout << "RSLCPP Job ran with Exit Code: " << static_cast<int>(exit_code) << std::endl;
  return 0;
}
