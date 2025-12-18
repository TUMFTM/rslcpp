// Copyright 2025 Simon Sagmeister
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rslcpp/rslcpp.hpp>
#include <rslcpp_helper_nodes/monitor.hpp>
#include <rslcpp_test/time_delay/receiver.hpp>
#include <rslcpp_test/time_delay/sender.hpp>
#include <rslcpp_time_delay/time_delay_loader.hpp>
#include <rslcpp_time_delay_backend/delay_backend.hpp>

#include "rclcpp/serialization.hpp"
// ============================================================================
// Region: Using rslcpp
// ============================================================================

class TestJob : public rslcpp::Job
{
  std::uint8_t get_exit_code() override { return exit_code_; }
  bool get_finished() override { return finished_; }
  rclcpp::Time get_initial_time() override { return rclcpp::Time(10, 0, RCL_ROS_TIME); }
  std::vector<rclcpp::Node::SharedPtr> create_and_get_nodes() override
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto sender_node = std::make_shared<TimeDelayPublisher>(options);
    auto reciever_node = std::make_shared<ReceiverNode>(options);
    auto monitor_node = std::make_shared<rslcpp::helper_nodes::SimulationMonitor>(
      [this](rslcpp::exit_code_t exit_code) {
        finished_ = true;
        exit_code_ = exit_code;
      },
      options);
    monitor_node->set_parameter(rclcpp::Parameter("timeout_s", 5));

    auto time_delay_loader = std::make_shared<rslcpp::time_delay::TimeDelayLoader>(options);

    return {sender_node, reciever_node, monitor_node, time_delay_loader};
  }

private:
  bool finished_ = false;
  rslcpp::exit_code_t exit_code_ = 0;
};
// Run the job determinsitically using rslcpp
int main(int argc, char ** argv)
{
  auto job = std::make_shared<TestJob>();
  auto exit_code = rslcpp::run_job(argc, argv, job);
  std::cout << "RSLCPP Job ran with Exit Code: " << static_cast<int>(exit_code) << std::endl;

  return 0;
}
