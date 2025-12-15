// Copyright 2025 Simon Sagmeister
#pragma once

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <iostream>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>
class ReceiverNode : public rclcpp::Node
{
public:
  explicit ReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("receiver_node", options)
  {
    // Initialize subscriptions for all topics
    subscribe_to_topic("/ipc");
    subscribe_to_topic("/rmw");
    subscribe_to_topic("/generic");
    subscribe_to_topic("/generic_rmw");
  }
  ~ReceiverNode()
  {
    // Print the delay report on destruction
    print_delay_report();
  }

private:
  // Delay buffer map: topic name -> vector of delay times in microseconds
  std::unordered_map<std::string, std::vector<std::uint64_t>> delay_buffer_map_;

  // Map to store subscriptions: topic name -> subscription
  std::unordered_map<std::string, rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr>
    subscriptions_;
  // Subscribe to a topic and set up the callback
  void subscribe_to_topic(const std::string & topic_name)
  {
    auto callback = [this, topic_name](const builtin_interfaces::msg::Time::SharedPtr msg) {
      // Get the current time
      auto now = this->get_clock()->now();

      // Calculate the time difference in microseconds
      auto send_timestamp = rclcpp::Time(*msg);

      auto delay = now - rclcpp::Time(*msg);
      std::uint64_t delay_ns = delay.nanoseconds();  // Convert to microseconds

      if (delay_ns != 0) {
        std::cout << "Delay for topic " << topic_name << ": " << delay_ns << " ns" << std::endl;
      }

      // Push the time difference into the correct vector in the map
      delay_buffer_map_[topic_name].push_back(delay_ns);
    };

    // Create the subscription and store it in the map
    subscriptions_[topic_name] =
      this->create_subscription<builtin_interfaces::msg::Time>(topic_name, 1, callback);
  }
  // Print the delay report
  void print_delay_report()
  {
    std::cout << "Delay Report:" << std::endl;
    for (const auto & [topic, delays] : delay_buffer_map_) {
      if (delays.empty()) {
        std::cout << "Topic: " << topic << " - No data received." << std::endl;
        continue;
      }

      // Calculate mean, min, and max
      auto min_delay = *std::min_element(delays.begin(), delays.end());
      auto max_delay = *std::max_element(delays.begin(), delays.end());
      auto mean_delay = std::accumulate(delays.begin(), delays.end(), 0ULL) / delays.size();

      // Print the report for the topic
      std::cout << "Topic: " << topic << " - Mean: " << mean_delay << " ns, Min: " << min_delay
                << " ns, Max: " << max_delay << " ns, Count: " << delays.size() << std::endl;
    }
  }
};