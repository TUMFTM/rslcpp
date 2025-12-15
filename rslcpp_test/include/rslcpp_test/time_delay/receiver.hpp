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
    subscribe_to_topic("/no_delay");
    subscribe_to_topic("/fixed_delay_1");
    subscribe_to_topic("/fixed_delay_2");
    subscribe_to_topic("/measured_delay");
    subscribe_to_topic("/measured_delay_with_skew");
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
      auto delay = now - rclcpp::Time(*msg);
      std::uint64_t delay_us = delay.nanoseconds() / 1'000;  // Convert to microseconds

      // Push the time difference into the correct vector in the map
      delay_buffer_map_[topic_name].push_back(delay_us);
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
      std::cout << "Topic: " << topic << " - Mean: " << mean_delay << " us, Min: " << min_delay
                << " us, Max: " << max_delay << " us, Count: " << delays.size() << std::endl;
    }
  }
};