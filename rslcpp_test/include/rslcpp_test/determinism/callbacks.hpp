// Copyright 2025 Simon Sagmeister
#pragma once
#include <chrono>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "rslcpp_test/determinism/callback_order_analysis.hpp"
#include "rslcpp_test/determinism/common.hpp"
inline void random_sleep()
{
  static thread_local std::mt19937 generator(std::random_device{}());
  std::uniform_int_distribution<int> distribution(1, 20);
  int sleep_duration_exponent = distribution(generator);
  std::this_thread::sleep_for(
    std::chrono::milliseconds(20) * exp(sleep_duration_exponent) / exp(20));
}
// Used to track executed callbacks to ensure each is printed exactly once
void generic_timer_callback(
  uint64_t & state, uint64_t callback_seed, rclcpp::Time current_time,
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher)
{
  register_callback(callback_seed);

  random_sleep();

  state = hash_combine(state, callback_seed);
  state = hash_combine(state, current_time.nanoseconds());
  std_msgs::msg::UInt64 msg;
  msg.data = state;
  publisher->publish(msg);
}
void generic_subscription_callback(
  uint64_t & state, uint64_t callback_seed, rclcpp::Time current_time,
  std_msgs::msg::UInt64::SharedPtr msg,
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher)
{
  register_callback(callback_seed);

  random_sleep();

  state = hash_combine(state, callback_seed);
  state = hash_combine(state, current_time.nanoseconds());
  state = hash_combine(state, msg->data);
  // Check if publisher is nullptr
  if (publisher == nullptr) {
    return;
  }
  std_msgs::msg::UInt64 out_msg;
  out_msg.data = state;
  publisher->publish(out_msg);
}
void generic_service_response_callback(
  uint64_t & state, uint64_t callback_seed, rclcpp::Time current_time,
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  register_callback(callback_seed);

  state = hash_combine(state, callback_seed);
  state = hash_combine(state, current_time.nanoseconds());
  state = hash_combine(state, std::stoull(future.get()->message));
  random_sleep();
}
void generic_service_callback(
  uint64_t & state, uint64_t callback_seed, rclcpp::Time current_time,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  register_callback(callback_seed);
  random_sleep();

  state = hash_combine(state, callback_seed);
  state = hash_combine(state, current_time.nanoseconds());

  // Take the state hash, convert it to string and write it into the response
  response->message = std::to_string(state);
  response->success = true;
}
void generic_subscription_callback_to_service_call(
  uint64_t & state, uint64_t callback_seed, rclcpp::Time current_time,
  std_msgs::msg::UInt64::SharedPtr msg, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
  std::function<void(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture)> service_response_cb)
{
  register_callback(callback_seed);

  random_sleep();

  state = hash_combine(state, callback_seed);
  state = hash_combine(state, current_time.nanoseconds());
  state = hash_combine(state, msg->data);

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(req, service_response_cb);
}
