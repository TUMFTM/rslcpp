// Copyright 2025 Simon Sagmeister
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "rslcpp_test/determinism/callbacks.hpp"
#include "rslcpp_test/determinism/determinism_check.hpp"
class NodeA : public rclcpp::Node
{
public:
  explicit NodeA(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("NodeA", options)
  {
  }
  uint64_t get_state() const { return state_; }
  ~NodeA() override { register_final_state(Nodes::NodeA, state_); }

private:
  // Internal state
  uint64_t state_ = 55272744ul;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub1_ =
    this->create_publisher<std_msgs::msg::UInt64>("/t1", 1);
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub2_ =
    this->create_publisher<std_msgs::msg::UInt64>("/t2", 1);

  // Timer
  rclcpp::TimerBase::SharedPtr timer1_ = this->create_timer(
    std::chrono::milliseconds(100),
    [this]() { generic_timer_callback(state_, 11162456753ul, now(), pub1_); });
  rclcpp::TimerBase::SharedPtr timer2_ = this->create_timer(
    std::chrono::milliseconds(50),
    [this]() { generic_timer_callback(state_, 11151784617ul, now(), pub2_); });

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub1_ =
    this->create_subscription<std_msgs::msg::UInt64>(
      "/t5", 1, [this](std_msgs::msg::UInt64::SharedPtr msg) {
        generic_subscription_callback(state_, 22255799981ul, now(), msg, nullptr);
      });
};
class NodeB : public rclcpp::Node
{
public:
  explicit NodeB(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("NodeB", options)
  {
  }
  uint64_t get_state() const { return state_; }
  ~NodeB() override { register_final_state(Nodes::NodeB, state_); }

private:
  // Internal state
  uint64_t state_ = 34983402ul;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub1_ =
    this->create_publisher<std_msgs::msg::UInt64>("/t3", 1);
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub2_ =
    this->create_publisher<std_msgs::msg::UInt64>("/t5", 1);

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub1_ =
    this->create_subscription<std_msgs::msg::UInt64>(
      "/t2", 1, [this](std_msgs::msg::UInt64::SharedPtr msg) {
        generic_subscription_callback(state_, 22229447589ul, now(), msg, pub1_);
      });

  // Timer
  rclcpp::TimerBase::SharedPtr timer_ = this->create_timer(std::chrono::milliseconds(40), [this]() {
    generic_timer_callback(state_, 11111071799ul, now(), pub2_);
  });
};
class NodeC : public rclcpp::Node
{
public:
  explicit NodeC(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("NodeC", options)
  {
  }
  uint64_t get_state() const { return state_; }
  ~NodeC() override { register_final_state(Nodes::NodeC, state_); }

private:
  // Internal state
  uint64_t state_ = 87997653ul;
  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub1_ =
    this->create_publisher<std_msgs::msg::UInt64>("/t4", 1);

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client1_ =
    create_client<std_srvs::srv::Trigger>("/s1");

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub1_ =
    this->create_subscription<std_msgs::msg::UInt64>(
      "/t1", 1, [this](std_msgs::msg::UInt64::SharedPtr msg) {
        generic_subscription_callback_to_service_call(
          state_, 22295316797ul, now(), msg, client1_,
          [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            generic_service_response_callback(state_, 44442759714ul, now(), future);
          });
      });
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub2_ =
    this->create_subscription<std_msgs::msg::UInt64>(
      "/t3", 1, [this](std_msgs::msg::UInt64::SharedPtr msg) {
        generic_subscription_callback(state_, 22227118361ul, now(), msg, pub1_);
      });
};
class NodeD : public rclcpp::Node
{
public:
  explicit NodeD(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("NodeD", options)
  {
  }
  uint64_t get_state() const { return state_; }
  ~NodeD() override { register_final_state(Nodes::NodeD, state_); }

private:
  // Internal state
  uint64_t state_ = 11380831ul;
  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub1_ =
    this->create_publisher<std_msgs::msg::UInt64>("/t5", 1);

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub2_ =
    this->create_subscription<std_msgs::msg::UInt64>(
      "/t4", 1, [this](std_msgs::msg::UInt64::SharedPtr msg) {
        generic_subscription_callback(state_, 2227672095ul, now(), msg, pub1_);
      });

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_ =
    create_service<std_srvs::srv::Trigger>(
      "/s1", [this](
               const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        generic_service_callback(state_, 33396054244ul, now(), request, response);
      });
};