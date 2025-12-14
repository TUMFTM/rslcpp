// Copyright 2025 Simon Sagmeister
#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <thread>
class CommunicationDelaySender : public rclcpp::Node
{
public:
  // Constructor that accepts rclcpp::NodeOptions
  explicit CommunicationDelaySender(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("time_delay_publisher", options)
  {
    // Create publishers for each topic
    rclcpp::PublisherOptions pub_opts_ipc;
    pub_opts_ipc.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    rclcpp::PublisherOptions pub_opts_non_ipc;
    pub_opts_non_ipc.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    ipc_publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("/ipc", 1, pub_opts_ipc);
    rmw_publisher_ =
      this->create_publisher<builtin_interfaces::msg::Time>("/rmw", 1, pub_opts_non_ipc);
    generic_publisher_ =
      this->create_generic_publisher("/generic", "builtin_interfaces/msg/Time", 1, pub_opts_ipc);
    generic_publisher_rmw_ = this->create_generic_publisher(
      "/generic_rmw", "builtin_interfaces/msg/Time", 1, pub_opts_non_ipc);

    // Create a timer to publish messages at 2 Hz
    timer_ipc_ =
      this->create_timer(std::chrono::milliseconds(15), [this]() { publish_time(ipc_publisher_); });

    timer_rmw_ =
      this->create_timer(std::chrono::milliseconds(16), [this]() { publish_time(rmw_publisher_); });
    timer_generic_ = this->create_timer(
      std::chrono::milliseconds(17), [this]() { publish_time(generic_publisher_); });

    timer_generic_rmw_ = this->create_timer(
      std::chrono::milliseconds(18), [this]() { publish_time(generic_publisher_rmw_); });
  }

private:
  void publish_time(rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr & pub)
  {
    auto now = this->get_clock()->now();
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = now.seconds();
    time_msg.nanosec = now.nanoseconds() % 1'000'000'000;

    pub->publish(time_msg);
  }
  void publish_time(rclcpp::GenericPublisher::SharedPtr & pub)
  {
    auto now = this->get_clock()->now();
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = now.seconds();
    time_msg.nanosec = now.nanoseconds() % 1'000'000'000;

    values_serializer_.serialize_message(&time_msg, &time_msg_serialized_);

    pub->publish(time_msg_serialized_);
  }
  // Publishers
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr ipc_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr rmw_publisher_;
  rclcpp::GenericPublisher::SharedPtr generic_publisher_;
  rclcpp::GenericPublisher::SharedPtr generic_publisher_rmw_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_ipc_;
  rclcpp::TimerBase::SharedPtr timer_rmw_;
  rclcpp::TimerBase::SharedPtr timer_generic_;
  rclcpp::TimerBase::SharedPtr timer_generic_rmw_;

  // Serialization
  rclcpp::Serialization<builtin_interfaces::msg::Time> values_serializer_{};
  rclcpp::SerializedMessage time_msg_serialized_{};
};