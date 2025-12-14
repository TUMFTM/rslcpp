// Copyright 2025 Simon Sagmeister
#pragma once
#include <chrono>
#include <functional>
#include <list>
#include <string>
#include <unordered_map>
#include <utility>

#include "rslcpp_time_delay_backend/types.hpp"
namespace rslcpp::time_delay
{
// Use a meyers singleton since it is thread safe
class DelayBackend
{
public:
  static DelayBackend & getInstance()
  {
    static DelayBackend instance;
    return instance;
  }

private:
  DelayBackend() = default;
  ~DelayBackend() = default;
  DelayBackend(const DelayBackend &) = delete;
  DelayBackend & operator=(const DelayBackend &) = delete;

  // Actual API
public:
  Enums::DelayType get_topic_delay_type(const std::string & topic_name);
  void set_measured_topic_delay_skew(
    const std::string & topic_name, double topic_measurement_skew = 1.0);
  void set_global_measurement_skew(double global_measurement_skew);
  void set_fixed_topic_delay(const std::string & topic_name, Time delay);
  void register_callback_start();
  Duration get_delay(const std::string & topic_name);

private:
  void set_topic_delay_type(const std::string & topic_name, Enums::DelayType delay_type);

private:
  decltype(std::chrono::steady_clock::now()) callback_start_ = std::chrono::steady_clock::now();
  std::unordered_map<std::string, Enums::DelayType> topic_delay_types_{};
  std::unordered_map<std::string, Time> topic_fixed_delay_{};
  std::unordered_map<std::string, double> topic_measurement_skew_{};
  double global_measurement_skew_ = 1.0;
};
}  // namespace rslcpp::time_delay
