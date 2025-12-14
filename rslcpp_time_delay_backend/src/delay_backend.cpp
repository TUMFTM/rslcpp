// Copyright 2025 Simon Sagmeister
#include "rslcpp_time_delay_backend/delay_backend.hpp"
namespace rslcpp::time_delay
{
Enums::DelayType DelayBackend::get_topic_delay_type(const std::string & topic_name)
{
  if (auto search = topic_delay_types_.find(topic_name); search != topic_delay_types_.end()) {
    return search->second;
  }

  return Enums::DelayType::NO_DELAY;
}
void DelayBackend::set_topic_delay_type(const std::string & topic_name, Enums::DelayType delay_type)
{
  topic_delay_types_[topic_name] = delay_type;
}
void DelayBackend::set_fixed_topic_delay(const std::string & topic_name, Time delay)
{
  set_topic_delay_type(topic_name, Enums::DelayType::FIXED);
  topic_fixed_delay_[topic_name] = delay;
}
void DelayBackend::register_callback_start() { callback_start_ = std::chrono::steady_clock::now(); }
Duration DelayBackend::get_delay(const std::string & topic_name)
{
  switch (get_topic_delay_type(topic_name)) {
    case Enums::DelayType::FIXED:
      return topic_fixed_delay_[topic_name];
    case Enums::DelayType::MEASURED_EXECUTION_TIME:
      return global_measurement_skew_ * topic_measurement_skew_[topic_name] *
             std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now() - callback_start_)
               .count();
    case Enums::DelayType::NO_DELAY:
    default:
      return 0;
  }
}
void DelayBackend::set_measured_topic_delay_skew(
  const std::string & topic_name, double topic_measurement_skew)
{
  set_topic_delay_type(topic_name, Enums::DelayType::MEASURED_EXECUTION_TIME);
  topic_measurement_skew_[topic_name] = topic_measurement_skew;
}
void DelayBackend::set_global_measurement_skew(double global_measurement_skew)
{
  global_measurement_skew_ = global_measurement_skew;
}
}  // namespace rslcpp::time_delay
