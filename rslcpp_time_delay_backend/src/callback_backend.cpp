// Copyright 2025 Simon Sagmeister
#include "rslcpp_time_delay_backend/callback_backend.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
namespace rslcpp::time_delay
{
void CallbackBackend::add_delayed_callable(DelayedCallable && delayed_callable)
{
  Time callback_time = current_time_ + delayed_callable.get_callback_delay();
  if (callback_time <= current_time_) {
    // Directly execute the callable if the delay is zero
    // Avoids a comparison in the publisher
    delayed_callable.invoke();
    return;
  }

  auto insertion_pos =
    std::find_if_not(callables_.begin(), callables_.end(), [callback_time](const auto & elem) {
      return elem.get_callback_execution_time() <= callback_time;
    });
  callables_.emplace(insertion_pos, DelayedPublish(callback_time, std::move(delayed_callable)));
}
void CallbackBackend::execute_ready_delayed_callables()
{
  // Loop through all avaialable callbacks
  while (!callables_.empty()) {
    auto & front = callables_.front();
    if (front.get_callback_execution_time() > current_time_) {
      break;
    }
    front.invoke();
    callables_.pop_front();
  }
}
void CallbackBackend::set_time(Time current_time) { current_time_ = current_time; }
Duration CallbackBackend::get_time_until_next_delayed_callable()
{
  if (callables_.empty()) {
    return std::numeric_limits<Duration>::max();
  }
  Time next_callback_time = callables_.front().get_callback_execution_time();
  if (next_callback_time <= current_time_) {
    return Duration(0);
  }
  return next_callback_time - current_time_;
}
}  // namespace rslcpp::time_delay
