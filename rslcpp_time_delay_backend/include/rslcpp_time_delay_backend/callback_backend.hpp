// Copyright 2025 Simon Sagmeister
#pragma once
#include <list>
#include <utility>

#include "rslcpp_time_delay_backend/types.hpp"
namespace rslcpp::time_delay
{
// Use a meyers singleton since it is thread safe
class CallbackBackend
{
  class DelayedPublish
  {
  public:
    DelayedPublish(Time callback_execution_time, DelayedCallable && callable)
    : callable_(std::move(callable)), callback_execution_time_(callback_execution_time)
    {
    }
    void invoke() { callable_.invoke(); }
    Time get_callback_execution_time() const { return callback_execution_time_; }
    // Delete copy constructor and copy assignemnt operator
    DelayedPublish(const DelayedPublish &) = delete;
    DelayedPublish & operator=(const DelayedPublish &) = delete;

    // Default move constructor and move assignment operator
    DelayedPublish(DelayedPublish &&) noexcept = default;
    DelayedPublish & operator=(DelayedPublish &&) noexcept = default;

  private:
    DelayedCallable callable_;
    Time callback_execution_time_{};
  };

public:
  static CallbackBackend & getInstance()
  {
    static CallbackBackend instance;
    return instance;
  }

private:
  CallbackBackend() = default;
  ~CallbackBackend() = default;
  CallbackBackend(const CallbackBackend &) = delete;
  CallbackBackend & operator=(const CallbackBackend &) = delete;

  // Actual API
public:
  void add_delayed_callable(DelayedCallable && callable);
  void execute_ready_publishers();
  void set_time(Time current_time);
  Duration get_time_until_next_callback();

private:
  Time current_time_{0};
  std::list<DelayedPublish> callables_{};
};
}  // namespace rslcpp::time_delay
