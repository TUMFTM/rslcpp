// Copyright 2025 Simon Sagmeister
#pragma once
#include <cstdint>
#include <memory>
namespace rslcpp::time_delay
{
using Time = std::uint64_t;
using Duration = std::int64_t;
namespace Enums
{
enum DelayType { NO_DELAY, FIXED, MEASURED_EXECUTION_TIME };
};
// region Type Erase for the delayed callbackes
struct DelayedCallableBase
{
  virtual ~DelayedCallableBase() = default;
  virtual void invoke() = 0;  // Pure virtual function to invoke the callable
};
// This is non copy able
template <typename CallableT>
class DelayedCallableWrapper : public DelayedCallableBase
{
public:
  explicit DelayedCallableWrapper(CallableT callable) : callable_(std::move(callable)) {}
  void invoke() override
  {
    callable_();  // Call the stored callable
  }
  // Delete copy and operators
  DelayedCallableWrapper(const DelayedCallableWrapper &) = delete;
  DelayedCallableWrapper & operator=(const DelayedCallableWrapper &) = delete;

private:
  CallableT callable_;
};
class DelayedCallable
{
public:
  template <typename CallableT>
  explicit DelayedCallable(Duration callback_delay_ns, CallableT callable)
  : callable_(std::make_unique<DelayedCallableWrapper<CallableT>>(std::move(callable))),
    callback_delay_ns_(callback_delay_ns)
  {
  }
  void invoke()
  {
    callable_->invoke();  // Delegate to the stored callable
  }
  Time get_callback_delay() const { return callback_delay_ns_; }
  // Delete copy operators
  DelayedCallable(const DelayedCallable &) = delete;
  DelayedCallable & operator=(const DelayedCallable &) = delete;
  // Default the move operators
  DelayedCallable(DelayedCallable &&) noexcept = default;
  DelayedCallable & operator=(DelayedCallable &&) noexcept = default;

private:
  std::unique_ptr<DelayedCallableBase> callable_{nullptr};
  Duration callback_delay_ns_{0};
};
// endregion

}  // namespace rslcpp::time_delay
