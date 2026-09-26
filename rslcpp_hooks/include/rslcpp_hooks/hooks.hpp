// Copyright 2026 Simon Sagmeister
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

// Only forward declarations: the vendored rclcpp depends on this package, not vice versa.
namespace rclcpp
{
class Node;
}  // namespace rclcpp

/// Hooks into the rslcpp simulation loop and the callback dispatch of the vendored rclcpp
/// executors, for observing a simulation (e.g. measuring callback execution times) without
/// touching any node.
///
/// Two halves, one per direction:
///   * rslcpp and the vendored rclcpp EMIT events: job_started(), step_begin(), ...
///   * everyone else REGISTERS functions for the events they care about: on_job_started(),
///     on_step_begin(), ... Each returns a Registration that unregisters when destroyed.
///
/// Registering per event rather than implementing an observer interface means a plugin
/// interested only in steps costs nothing per callback, and a plain function needs no class.
///
/// Everything runs on the single simulation thread, so nothing is locked.
namespace rslcpp::hooks
{
enum class EntityKind : std::uint8_t { SUBSCRIPTION, TIMER, SERVICE, CLIENT, WAITABLE };

struct CallbackInfo
{
  EntityKind kind;
  /// Address of the rclcpp object that is executed: SubscriptionBase*, TimerBase*,
  /// ServiceBase*, ClientBase* or Waitable*. These are the same addresses that
  /// rclcpp::CallbackGroup::collect_all_ptrs yields, so hooks can map them to nodes.
  const void * entity;
};

using NodeList = std::vector<std::shared_ptr<rclcpp::Node>>;
using JobStartedHook = std::function<void(const NodeList &)>;
using StepHook = std::function<void(std::int64_t sim_time_ns)>;
using CallbackHook = std::function<void(const CallbackInfo &)>;
using JobFinishedHook = std::function<void()>;

using HookId = std::uint64_t;
void remove_hook(HookId id);

/// Owns one registration and removes it on destruction, so a hook can never outlive the
/// object it captures. Move-only; `reset()` unregisters early.
class Registration
{
public:
  Registration() = default;
  explicit Registration(HookId id) : id_(id) {}
  ~Registration() { reset(); }
  Registration(Registration && other) noexcept : id_(other.id_) { other.id_ = 0; }
  Registration & operator=(Registration && other) noexcept
  {
    if (this != &other) {
      reset();
      id_ = other.id_;
      other.id_ = 0;
    }
    return *this;
  }
  Registration(const Registration &) = delete;
  Registration & operator=(const Registration &) = delete;

  void reset()
  {
    if (id_ != 0) {
      remove_hook(id_);
      id_ = 0;
    }
  }
  HookId id() const { return id_; }

private:
  HookId id_ = 0;
};

// --- Registering -----------------------------------------------------------------------
// The returned Registration must be kept: dropping it unregisters immediately.

/// After all nodes are created and added to the executor, before the first step.
[[nodiscard]] Registration on_job_started(JobStartedHook hook);
[[nodiscard]] Registration on_step_begin(StepHook hook);
[[nodiscard]] Registration on_step_end(StepHook hook);
/// Around each executed callback. The two must be read as a pair: end hooks run in reverse
/// registration order, so one hook's interval encloses those registered after it. A hook
/// registered between a start and its end sees the unmatched end and must tolerate it.
[[nodiscard]] Registration on_callback_start(CallbackHook hook);
[[nodiscard]] Registration on_callback_end(CallbackHook hook);
/// After the simulation loop has ended, while all nodes are still alive.
[[nodiscard]] Registration on_job_finished(JobFinishedHook hook);

// --- Emitting --------------------------------------------------------------------------
// Called by rslcpp::run_job and the vendored rclcpp executors. Each is a no-op while
// nothing is registered for that event. callback_start() and callback_end() must bracket
// exactly one callback; the emitting site is responsible for pairing them, including when
// the callback throws (in rslcpp a throwing callback ends the simulation anyway).

void job_started(const NodeList & nodes);
void step_begin(std::int64_t sim_time_ns);
void step_end(std::int64_t sim_time_ns);
void callback_start(const CallbackInfo & info);
void callback_end(const CallbackInfo & info);
void job_finished();
}  // namespace rslcpp::hooks
