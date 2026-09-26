# rslcpp_hooks

Observer hooks into the `rslcpp` simulation loop and the callback dispatch of the vendored `rclcpp` executors. They let a plugin observe every executed callback (e.g. to measure execution times) without changing any node.

## API

Header: [`include/rslcpp_hooks/hooks.hpp`](./include/rslcpp_hooks/hooks.hpp)

Two halves, one per direction. `rslcpp` and the vendored `rclcpp` **emit** events; everyone else **registers** a function for the events they care about:

| Event | Emitted by | Register with |
|---|---|---|
| `job_started(nodes)` | `rslcpp::run_job`, after all nodes are added to the executor | `on_job_started` |
| `step_begin(ns)` / `step_end(ns)` | `rslcpp::run_job`, around each simulation step | `on_step_begin` / `on_step_end` |
| `callback_start(info)` / `callback_end(info)` | the vendored `rclcpp` executors, around each subscription, timer, service, client and waitable callback | `on_callback_start` / `on_callback_end` |
| `job_finished()` | `rslcpp::run_job`, after the loop, while all nodes are still alive | `on_job_finished` |

Each `on_*` returns a `Registration` that removes the hook when destroyed, so a hook can never outlive the object it captures. It is `[[nodiscard]]`: dropping it unregisters immediately.

```cpp
class MyPlugin
{
public:
  MyPlugin()
  {
    hooks_.push_back(rslcpp::hooks::on_callback_end(
      [this](const rslcpp::hooks::CallbackInfo & info) { count_[info.entity]++; }));
  }
private:
  std::vector<rslcpp::hooks::Registration> hooks_;
  std::unordered_map<const void *, std::size_t> count_;
};
```

Registering per event, rather than implementing an observer interface, means a plugin interested only in steps costs nothing per callback, and a plain function needs no class at all.

The emitting side pairs the two callback hooks itself:

```cpp
const rslcpp::hooks::CallbackInfo hook_info{rslcpp::hooks::EntityKind::TIMER, timer.get()};
rslcpp::hooks::callback_start(hook_info);
timer->execute_callback(data);
rslcpp::hooks::callback_end(hook_info);
```

`callback_end` hooks run in reverse registration order, so the interval of a hook registered first encloses all later ones. A hook registered between a start and its end sees the unmatched end and has to tolerate it.

`CallbackInfo::entity` is the address of the executed `rclcpp` object (`SubscriptionBase*`, `TimerBase*`, `ServiceBase*`, `ClientBase*` or `Waitable*`), the same address `rclcpp::CallbackGroup::collect_all_ptrs` yields. Hooks map it to nodes themselves; the executor never does a lookup.

## Overhead

With nothing registered, each hook is a call that iterates an empty vector, about 3 ns per callback -- 0.02 % of a run. With a hook registered, the cost is one indirect call per event plus whatever the hook does.

## Relation to the time-delay backend

The callback scopes in the vendored `rclcpp` open **after** `DelayBackend::register_callback_start()`, so that call is not inside the measured interval. Conversely, a `MEASURED` delay is computed from that marker at publish time and therefore contains an observer's start-hook overhead.

A publish with a configured delay is queued and executed by `rslcpp::run_job` between the steps, outside every callback scope, so an observer does not see it as part of the publishing callback.

## Constraints

- Single-threaded by contract, like `rslcpp` itself: nothing is locked.
- Only the vendored `rclcpp` in [`rslcpp_rclcpp/`](../rslcpp_rclcpp/) calls the callback hooks.
