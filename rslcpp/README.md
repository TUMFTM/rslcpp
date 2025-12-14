# rslcpp

Core deterministic simulation runtime.

This package defines the `rslcpp::Job` interface and the simulation loop `rslcpp::run_job(...)`.

## What you get

- `rslcpp::Job`: a small interface to describe *what to run* and *how time advances*.
- `rslcpp::run_job(argc, argv, job)`: runs your job until finished and returns an exit code.
- Per-tick simulation time update for each node’s ROS clock.

## Core API

- Header: [`include/rslcpp/rslcpp.hpp`](./include/rslcpp/rslcpp.hpp)
  - `class rslcpp::Job` (with `Job::SharedPtr` and `Job::UniquePtr` typedefs)
  - `rslcpp::exit_code_t rslcpp::run_job(int argc, char ** argv, Job::SharedPtr job)`

### `Job` responsibilities

Your implementation provides:

- `create_and_get_nodes()`: create all nodes and return them.
- `get_initial_time()`: initial simulation time (`rclcpp::Time`).
- `get_time_step_size()`: fixed time step (`rclcpp::Duration`).
- `get_finished()`: when to stop.
- `get_exit_code()`: exit code returned after the loop.

## Minimal example

A typical integration looks like:

```cpp
#include <rslcpp/rslcpp.hpp>

class MyJob : public rslcpp::Job {
public:
  std::vector<rclcpp::Node::SharedPtr> create_and_get_nodes() override;
  rclcpp::Time get_initial_time() override;
  rclcpp::Duration get_time_step_size() override;
  bool get_finished() override;
  rslcpp::exit_code_t get_exit_code() override;
};

int main(int argc, char ** argv)
{
  auto job = std::make_shared<MyJob>();
  return rslcpp::run_job(argc, argv, job);
}
```

For complete examples, see [`rslcpp_test/executables/`](../rslcpp_test/executables/).

## Important runtime expectations

- All nodes must have `use_sim_time` enabled.
  - `rslcpp::run_job` checks this at startup and throws if a node does not use sim time.
- Intended usage is a single-process simulation with intra-process communication enabled.

## Related packages

- [`rslcpp_dynamic_job`](../rslcpp_dynamic_job/): ready-made job executable that dynamically loads composable nodes
- [`rslcpp_time_delay`](../rslcpp_time_delay/) + [`rslcpp_time_delay_backend`](../rslcpp_time_delay_backend/): per-topic message delay configuration
- [`rslcpp_helper_nodes`](../rslcpp_helper_nodes/): monitor / rosbag player / rosbag recorder helper nodes
