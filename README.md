# rslcpp

Deterministic, step-based simulation runtime for ROS 2 C++ nodes.

`rslcpp` lets you run a set of ROS 2 nodes in a single-threaded simulation loop with an explicit simulation clock and a fixed time step. The goal is to make simulation runs reproducible and simple to set up: write normal `rclcpp::Node`s, enable `use_sim_time`, and run them either via a small `Job` interface or by dynamically loading composable nodes from the command line.

## Core ideas (what to know)

- **Job**: you provide a `rslcpp::Job` which
  - creates all nodes for the simulation,
  - defines **initial sim time** and **time step size**,
  - decides when the simulation is finished and what exit code to return.
- **Simulation clock**: `rslcpp` overwrites each node’s ROS time every tick (so timers/subscriptions run against simulation time). Nodes must run with `use_sim_time:=true`.
- **Deterministic callback execution**: the runtime drives an events-based executor and advances time in fixed steps.
- **Optional topic delays**: this repo includes a time-delay backend and a small loader node that reads a CSV and applies per-topic delays.
- **Dynamic composition**: you can run your nodes without writing a custom `main()` by loading ROS 2 components at runtime.

## Quickstart (build)

This repo is a ROS 2 workspace (multiple packages). The simplest way to build is inside a ROS container.

```bash
docker run -it --rm \
  -v "$(pwd)":/dev_ws/src \
  -w /dev_ws \
  ros:jazzy \
  bash

source /opt/ros/jazzy/setup.bash

# Build the workspace.
# Note: building tests may require extra deps for the vendored rclcpp.
colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 5 minutes: run a deterministic job

The repository ships small example executables in `rslcpp_test`.

```bash
ros2 run rslcpp_test communication_delay --ros-args -p use_sim_time:=true
ros2 run rslcpp_test determinism --ros-args -p use_sim_time:=true
ros2 run rslcpp_test time_delay --ros-args -p use_sim_time:=true
```

## 5 minutes: run *your* nodes (no custom main)

If your nodes are **composable components** (registered with `RCLCPP_COMPONENTS_REGISTER_NODE(...)`), you can load them via `rslcpp_dynamic_job`:

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component <package_name> <fully_qualified_component_class> \
  --component <package_name> <fully_qualified_component_class> \
  --ros-args -p use_sim_time:=true
```

Per-component parameters/remaps are supported via `--component-ros-args` (see `rslcpp_dynamic_node_composition`):

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component rslcpp_time_delay rslcpp_time_delay::TimeDelayLoader \
  --component-ros-args -p delay_config_file_path:=/abs/path/to/config.csv \
  --component rslcpp_helper_nodes rslcpp_helper_nodes::SimulationMonitor \
  --component-ros-args -p timeout_s:=10 \
  --ros-args -p use_sim_time:=true
```

You can configure the simulation loop itself (initial time + time step) using ROS parameters consumed by `rslcpp_dynamic_job`:

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component <pkg> <Component> \
  --ros-args \
    -p use_sim_time:=true \
    -p initial_time_ns_since_epoch:=0 \
    -p time_step_size_ns:=1000000
```

## Minimal custom integration (write a Job)

If you prefer full control, implement `rslcpp::Job` and call `rslcpp::run_job`:

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

int main(int argc, char ** argv) {
  auto job = std::make_shared<MyJob>();
  return rslcpp::run_job(argc, argv, job);
}
```

See `rslcpp_test/executables/*.cpp` for complete, working examples.

## Repository layout

- `rslcpp/`: core job interface + deterministic simulation loop
- `rslcpp_dynamic_node_composition/`: CLI parsing + dynamic loading of ROS 2 components
- `rslcpp_dynamic_job/`: a ready-to-use `Job` that loads components from CLI
- `rslcpp_time_delay_backend/`: delay scheduling + delay models (fixed / measured)
- `rslcpp_time_delay/`: a loader component that reads a CSV config and programs delays
- `rslcpp_helper_nodes/`: reusable simulation helper nodes (monitor, bag player/recorder)
- `rslcpp_test/`: small executables demonstrating determinism and delays
- `rslcpp_rclcpp/`: vendored/forked ROS 2 `rclcpp`/`rclcpp_components` used by this workspace

## Notes & constraints

- `rslcpp` expects all nodes to run with `use_sim_time:=true` (it will refuse to run otherwise).
- The intended mode is **single-process** execution (intra-process communication enabled). Mixing in multi-process/DDS communication can reduce determinism.
