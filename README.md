# rslcpp | Deterministic Simulations using ROS 2 Nodes

`rslcpp` lets you run a set of ROS 2 nodes in a single-threaded simulation loop with an explicit simulation clock and a fixed time step. The goal is to make simulation runs reproducible and simple to set up: write normal `rclcpp::Node`s, enable `use_sim_time`, and run them either via a small `Job` interface or by dynamically loading composable nodes from the command line.

## Advantages & applications

- **Reproducible simulations across hardware**: fixed-step execution with an explicit simulation clock makes runs easier to reproduce across machines (and across repeated runs on the same machine).
- **Baseline for CI and large-scale scenario execution**: reproducible, compute-independent behavior makes it practical to run the same scenarios in CI or at scale and compare results across runs and machines.
- **No changes to your node logic**: you can typically reuse existing `rclcpp::Node`s as-is—just run them with `use_sim_time:=true` (and, for CLI loading via `rslcpp_dynamic_job`, provide them as composable components).
- **Treat a multi-node system like a single script/function**: run many nodes in one deterministic loop, which is convenient for reinforcement learning / machine learning pipelines (e.g., one process you can start/stop from Python, one exit code, one log directory).
- **Run faster-than-realtime or slower-than-realtime**: simulation time is advanced in discrete steps and is decoupled from wall-clock time, so you can speed up execution for throughput (e.g., training) or let it run slower when algorithms are still too heavy for realtime during early development.
- **Debug whole node graphs with normal breakpoints**: since the system runs in a single process/executor, hitting a breakpoint in one callback naturally pauses the entire simulation without additional synchronization overhead.
- **Controlled timing + delay modeling**: use the included time-delay tooling to inject fixed or measured delays per topic via configuration, without rewriting your nodes.

## Core ideas (what to know)

- **Job**: The central entry point (`rslcpp::Job`) that orchestrates the simulation. It is responsible for:
  - Instantiating all participating nodes.
  - Defining the **initial simulation time** and the fixed **time step size**.
  - Determining the termination condition and the final exit code.
- **Simulation clock**: `rslcpp` takes control of time. It updates the ROS time for all nodes at each step, ensuring that timers and subscriptions execute based on simulation time, not wall-clock time. (Requires `use_sim_time:=true`).
- **Deterministic callback execution**: The runtime drives a custom executor that processes events and advances time in discrete, fixed steps.
- **Optional topic delays**: The framework includes a backend for injecting communication delays. A loader node can read a CSV configuration to apply fixed or probabilistic delays to specific topics.
- **Dynamic composition**: Run existing nodes without writing a custom `main()` function by loading them as ROS 2 components at runtime via the CLI.

## Quickstart (build)

This repo is a ROS 2 workspace (multiple packages). Docker is optional: it should build in any ROS 2 Jazzy environment as well.

### Native (ROS 2 Jazzy installed)

```bash
source /opt/ros/jazzy/setup.bash

# Note: building tests may require extra deps for the vendored rclcpp.
colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

### Docker (convenient, reproducible)

```bash
# Start the container
docker run -it --rm \
  -v "$(pwd)":/dev_ws/src \
  -w /dev_ws \
  ros:jazzy \
  bash

# Everything following will be done inside the container

source /opt/ros/jazzy/setup.bash

# Build the workspace.
# Note: building tests may require extra deps for the vendored rclcpp.
colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 2 minutes: run a deterministic job

The repository ships small example executables in `rslcpp_test`.

```bash
ros2 run rslcpp_test determinism --ros-args -p use_sim_time:=true
ros2 run rslcpp_test time_delay --ros-args -p use_sim_time:=true
ros2 run rslcpp_test communication_delay --ros-args -p use_sim_time:=true
```

## 5 minutes: run *your* nodes (no custom main)

This uses the **Dynamic Job** (`rslcpp_dynamic_job`) to load composable nodes at runtime.

If your nodes are **composable components** (registered with `RCLCPP_COMPONENTS_REGISTER_NODE(...)`), you can run them without writing a custom `main()`:

See `rslcpp_dynamic_job/README.md` for the full Dynamic Job documentation (including per-component arguments and more examples).

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component <package_name> <fully_qualified_component_class> \
  --component <package_name> <fully_qualified_component_class> \
  --ros-args \
    -p use_sim_time:=true \
    -p initial_time_ns_since_epoch:=0 \
    -p time_step_size_ns:=1000000 \
    --params-file <parameter-file>
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
- `rslcpp_rclcpp/`: vendored/forked ROS 2 `rclcpp`, `rclcpp_components`, `rclcpp_action`, and `rclcpp_lifecycle` packages (modified to integrate with the delay backend)

## Notes & constraints

- `rslcpp` expects all nodes to run with `use_sim_time:=true` (it will refuse to run otherwise).
- The intended mode is **single-process** execution (intra-process communication enabled). Mixing in multi-process/DDS communication can reduce determinism.
- Cyclic pub/sub chains that “feed themselves” without any timer or other time-driven trigger can deadlock the simulation (no new events become ready for the executor).
- **Modified rclcpp**: This workspace includes a custom version of `rclcpp` (`rslcpp_rclcpp/`) to enable the time-delay backend. It is API-compatible with standard `rclcpp`, so no code changes are needed in your nodes, but it must be used for the delay features to function.

## References

If you use `rslcpp` in your work, please consider citing our paper. This also contains
more detailed explanation of the concepts and some benchmarks.

```bibtex
TBD
```

## Core developers

- Simon Sagmeister
- Marcel Weinmann
- Phillip Pitschi

Thank also to the students who worked with the framework during their thesis and thus providing valuable input on requirements and design.
