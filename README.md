# rslcpp | Deterministic Simulations using ROS 2

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Linux-FCC624?logo=linux&logoColor=black)](https://www.linux.org/)

`rslcpp` - the ros simulation library for C++ - lets you run a set of ROS 2 nodes in a single-threaded simulation loop with an explicit simulation clock and a fixed time step. The goal is to make simulation runs reproducible and simple to set up: write normal `rclcpp::Node` classes, enable `use_sim_time`, and run them either via a small `Job` interface or by dynamically loading composable nodes from the command line.

## Advantages & Applications

- **Guaranteed Reproducibility**: Eliminate any hardware variability. By enforcing a fixed time step and explicit scheduling, `rslcpp` ensures that a simulation run produces the exact same result every time—whether on a developer laptop or a CI server.
- **Zero-Touch Integration**: Bring your existing ROS 2 nodes. `rslcpp` works with standard `rclcpp::Node` classes. No custom APIs, wrappers, or refactoring required.
- **Flexible Time Dilation**: Decouple simulation speed from wall-clock time. Run **faster-than-realtime** for high-throughput training (RL/ML) or **slower-than-realtime** to debug heavy algorithms on limited hardware without "falling behind."
- **Atomic Execution for ML/RL**: Treat a complex multi-node system as a single, controllable unit. Start, step, and stop the entire graph synchronously, making it ideal for reinforcement learning loops and Gym environments where the simulation must wait for the agent.
- **Simplified "Pause-the-World" Debugging**: Debugging distributed systems is hard; `rslcpp` makes it easy. Since the system runs in a single process, hitting a breakpoint in *any* node instantly pauses the entire simulation. You can inspect the full system state without timeouts or synchronization issues.
- **Realistic Network Modeling**: Validate system robustness by injecting deterministic delays. Simulate network latency or processing jitter using fixed or probabilistic models, configured via CSV, without changing a line of code.

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

This repository is a standard ROS 2 workspace. You can build it natively on ROS 2 Jazzy or use the provided Docker container for a guaranteed environment.

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

This uses the [`rslcpp_dynamic_job/`](./rslcpp_dynamic_job/) to load composable nodes at runtime.

If your nodes are **composable components** (registered with `RCLCPP_COMPONENTS_REGISTER_NODE(...)`), you can run them without writing a custom `main()`:

See [`rslcpp_dynamic_job/Readme.md`](./rslcpp_dynamic_job/) for the full Dynamic Job documentation (including per-component arguments and more examples).

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

For complex scenarios requiring custom orchestration, programmatic node creation, or specific termination logic, you can implement the `rslcpp::Job` interface directly:

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

See [`rslcpp_test/executables/*.cpp`](./rslcpp_test/executables/) for complete, working examples.

## Repository layout

- [`rslcpp/`](./rslcpp/): core job interface + deterministic simulation loop
- [`rslcpp_dynamic_node_composition/`](./rslcpp_dynamic_node_composition/): CLI parsing + dynamic loading of ROS 2 components
- [`rslcpp_dynamic_job/`](./rslcpp_dynamic_job/): a ready-to-use `Job` that loads components from CLI
- [`rslcpp_time_delay_backend/`](./rslcpp_time_delay_backend/): delay scheduling + delay models (fixed / measured)
- [`rslcpp_time_delay/`](./rslcpp_time_delay/): a loader component that reads a CSV config and programs delays
- [`rslcpp_helper_nodes/`](./rslcpp_helper_nodes/): reusable simulation helper nodes (monitor, bag player/recorder)
- [`rslcpp_test/`](./rslcpp_test/): small executables demonstrating determinism and delays
- [`rslcpp_rclcpp/`](./rslcpp_rclcpp/): vendored/forked ROS 2 `rclcpp`, `rclcpp_components`, `rclcpp_action`, and `rclcpp_lifecycle` packages (modified to integrate with the delay backend)

## Notes & constraints

- `rslcpp` expects all nodes to run with `use_sim_time:=true` (it will refuse to run otherwise).
- The intended mode is **single-process** execution (intra-process communication enabled). Mixing in multi-process/DDS communication can reduce determinism.
- Cyclic pub/sub chains that “feed themselves” without any timer or other time-driven trigger can deadlock the simulation (no new events become ready for the executor).
- **Modified rclcpp**: This workspace includes a custom version of [`rclcpp/`](./rslcpp_rclcpp/) to enable the time-delay backend. It is API-compatible with standard `rclcpp`, so no code changes are needed in your nodes, but it must be used for the delay features to function.

## References

If you use `rslcpp` in your work, please consider citing our paper. This also contains
more detailed explanation of the concepts and some benchmarks.

```bibtex
TBD
```

## Maintainers & Credits

- [Simon Sagmeister](https://github.com/simonsag96)
- [Marcel Weinmann](https://github.com/MarcelWeinmann)
- [Phillip Pitschi](https://github.com/PhillPi)

Thank also to the students who worked with the framework during their thesis and thus providing valuable input on requirements and design.
