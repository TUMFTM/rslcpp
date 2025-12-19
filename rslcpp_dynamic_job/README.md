# rslcpp_dynamic_job

Run composable ROS 2 nodes deterministically *without writing a custom main()*, using `rslcpp`.

This package provides:

- `rslcpp::dynamic_job::DynamicJob`: a `rslcpp::Job` implementation that loads components from CLI.
- `ros2 run rslcpp_dynamic_job dynamic_job`: an executable that parses `--component ...` blocks, creates nodes, and runs them via `rslcpp::run_job`.

## Executable

- `dynamic_job` (see [`executables/dynamic_job.cpp`](./executables/dynamic_job.cpp))

## Usage

### Load components (basic)

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component <package_name> <fully_qualified_component_class> \
  --component <package_name> <fully_qualified_component_class> \
  --ros-args \
    -p use_sim_time:=true \
    -p initial_time_ns_since_epoch:=0
```

### Component-local ROS args

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component rslcpp_time_delay rslcpp_time_delay::TimeDelayLoader \
  --component-ros-args -p delay_config_file_path:=/abs/path/to/delays.csv \
  --component rslcpp_helper_nodes rslcpp_helper_nodes::SimulationMonitor \
  --component-ros-args -p timeout_s:=10 \
  --ros-args -p use_sim_time:=true
```

## Simulation loop parameters

`DynamicJob` reads (optional) parameters at node creation time and forwards them to the internal job backend:

- `initial_time_ns_since_epoch` (int)

These parameters are shown in the basic example above.

## Programmatic control (job backend)

Header: [`include/rslcpp_dynamic_job/backend.hpp`](./include/rslcpp_dynamic_job/backend.hpp)

- `rslcpp::dynamic_job::set_initial_time(rclcpp::Time)`
- `rslcpp::dynamic_job::set_exit_code(uint8_t)` (also marks the job finished)

This is useful for components that want to influence the simulation without owning the main loop.

For concrete usage examples of these functions from within composable nodes, see [`rslcpp_helper_nodes/`](../rslcpp_helper_nodes/) (e.g., `SimulationMonitor` uses `set_exit_code(...)`, and `BagPlayer` uses `set_initial_time(...)`).

## Notes

- The executable configures `rclcpp::NodeOptions` for performance/determinism (IPC enabled, rosout/statistics disabled, no clock thread).
- `rslcpp` requires `use_sim_time:=true` on all nodes.
