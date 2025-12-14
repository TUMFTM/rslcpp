# rslcpp_dynamic_job

Run composable ROS 2 nodes deterministically *without writing a custom main()*, using `rslcpp`.

This package provides:

- `rslcpp::dynamic_job::DynamicJob`: a `rslcpp::Job` implementation that loads components from CLI.
- `ros2 run rslcpp_dynamic_job dynamic_job`: an executable that parses `--component ...` blocks, creates nodes, and runs them via `rslcpp::run_job`.

## Executable

- `dynamic_job` (see `executables/dynamic_job.cpp`)

## Usage

### Load components

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component <package_name> <fully_qualified_component_class> \
  --component <package_name> <fully_qualified_component_class> \
  --ros-args -p use_sim_time:=true
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
- `time_step_size_ns` (int)

Example:

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component <pkg> <Component> \
  --ros-args \
    -p use_sim_time:=true \
    -p initial_time_ns_since_epoch:=0 \
    -p time_step_size_ns:=1000000
```

## Programmatic control (job backend)

Header: `include/rslcpp_dynamic_job/backend.hpp`

- `rslcpp::dynamic_job::set_initial_time(rclcpp::Time)`
- `rslcpp::dynamic_job::set_time_step_size(rclcpp::Duration)`
- `rslcpp::dynamic_job::set_exit_code(uint8_t)` (also marks the job finished)

This is useful for components that want to influence the simulation without owning the main loop.

## Notes

- The executable configures `rclcpp::NodeOptions` for performance/determinism (IPC enabled, rosout/statistics disabled, no clock thread).
- `rslcpp` requires `use_sim_time:=true` on all nodes.
