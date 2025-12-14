# rslcpp_time_delay

CSV-driven configuration of per-topic message delays for `rslcpp` simulations.

This package provides a composable node that reads a delay config CSV and programs the global `rslcpp::time_delay::DelayBackend` singleton.

## Component

- Library: `rslcpp_time_delay_components`
- Component class: `rslcpp_time_delay::TimeDelayLoader`
- Component executable: `time_delay_loader_component`

Implementation:

- [`include/rslcpp_time_delay/time_delay_loader.hpp`](./include/rslcpp_time_delay/time_delay_loader.hpp)
- [`components/time_delay_loader.cpp`](./components/time_delay_loader.cpp)

## Parameters

Declared by the node `TimeDelayLoader`:

- `delay_config_file_path` (string)
  - defaults to `${share}/config/ExampleDelayConfig.csv`
- `global_measurement_skew` (double, default `1.0`)

## CSV format

See [`config/ExampleDelayConfig.csv`](./config/ExampleDelayConfig.csv):

| Column | Description |
|--------|-------------|
| `Topic Name` | The ROS topic name |
| `Delay Type (FIXED/MEASURED)` | Either `FIXED` or `MEASURED` |
| `Fixed Delay_ms` | Delay in milliseconds (for `FIXED` type; converted to nanoseconds internally) |
| `Measured Delay Skew` | Unitless multiplier (for `MEASURED` type) |

## Usage with `rslcpp_dynamic_job`

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component rslcpp_time_delay rslcpp_time_delay::TimeDelayLoader \
  --component-ros-args -p delay_config_file_path:=/abs/path/to/delays.csv \
  --ros-args -p use_sim_time:=true
```

## Notes

- This config affects publish timing inside this workspace’s single-process simulation runtime.
- For a complete working example, run `ros2 run rslcpp_test time_delay --ros-args -p use_sim_time:=true`.
