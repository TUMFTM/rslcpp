# rslcpp_helper_nodes

Reusable nodes that are convenient in `rslcpp` simulations.

This package ships **composable components** (and corresponding executables) for:

- simulation monitoring / timeout + early exit
- rosbag playback (generic publisher)
- rosbag recording (generic subscription)

## Components & executables

Built from the `rslcpp_helper_nodes_components` library:

- `rslcpp_helper_nodes::SimulationMonitor` → `simulation_monitor_component`
- `rslcpp_helper_nodes::BagPlayer` → `bag_player_component`
- `rslcpp_helper_nodes::BagRecorder` → `bag_recorder_component`

## SimulationMonitor

Header: `include/rslcpp_helper_nodes/monitor.hpp`

### What it does

- Subscribes to `/rslcpp/error_code` (`std_msgs/msg/UInt8`). When a message arrives, it triggers the abort callback.
- A timer checks for a simulation timeout and triggers abort with exit code `1`.

### Parameter

- `timeout_s` (int, default `3`)

### Usage

With `rslcpp_dynamic_job`:

```bash
ros2 run rslcpp_dynamic_job dynamic_job \
  --component rslcpp_helper_nodes rslcpp_helper_nodes::SimulationMonitor \
  --component-ros-args -p timeout_s:=10 \
  --ros-args -p use_sim_time:=true
```

## BagPlayer

Header: `include/rslcpp_helper_nodes/player.hpp`

### Parameters

- `bag_file_path` (string, default empty)
- `pub_intervall_us` (int, default `100`)

### What it does

- Opens a rosbag and creates a `GenericPublisher` for each recorded topic.
- Publishes messages when their bag timestamp is earlier than the current simulation clock.
- Sets the simulation initial time via `rslcpp::dynamic_job::set_initial_time(...)` based on the first message.

## BagRecorder

Header: `include/rslcpp_helper_nodes/recorder.hpp`

### Parameters

- `log_dir` (string, default `rslcpp/logs`)
- `ignore_topic_list` (string array)
- `record_topic_list` (string array)
  - if empty/unspecified, records all topics
- `topic_scan_period_s` (double, default `0.1`)

### What it does

- Periodically scans available topics and subscribes using `GenericSubscription`.
- Writes serialized messages to an MCAP bag using the node’s clock (`this->now()`).

## Notes

- These helpers are designed for the **single-process** simulation mode used by `rslcpp`.
- For a working monitor example, see `rslcpp_test/executables/*.cpp`.
