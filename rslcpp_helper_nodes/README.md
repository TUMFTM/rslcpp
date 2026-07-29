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

Header: [`include/rslcpp_helper_nodes/monitor.hpp`](./include/rslcpp_helper_nodes/monitor.hpp)

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

Header: [`include/rslcpp_helper_nodes/player.hpp`](./include/rslcpp_helper_nodes/player.hpp)

### Parameters

- `bag_file_path` (string, default empty)
- `pub_intervall_us` (int, default `100`)
- `pub_progress` (bool, default `false`)
  - if `true`, publishes playback progress on `/rslcpp/progress`
- `topics` (string array, default empty)
  - if empty/unspecified, all topics of the bag are played
- `start_offset` (double, default `0.0`)
  - seconds to skip from the beginning of the bag
- `qos_overrides` (map, default empty)
  - per topic QoS overrides, see [QoS overrides](#qos-overrides)

### QoS overrides

By default each publisher offers the QoS profile that was recorded in the bag. Individual policies
can be overridden per topic, analogous to `ros2 bag play --qos-profile-overrides-path`, but as
regular node parameters:

```yaml
/**:
  ros__parameters:
    bag_file_path: /path/to/bag
    qos_overrides:
      /sensor/lidar/points:
        reliability: best_effort   # system_default | reliable | best_effort
        durability: transient_local # system_default | volatile | transient_local
        history: keep_last          # system_default | keep_last | keep_all
        depth: 10
        deadline: 0.25              # seconds, or a sec/nsec pair as shown below
        lifespan:
          sec: 2
          nsec: 500000000
        liveliness: automatic       # system_default | automatic | manual_by_topic
        liveliness_lease_duration: 5.0
        avoid_ros_namespace_conventions: false
```

- Only the listed policies are overridden, everything else stays as recorded.
- The extra `publisher` level of the [rclcpp QoS overriding](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-profile-overrides)
  feature is accepted as well, i.e. `qos_overrides./topic.publisher.reliability` works too.
- An unknown policy name or value aborts the startup instead of silently playing with the wrong QoS.
- An override for a topic that is not played only produces a warning.

### What it does

- Opens a rosbag and creates a `GenericPublisher` for each recorded topic.
- Publishes messages when their bag timestamp is earlier than the current simulation clock.
- Sets the simulation initial time via `rslcpp::dynamic_job::set_initial_time(...)` based on the first message.
- When `pub_progress` is enabled, publishes the elapsed time relative to the bag length on `/rslcpp/progress` (`std_msgs/msg/Float32`, a value in `[0, 1]`).

## BagRecorder

Header: [`include/rslcpp_helper_nodes/recorder.hpp`](./include/rslcpp_helper_nodes/recorder.hpp)

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
- For a working monitor example, see [`rslcpp_test/executables/`](../rslcpp_test/executables/).
