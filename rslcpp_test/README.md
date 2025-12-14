# rslcpp_test

Small executables demonstrating how to use `rslcpp` and validating deterministic behavior.

This package is meant as both:

- usage examples you can copy/paste into your own code, and
- regression tests you can run in a built workspace.

## Executables

Built from `executables/*.cpp`:

- `communication_delay`
  - minimal `rslcpp::Job` that runs a sender + receiver + monitor
- `determinism`
  - runs a deterministic node network, then prints a callback execution report and writes `callback_execution_order.txt`
- `time_delay`
  - runs a small sender/receiver setup plus the `rslcpp::time_delay::TimeDelayLoader` node

## Run

```bash
ros2 run rslcpp_test communication_delay --ros-args -p use_sim_time:=true
ros2 run rslcpp_test determinism --ros-args -p use_sim_time:=true
ros2 run rslcpp_test time_delay --ros-args -p use_sim_time:=true
```

## Where to look next

- Start with `executables/communication_delay.cpp` for the smallest “hello world” job.
- If you want to run your own components without writing a job, see `rslcpp_dynamic_job`.
