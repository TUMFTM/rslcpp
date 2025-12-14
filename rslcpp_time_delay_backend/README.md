# rslcpp_time_delay_backend

Core primitives for scheduling delayed publishes and computing per-topic delay values.

This package is used by the rest of the workspace to model communication delay **inside a single-process simulation**.

## What you get

- `rslcpp::time_delay::CallbackBackend`
  - stores a time-ordered list of delayed callables
  - `set_time(now)` and `execute_ready_publishers()`
- `rslcpp::time_delay::DelayBackend`
  - per-topic delay model and parameters
  - supports (see `Enums::DelayType` in [`types.hpp`](./include/rslcpp_time_delay_backend/types.hpp)):
    - `NO_DELAY` – no artificial delay
    - `FIXED` – constant delay per topic (configured via CSV as `FIXED`)
    - `MEASURED_EXECUTION_TIME` – delay based on callback execution time (configured via CSV as `MEASURED`)

Headers:

- [`include/rslcpp_time_delay_backend/callback_backend.hpp`](./include/rslcpp_time_delay_backend/callback_backend.hpp)
- [`include/rslcpp_time_delay_backend/delay_backend.hpp`](./include/rslcpp_time_delay_backend/delay_backend.hpp)
- [`include/rslcpp_time_delay_backend/types.hpp`](./include/rslcpp_time_delay_backend/types.hpp)

## How it is used in this repo

- The `rslcpp` simulation loop calls `CallbackBackend::set_time(sim_time)` once per tick and then calls `CallbackBackend::execute_ready_publishers()`.
- The vendored `rclcpp` in this workspace integrates with the backend by scheduling publishes via `CallbackBackend` using a delay computed from `DelayBackend` for the publisher topic.

## Delay configuration API

- Fixed delay per topic:
  - `DelayBackend::set_fixed_topic_delay(topic_name, delay_ns)`
- Measured execution-time-based delay:
  - `DelayBackend::set_measured_topic_delay_skew(topic_name, skew)`
  - `DelayBackend::set_global_measurement_skew(skew)`

## Related package

- [`rslcpp_time_delay`](../rslcpp_time_delay/README.md): provides a composable node that loads a CSV config and programs `DelayBackend`.
