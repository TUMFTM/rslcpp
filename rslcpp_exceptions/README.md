# rslcpp_exceptions

Small header-only exception types used across the `rslcpp` workspace.

Header: `include/rslcpp_exceptions/exceptions.hpp`

## What you get

- `rslcpp::exceptions::NodeNotUsingSimTime`
  - thrown if a node does not have `use_sim_time:=true`
- `rslcpp::exceptions::IntraProcessPublishDisabled`
  - used when intra-process publish is disabled
- `rslcpp::exceptions::NonIntraProcessCommunication`
- `rslcpp::exceptions::UnsupportedTimeDelayFeature`

## Compile-time toggles

The header uses preprocessor macros to decide whether `conditional_throw()` actually throws.

Important: `include/rslcpp_exceptions/exceptions.hpp` currently **defines default values for these macros inside the header**. That means changing behavior typically requires editing the header (or explicitly `#undef`/re-`#define` before including it in a TU).

- `RSLCPP_ACTIVATE_EXCEPTIONS`
  - gates `RSLCPP_INTERNAL_THROW_EXCEPTION(...)`
- `RSLCPP_DEACTIVATE_NONIPC`
- `RSLCPP_DEACTIVATE_UNSUPPORTED`

These switches are intended to allow “warn-only” modes during experimentation.
