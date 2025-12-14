# rslcpp_dynamic_node_composition

Dynamic loading of ROS 2 *composable nodes* (rclcpp components) from the command line.

This package is used by `rslcpp_dynamic_job`, but you can also use it directly in your own `main()` if you want runtime-configurable node sets.

## What you get

- `rslcpp::dynamic_composition::ComponentDescription`
- `rslcpp::dynamic_composition::ComponentLoader`
- `parse_components_from_command_line_arguments(argc, argv)`

Header: `include/rslcpp_dynamic_node_composition/composition.hpp`

## CLI format

The parser recognizes component blocks before `--ros-args`:

```bash
--component <package_name> <fully_qualified_component_class> \
  [--component-ros-args <component_ros_args...>] \
--component <package_name> <fully_qualified_component_class> \
  [--component-ros-args <component_ros_args...>] \
--ros-args <global_ros_args...>
```

Notes:

- Each `--component` must have at least `<package_name> <component_class>`.
- If you want per-component arguments, you must start them with `--component-ros-args`.
- The function **modifies** `argc`/`argv`: it strips the parsed component arguments so the remaining args can be passed to `rclcpp`.

## Programmatic use

```cpp
#include <rslcpp_dynamic_node_composition/composition.hpp>

int main(int argc, char ** argv)
{
  auto components = rslcpp::dynamic_composition::parse_components_from_command_line_arguments(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rslcpp::dynamic_composition::ComponentLoader loader;
  auto nodes = loader.load_and_create_nodes(components, options);

  // Add nodes to your executor...
}
```

## How components are found

The loader resolves the shared library path via the ament index (`rclcpp_components` resource) and then instantiates:

- `rclcpp_components::NodeFactoryTemplate<YourComponentClass>`

If a component cannot be found, it throws `rslcpp::dynamic_composition::exceptions::ComponentNotFound`.
