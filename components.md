# ROS 2 Components

Components, also known as *composable nodes*, or *nodelets* in ROS, are a simple way to manage an entire, distributed control architecture in a simplified and modular way from the host OS point of view. In essence, a component is a shared library that compiles a ROS 2 node class. It can then be loaded and unloaded, at runtime, inside a *container* process, in which runs a *component manager* node. For a description of this framwework, as well as a list of commands, see the [official documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html).

What follows is a description of the steps to take to enable the use of components in a ROS 2 package. In the following, the terms *component*, *composable nodes*, and *plugin* are used interchangeably.

**Be aware that composition is a C++-only feature!**

## Codebase organization

To organize the code for composition, some best practices should be followed.

First, the code for the component should be separated from the rest, *i.e.*, its source files and headers should not mix with other code, pertaining for example to some application that also has to be compiled. So, if you want to also build an executable that runs the same node, you should write its `main` function in a separate file.

## Dependencies

Just add `rclcpp_components` in both your `package.xml` and `CMakeLists.txt` files accordingly.

## Node class source code

The following rules should be followed while writing source code for a component:

1. The node class should be defined in a namespace that has the same name of the package. This is to avoid conflicts with plugin names in other packages.
2. The constructor of the node class should take only one argument, of type `const rclcpp::NodeOptions &`.
3. At the end of one of the source files where the class is defined, *e.g.*, the file where the constructor is implemented, one should place the following macro invocation to correctly register the plugin with `pluginlib`:

```cpp
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(YourNamespace::YourNodeClass)
```

## CMake build configuration

The node class should build into a shared library, using the CMake `add_library` command. The following is an example of how to do this, taken from one of the examples:

```cmake
add_library(pub SHARED src/pub.cpp)
target_compile_definitions(pub PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pub
  rclcpp
  rclcpp_components
  std_msgs)
```

The `COMPOSITION_BUILDING_DLL` macro is used to define a symbol that is used to export the symbols of the shared library on Windows. This is not necessary on Linux, but it does not hurt to have it there.

The component should subsequently be registered within ament with the following command, taken as well from one of the examples, note the target name and namespace of the node class:

```cmake
rclcpp_components_register_nodes(pub "pub_sub_components::Publisher")
```

Finally, you need to install your target library and header files. This is done with the complete version of the `install` command. Consider the following example, taken from the same example package:

```cmake
install(TARGETS pub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

## Launch files

You may add to the `LaunchDescription` a `launch_ros.actions.ComposableNodeContainer` action, which will load a component manager node in a new process. To specify which components to immediately load into it, use the `launch_ros.descriptions.ComposableNode` description. Both take arguments that are very close to what you would pass to a `Node` action.

Consider the following example, taken from the example package, in which two components are loaded inside the same container:

```python
container = ComposableNodeContainer(
    name='pub_sub_container',
    namespace='pub_sub_components',
    package='rclcpp_components',
    executable='component_container',
    emulate_tty=True,
    output='both',
    log_cmd=True,
    composable_node_descriptions=[
        ComposableNode(
            package='pub_sub_components',
            plugin='pub_sub_components::Publisher',
            name='publisher_node',
            namespace='pub_sub_components',
            parameters=[],
            extra_arguments=[{'use_intra_process_comms': True}]),
        ComposableNode(
            package='pub_sub_components',
            plugin='pub_sub_components::Subscriber',
            name='subscriber_node',
            namespace='pub_sub_components',
            parameters=[],
            extra_arguments=[{'use_intra_process_comms': True}])
    ]
)
```

The `container` action would then be added to the `LaunchDescription` to be returned.

The `parameters` argument can **either** be a dictionary of parameters **or** a config file.

The `use_intra_process_comms` extra argument is necessary to enable intra-process communication based on shared memory between the components, to optimize communication performance bypassing the RMW. It should always be specified when using components.

## Config files

For some reason, the Launch System is less capable of resolving node names when processing config files for components. This is why it is recommended to **enter full node names in config files**, *i.e.*, the full namespace and node name, as in the following example:

```yaml
/namespace/node_name:
  param_name: value
```

This will ensure that the parameter values are correctly loaded and set for a component.

Of course, the node namespace and name should correspond to those specified when launching the component.

## Feedback

If you have any questions or suggestions, please open an issue or contact us here on GitHub.

---

## License

This work is licensed under the Apache 2.0 License. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2024, Intelligent Systems Lab, University of Rome Tor Vergata
