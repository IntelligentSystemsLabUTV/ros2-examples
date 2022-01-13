# ROS 2 Interfaces

## Packaging

It is better, and nowadays customary, to create entire packages made only of interfaces. These could be related to single parts of a project or to an entire one, could be of only one kind or many, the choice is yours. Including interfaces in packages that are also made of code that uses them is a bad practice: usually these interfaces are needed by more than one package in order to establish some sort of communication between nodes. This way, you can list interfaces as dependencies of the packages that use them.

In order to create an interfaces-only package, you have to do the following:

1. Create a package specifying no build type (should default to ament_cmake) and no dependencies. **Its name should be _PACKAGE_interfaces_, to clarify its purpose.**

2. Remove _include/_ and _src/_.

3. Create directories for the kinds of interfaces you want to add: _msg_, _srv_, _action_.

4. In _package.xml_, add the following lines:

   ```xml
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <depend>action_msgs</depend> <!--Only for actions!-->
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

5. In _CMakeLists.txt_ you can remove many things, but the only necessary modification is the following line:

   ```cmake
   find_package(rosidl_default_generators REQUIRED)
   ```

   Then you have to add files to generate interfaces from with _rosidl_:

   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
       "msg/file1.msg"
       ...
       "srv/fileN.srv"
       ...
       "action/fileM.action"
       ...
   	# And so on...
   )
   ```

6. **If using actions**, the following dependencies are required:
    - *rclcpp_action*
    - Interfaces package where actions are specified.

Then you can build the package with colcon. Generated files will be placed in:

- For Python: _install/\*/interfaces/lib/python3.8/site-packages/\*/interfaces/*_ and must only be imported:

  ```python
  from PACKAGE_interfaces.msg import MyInterface
  from PACKAGE_interfaces.srv import MyInterface
  from PACKAGE_interfaces.action import MyInterface
  # And so on...
  ```

- For C++, headers will be placed in: _install/\*/interfaces/include/\*/interfaces/*_ and they can be included from there like this:

  ```c++
  #include "PACKAGE_interfaces/msg/MyInterface.hpp"
  #include "PACKAGE_interfaces/srv/MyInterface.hpp"
  #include "PACKAGE_interfaces/action/MyInterface.hpp"
  // And so on...

  // Only for actions:
  #include <rclcpp_action/rclcpp_action.hpp>
  ```

Remember to source install scripts to see the new package and compile against it!

## Best Practices

- Interface files extensions must be like _.msg_, _.srv_, _.action_, with uppercase camel notation names.
- Of course, all packages that use your interfaces will depend on the new interfaces package.

## Interface format

Below you can find some notes about how to write interface definition files of various kinds. For more information about interface files and types see the [ROS 2 documentation](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html#about-ros-2-interfaces).

### Messages

- Comments are allowed with _#_.

- Specify fields one by one.

- Syntax is:

  ```
  type field_name
  ```

- Types must be from ROS 2 standard types, or from another interface you can include first.
  Inclusion syntax is: _package/message_.
  Don’t forget to add a dependency for the other package in both _package.xml_ and _CMakeLists.txt_!
  Any message you’ve already created in the same package may also be included.

- Arrays can be specified as types with syntax:

  ```
  type[]
  ```

- Constants can be specified in the code with the following syntax:

  ```
  constanttype CONSTANTNAME=constantvalue
  ```

  note that constant names HAVE to be uppercase. Constants will be available while coding as special values for message fields.

### Services

Since services are client-server communications that are made of two messages, a request and a response, all of the above applies to the two parts of a _.srv_ file, which must be separated like this:

```
REQUEST
---
RESPONSE
```

Constants can be specified as above in both the request and the response, and will be available in only one of the two depending on where they were specified.

### Actions

All of the above also applies to action interface files. Actions are provided by the ROS 2 middleware by means of two services, namely *Goal* and *Result*, and a *Feedback* topic. Message formats for all three can be defined in a single *.action* interface file like this:

```
GOAL
---
RESULT
---
FEEDBACK
```

Again, there can be constants appropriately specified in each of the three messages.
