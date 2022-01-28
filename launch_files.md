# ROS 2 Launch Files

ROS launch files are Python scripts used to fully interact with the ROS Launch System. A launch file specifies which modules and nodes should be started and how, configuring their input arguments and many aspects of their processes. For an initial description of what launch files can do please see the [official documentation](https://docs.ros.org/en/galactic/Tutorials/Launch/Launch-Main.html).

What follows is a thorough description of launch files syntax, functionalities, and related conventions and best practices.

## Launch files packages

A common best practice is to centralize all launch files for a project in a single package, which we'll commonly refer to as a `bringup` package. It can be created in the following way inside a ROS 2 workspace:

- create a new package with `ament_cmake` build type, **its name should end with `_bringup` to state that this package consists only of launch files**;
- remove `include` and `src` directories;
- create a new `launch` directory;
- in `CMakeLists.txt` remove C/C++ standards directives, linters and testers directives, compiler directives, leaving only ament-related stuff, then add the following lines to install (symbolic links to) the whole launch files folder:

  ```cmake
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
  ```

- in the `package.xml` file, add lines like the following for each package of which you want to start nodes:

  ```xml
  <exec_depend>OTHER_PACKAGE</exec_depend>
  ```

  after the `<buildtool_depend>` line; this states a dependency only at execution time, so it's not necessary in the `CMakeLists.txt` file.

## Launch file structure

Launch files are essentially Python files, not scripts but modules which must provide some functions that the ROS 2 Launch System, written in Python, will call instead of its own to launch your executables and nodes. The names and structure of these functions must follow a convention that the Launch System expects, but the rest is up to you.

Each launch file must be placed inside the `launch/` directory of your package, and end with the `.launch.py` extension. The usage of the `--symlink-install` flag of `colcon build` is suggested since it removes the necessity to rebuild the package any more time if a launch file is modified.

The most basic and minimal structure of a launch file is as follows (except for explanatory comments):

```python
from launch import LaunchDescription # ROS object that tells how a module should be started
from launch_ros.actions import Node # ROS object that represents a Node to start

"""Generates a launch description for the given module"""
def generate_launch_description():
  ld = LaunchDescription()
  # Without the following the Launch System would just spawn a process
  # that would terminate immediately since no node has been specified
  node = Node(
    package='YOUR_PACKAGE',
    executable='YOUR_EXECUTABLE'
  )
  ld.add_action(node)
  # And this starts one node, just repeat the previous block of code
  # to start more nodes at once!
  return ld
```

But many more configurations can be done in such an environment, even accepting parameters from the command line:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

# The following are required to pass arguments from command line
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""Generates a launch description for the given module."""
def generate_launch_description():
  # Set command line arguments
  arg_1 = DeclareLaunchArgument(
    'ARG_NAME', # Argument name
    description='ARG_DESCRIPTION', # Argument description (for introspection tools)
    default_value='DEFAULT_VALUE', # Argument default value (as a string)
    choices=[ # Array of feasible values for this argument
      'VAL_1',
      'VAL_2' # , ...
    ]
  )
  ld.add_action(arg_1)
  # Repeat the above code block for as many CL arguments as you want!

  ld = LaunchDescription()
  node = Node(
    package='YOUR_PACKAGE',
    executable='YOUR_EXECUTABLE',
    name='NEW_NAME', # This will override the node's name specified in the code
    exec_name='NEW_PROCESS_NAME', # Marks the process in logs/tools instead of the basename
    namespace='NEW_NAMESPACE', # Specifies new ROS namespace for this executable
    shell=True, # If True, the process will be started in a new shell
    emulate_tty=True, # If False, logs are processed as text only (e.g. removing colors)
    output='both', # Configures logging in files and console, see below
    log_cmd=True, # Prints the final cmd before executing the process in the logs
    arguments=[ # Array of strings/parametric arguments that will end up in process's argv
      'FIXED_ARG', # , ...
      LaunchConfiguration('ARG_NAME') # , ...
    ],
    ros_arguments=[
      # List of strings that you would write after --ros-args when using ros2 run
    ],
    remappings=[ # Array of remapping rules specified as tuples of strings
      ('TOPIC_NAME', 'NEW_TOPIC_NAME'),
      ('SERVICE_NAME', 'NEW_SERVICE_NAME') # , ...
    ],
    parameters=[ # Array of dictionaries that specify node parameters
      {'PARAMETER': 'VALUE'} # , ...
    ] # Note that this could be replaced by a YAML file path
  )
  ld.add_action(node)
  # Again, previous section can be repeated to start more nodes at once!
  return ld
```

To build the YAML parameters file path, especially if this follows some kind of conventions like the `share/config` one, you could do something like this:

```python
import os
from ament_index_python.packages import get_package_share_directory
# ...

def generate_launch_description():
  # ...

  config = os.path.join(
    get_package_share_directory('PACKAGE_NAME'),
    'config',
    'FILE_NAME'
  )
  # Then place config in the parameters list argument

  # ...
```

See the `parameters_example/launch/launch_param_example.launch.py` launch file for more details.

For more information about what each argument to the object constructors does please see code documentation of the following ROS 2 Launch System modules:

- `launch_ros/actions/node.py`;
- `launch/actions/execute_process.py` (here there's actually a lot more stuff that allows one to customize the process environment, startup, termination by signals, debugging prefixes, logging and respawning).

## Starting modules with launch files

After having built and sourced a package with a launch file, the command to start it is `ros2 launch`. Have a look at its helper for its syntax and all its options. Eventual command line arguments specified in the launch file can be set with:

```bash
ros2 launch PACKAGE_NAME LAUNCH_FILE_NAME arg1:=value1 arg2:=value2 [...]
```

As soon as the Launch System loads and starts the executables specified in the launch file, you'll start to see some output in the console: by default, it will be the combined output of all the nodes started by the launch file. Also, if running in a shell or terminal, stdin will be combined too and if CTRL+C is pressed then SIGTERM is delivered to all processes simultaneously, making them all terminate.

**Even if you specify no particular configuration for a node, e.g. as in the first code example above, the Launch System is still going to add `--ros-args` to the process's command line, hence to its argv (not knowing this caused the writer quite many headaches when checking argc).**

Logging can be configured via the `output` argument of the `Node` object. When you use `ros2 run` to start an executable it doesn't redirect process output, so you're going to see stdout and stderr in your console, and no log files will be generated. If you use `ros2 launch` instead, output will be redirected: by default it will be reduced to stderr only in the console while everything goes in a log `.txt` file, usually written in a subdirectory of `~/.ros/`. Options for the `output` argument, taken from the Rolling documentation of `launch/logging/__init__.py`, are as follows:

- **`'screen'`:** stdout and stderr are logged to the screen;
- **`'log'`:** stdout and stderr are logged to launch log file and stderr to the screen;
- **`'both'`:** both stdout and stderr are logged to the screen and to launch main log file;
- **`'own_log'`:** for stdout, stderr and their combination to be logged to their own log files;
- **`'full'`:** to have stdout and stderr sent to the screen, to the main launch log file, and their own separate and combined log files.

## [Event handlers](https://docs.ros.org/en/galactic/Tutorials/Launch/Using-Event-Handlers.html)

Launch files let you do much more than starting up your application(s): they can be written as complete Python scripts to perform many different operations during startup, operation, and termination of the various actions they create. See the linked documentation for details.

## [Large projects](https://docs.ros.org/en/galactic/Tutorials/Launch/Using-ROS2-Launch-For-Large-Projects.html)

Starting single modules and entire architectures with launch files are not the same thing. In the latter case, it is better to reuse launch files that one has already written, to lay down an high-level launch structure and eventually modify the rest. See the linked documentation for details.
