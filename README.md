# ROS 2 Examples
Collection of examples about the main ROS 2 features and core subsystems. These are meant to be used as tutorials or class materials. The format is simple: all code is thoroughly documented, and comments marked with an exclamation mark are meant to explain what is happening in detail.

Examples were built and tested on ROS 2 Galactic Geochelone and Foxy Fitzroy. The repository offers also an installation script for Galactic and a shell file that wraps many useful ROS 2 shell commands in a simpler format (also solving some sourcing path problems, especially for Galactic). Other useful notes are available in the additional Markdown files. Where it is deemed necessary, links to official documentation pages are provided.

The repository is built around a Visual Studio Code template that you can find [here](https://github.com/robmasocco/vscode_ros2_workspace), which automates many tasks that concern ROS 2 workspace organization and maintenance. It's suggested that you use this inside such IDE.

**Currently this repository hosts only C++ examples, meant to be developed, built and executed on Linux machines only. Ubuntu 20.04 LTS Focal Fossa was used and is the currently supported and suggested platform.**

## List of examples

- **hello_ros2:** What *ros2 pkg create* generates by default.
- **topic_pubsub:** Example package with two executables: a publisher and a subscriber for a given topic.
- **simple_service:** Classic ROS 2 client/server example.
- **smp_example:** This example shows what must be done in order to write a multithreaded ROS 2 application.
- **ros2_example_interfaces:** Example of an interfaces-only package (see _interfaces.md_).
- **parameters_example:** This example offers an in-depth coverage of node parameters.
- **ros2_examples_bringup:** This package shows some conventions and best practices about launch files (see *launch_files.md*).
- **namespaces_examples:** This package contains some examples to show what remapping rules are and which features they offer.
- **actions_example:** This package shows how ROS 2 actions can be implemented, providing two example *client* and *server* nodes that compute the Fibonacci sequence. Its a rework of the official example provided [here](https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html).
- **custom_context:** Quick example code that shows how an RCL context can be customized, initialized, and finally shut down.

## Additional resources

- **interfaces.md:** Describes best practices and conventions about interface files and packages.
- **launch_files.md:** Describes best practices and convetions about launch files and their packages.
- **ros2_debugging.md:** Gives some hints about how a ROS 2 application could be debugged.
- **ros2_cmds.sh:** Contains many shell commands to ease ROS 2 workspace sourcing and maintenance.
