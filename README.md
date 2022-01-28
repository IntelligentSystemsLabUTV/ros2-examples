# ROS 2 Examples

Collection of examples about the main ROS 2 features and core subsystems. These are meant to be used as tutorials or class materials. The format is simple: all code is thoroughly documented, and comments marked with an exclamation mark are meant to explain what is happening in detail.

Examples were built and tested on ROS 2 Galactic Geochelone, and are also supplied for Foxy Fitzroy. The repository offers also an installation script for Galactic and a shell file that wraps many useful ROS 2 shell commands in a simpler format (also solving some sourcing path problems, especially for Galactic). Other useful notes are available in the additional Markdown files. Where it is deemed necessary, links to official documentation pages are provided.

The repository is built around a Visual Studio Code template that you can find [here](https://github.com/robmasocco/vscode_ros2_workspace), which automates many tasks that concern ROS 2 workspace organization and maintenance. It's suggested that you use this inside such IDE.

**Currently this repository hosts only C++ examples, meant to be developed, built and executed on Linux machines only. Ubuntu 20.04 LTS Focal Fossa was used and is the currently supported and suggested platform.**

## List of examples

- **hello_ros2:** What `ros2 pkg create` generates by default.
- **topic_pubsub:** Examples about topics and messages:
  - `pub` and `sub` show how a node can subscribe or publish to a topic;
  - a variation of the above shows how to use custom interfaces and how to pass command-line options to CMake when building with colcon;
  - `periodic_sub` shows how to dynamically deactivate and reactivate topic subscriptions;
  - `intrapc_*` tests demonstrate ROS 2 intra-process communication capabilities, showing how nodes must be set up and used to enforce such behaviour.
- **simple_service:** Classic ROS 2 client/server example.
- **smp_example:** This example shows what must be done in order to write a multithreaded ROS 2 application.
- **ros2_example_interfaces:** Example of an interfaces-only package (see `interfaces.md`).
- **parameters_example:** This example offers an in-depth coverage of node parameters.
- **ros2_examples_bringup:** This package shows some conventions and best practices about launch files (see `launch_files.md`).
- **namespaces_examples:** This package contains some examples to show what remapping rules are and which features they offer.
- **actions_example:** This package shows how ROS 2 actions can be implemented, providing two example `client` and `server` nodes that compute the Fibonacci sequence. Its a rework of the official example provided [here](https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html).
- **custom_context:** Quick example that shows how an RCL context can be created, initialized, and finally shut down; not using the global context allows one to specify ad-hoc signal handlers and cleanup routines but, as is comprehensively explained in the `termination` example, this has to be done keeping in mind how the middleware works and needs to be managed.

## Additional resources

- **interfaces.md:** Describes best practices and conventions about interface files and packages.
- **launch_files.md:** Describes best practices and convetions about launch files and their packages.
- **ros2_debugging.md:** Gives some hints about how a ROS 2 application could be debugged.
- **ros2_cmds.sh:** Contains many shell commands to ease ROS 2 workspace sourcing and maintenance.
- **cli_cheats_sheet.pdf:** Quick summary of all CLI introspection tools commands.

## Useful references (links to Galactic docs)

- [**rqt_console**](https://docs.ros.org/en/galactic/Tutorials/Rqt-Console/Using-Rqt-Console.html)**:** GUI to view, filter, save and reload log messages from multiple nodes at the same time.
- [**Creating and Using Plugins (C++)**](https://docs.ros.org/en/galactic/Tutorials/Pluginlib.html)**:** Basic tutorial about ROS 2 plugins.
- [**Efficient intra-process communication**](https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html)**:** Notes about how intra-process communication can be enforced in compliant situations (design document available [here](https://design.ros2.org/articles/intraprocess_communications.html)).
- [**Monitoring for parameter changes (C++)**](https://docs.ros.org/en/galactic/Tutorials/Monitoring-For-Parameter-Changes-CPP.html)**:** Tutorial about the `ParameterEventHandler` class, to monitor and respond to parameter changes taking place in all nodes.
- [**The ROS_DOMAIN_ID**](https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html)**:** What it is and what it is useful for.
- [**Recording and playing back data**](https://docs.ros.org/en/galactic/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)**:** How ROS 2 bags work and how they could be useful to log data. It is also possible to record them from an appropriate node, like [here](https://docs.ros.org/en/galactic/Tutorials/Ros2bag/Recording-A-Bag-From-Your-Own-Node-Cpp.html).
- [**About composition**](https://docs.ros.org/en/galactic/Concepts/About-Composition.html)**:** Introduction to the components distributed paradigm.
- [**Composing multiple nodes in a single process**](https://docs.ros.org/en/galactic/Tutorials/Composition.html)**:** Introduction to the components introspection tools and different loading methods, with links to some source code examples.
- [**Managed nodes**](https://design.ros2.org/articles/node_lifecycle.html)**:** Design document about nodes with lifecycle.
- [**Lifecycle**](https://github.com/ros2/demos/blob/galactic/lifecycle/README.rst)**:** Quick example that shows how nodes with lifecycle can be created and managed.
