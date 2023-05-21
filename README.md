# ros2-examples

Collection of examples about the main features and core subsystems of the [Robot Operating System 2](https://docs.ros.org/en/humble/index.html) robotics middleware, for tutorials or class materials.

## Table of contents

- [ros2-examples](#ros2-examples)
  - [Table of contents](#table-of-contents)
  - [Requirements](#requirements)
  - [Usage guidelines](#usage-guidelines)
    - [Docker containers](#docker-containers)
  - [Code organization](#code-organization)
    - [List of examples](#list-of-examples)
  - [Additional resources](#additional-resources)
    - [Useful references](#useful-references)
  - [Feedback](#feedback)
  - [License](#license)
  - [Copyright](#copyright)

## Requirements

- **Git** (to clone this repository) (find a comprehensive tutorial from zero to hero [here](https://www.atlassian.com/git/tutorials/what-is-git))
- **ROS 2 Humble Hawksbill**
- **GCC toolchain** (or any other C++ compiler that supports C++17)
- **Python 3**
- **CMake**
- **Visual Studio Code** (optional, other IDEs should work but the repository is built around VS Code)
- **Docker** (optional)

To clone this repository once you have Git installed, either use a GUI client or (better) run the following command in a terminal:

```bash
git clone https://github.com/IntelligentSystemsLabUTV/ros2-examples.git
```

and the repository will be cloned in the current directory, in a new directory called `ros2-examples`.

To update the repository's contents, make sure to be in the `humble` branch (rember: `git checkout` enables you to switch branches) and run:

```bash
git pull
```

The contents of this repository were tested on a system running **Ubuntu Linux 22.04**, which is the officially supported host OS. You may as well be able to install ROS 2 on Windows and macOS, and thus run the examples on those platforms, but this is not guaranteed. Support for Windows is encouraged by means of the [Windows Subsystem for Linux 2](https://docs.microsoft.com/en-us/windows/wsl/install-win10) (WSL 2), which allows you to run a Linux kernel on Windows hosts; the `Ubuntu-22.04` distribution would be the best choice.

The repository is built around a Visual Studio Code template that you can find [here](https://github.com/robmasocco/vscode_ros2_workspace), which automates many tasks that concern ROS 2 workspace organization and maintenance. It's suggested that you play with this repository inside VS Code.

## Usage guidelines

If this is the first time you hear about ROS 2, go have a look at the [official documentation](https://docs.ros.org/en/humble/index.html) and some basic [tutorials](https://docs.ros.org/en/humble/Tutorials.html) before proceeding. It is also recommended to have a look at the [installation guide](https://docs.ros.org/en/humble/Installation.html) to get a grasp of the basic concepts and terminology.

The file [`bin/ros2_humble_install.sh`](bin/ros2_humble_install.sh) contains a script that automates the installation of ROS 2 Humble Hawksbill on Ubuntu Linux 22.04. It is not guaranteed to work on other platforms, but it should be easy to adapt it to other Debian-based distributions.

The file [`config/ros2_cmds.sh`](config/ros2_cmds.sh) contains a list of useful CLI commands to source the ROS 2 installation and its autocompletion scripts, which you can copy and paste in your `.bashrc` file to have them available in your shell; alternatively, you can `source` that file.

At this point, you should be able to build, run and modify all of the examples in this repository. **It is strongly suggested that you create your own branch and work on that**, so that you can always pull the latest updates from the `humble` branch without losing your work.

The following sections will provide a brief overview of the repository's structure and contents.

### Docker containers

This repository is based on the [`Distributed Unified Architecture`](https://github.com/IntelligentSystemsLabUTV/dua-template), which provides support for Docker containers, so that you can run the examples without having to install ROS 2 on your host machine, and more. If you are interested, please find more information in the [`dua_template.md`](dua_template.md) file.

According to the DUA target architecture, this repository offers the following containers:

- `x86-dev`
- `x86-cudev`
- `armv8-dev`

which are intended to provide a development environment on all major platforms, with all the limitations that come with each. In case you want to run code from this repository on a different machine using DUA containers, you can use the DUA configuration scripts to add targets as explained in the [`dua_template.md`](dua_template.md) file.

## Code organization

The format is simple: all the code is thoroughly documented, and comments marked with an exclamation mark are meant to explain what is happening in detail.

In C++ source files, you may find:

```c++
// This is a comment.
//! This is a comment that explains what is happening in detail.
```

whereas in Python code:

```python
# This is a comment.
#! This is a comment that explains what is happening in detail.
```

Examples are provided for both C++ and Python, although the main focus of this project is on the former.

Expect this repository to be updated frequently, as it is meant to be a work in progress.

All source code is found in the `src` directory, which is organized as follows:

- `cpp` contains all the C++ code;
- `python` contains all the Python code;
- non-language-specific packages are found in the root `src` directory;
- the `cpp/advanced` and `python/advanced` directories contain examples that are more complex and are aimed at achieving a deeper understanding of ROS 2 by discussing some of its more advanced features.

### List of examples

The following is a list of all the examples that are currently available in this repository.

**Please note that since this is a work in progress, some packages listed here may not be available yet, being still under development. They will be automatically ignored by `colcon` to avoid issues. Similarly, this list may not include packages that are not ready yet.**

- **Configuration packages**
  - **ros2_examples_interfaces:** Example of an interfaces-only package (see [`interfaces.md`](interfaces.md)).
  - **ros2_examples_bringup:** This package shows some conventions and best practices about launch files (see [`launch_files.md`](launch_files.md)).

- **C++ examples**
  - **topic_pubsub_cpp:** Examples about topics and messages:
    - `pub` and `sub` show how a node can subscribe or publish to a topic;
    - `periodic_sub` shows how to dynamically deactivate and reactivate topic subscriptions.
  - **custom_topic_cpp:** Similar to the above's `pub` and `sub`, the only difference is that they now use custom interfaces for messages defined in `ros2_examples_interfaces` and a 'best effort' QoS policy.
  - **simple_service_cpp:** Classic ROS 2 client/server example.
  - **parameters_example_cpp:** This example offers an in-depth coverage of node parameters.
  - **actions_example_cpp:** This package shows how ROS 2 actions can be implemented, providing two example `client` and `server` nodes that compute the Fibonacci sequence.
  - **plugins_demo:** The three packages inside that directory form a variation of the official example about plugins and `pluginlib` (see the reference below), with the main differences being that:
    - optimal dynamic shared objects generation is enforced through the use of CMake target properties and GCC visibility macros;
    - the set of packages reflects what could happen in a common use-case scenario, where a ROS 2 plugin enables the developer to define different implementations of a same object or algorithm, or simply separate the implementation from the code that actually uses it; in particular:
      - **polygon_base** contains only headers that define a base class, so a common interface to all plugins that application code should use;
      - **polygons** uses those headers to create two shared libraries, which represent two different implementations of a same object, and registers them with `pluginlib`;
      - **polygons_tester** is an example of a package that uses plugins, including base class headers and loading specialized shared libraries through `pluginlib`.
  - **Advanced examples**
    - **namespaces_examples:** This package contains some examples to show what remapping rules are and which features they offer.
    - **ros2_examples_headers:** Example of an headers-only package, to provide to other packages in the same workspace, that will depend on this.
    - **smp_example:** This example shows what must be done in order to write a multithreaded ROS 2 application.

- **Python examples**

## Additional resources

Find more information about specific topics in the following files:

- [**`interfaces.md`**](interfaces.md)**:** Describes best practices and conventions about interface files and packages.
- [**`launch_files.md`**](launch_files.md)**:** Describes best practices and convetions about launch files and bringup packages.
- [**`ros2_cli_cheat_sheet.pdf`**](ros2_cli_cheat_sheet.pdf)**:** Quick summary of all CLI introspection tools commands available in ROS 2.
- [**`vscode_cheat_sheet_linux.pdf`**](vscode_cheat_sheet_linux.pdf)**:** Quick summary of useful Visual Studio Code keyboard commands on Linux.

### Useful references

Some useful references about ROS 2 features:

- [**ROS/Patterns/Conventions**](http://wiki.ros.org/ROS/Patterns/Conventions)**:** Naming and measurement units conventions to respect when developing ROS 2 applications (the article is about ROS but still meaningful).
- [**`rqt_console`**](https://docs.ros.org/en/humble/Tutorials/Rqt-Console/Using-Rqt-Console.html)**:** GUI to view, filter, save and reload log messages from multiple nodes at the same time.
- [**Creating and Using Plugins (C++)**](https://docs.ros.org/en/humble/Tutorials/Pluginlib.html)**:** Basic tutorial about ROS 2 plugins: shared libraries to implement a common interface for algorithms, subsystems, and more, to be dynamically loaded at runtime.
- [**Efficient intra-process communication**](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html)**:** Notes about how intra-process communication can be enforced in compliant situations (design document available [here](https://design.ros2.org/articles/intraprocess_communications.html)). Note that this works at the middleware level: ROS 2 messages are passed between entities instead of being handed over to the DDS, but the latter would still bypass the network stack when implementing shared memory transport by default (*e.g.*, eProsima's FastDDS, which is the default RMW implementation in Humble).
- [**Monitoring for parameter changes (C++)**](https://docs.ros.org/en/humble/Tutorials/Monitoring-For-Parameter-Changes-CPP.html)**:** Tutorial about the `ParameterEventHandler` class, to monitor and respond to parameter changes taking place in all nodes.
- [**The `ROS_DOMAIN_ID` environment variable**](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)**:** What it is and what it is useful for.
- [**Recording and playing back data**](https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)**:** How ROS 2 bags work and how they could be useful to log data. It is also possible to record them from an appropriate node, like [here](https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-A-Bag-From-Your-Own-Node-Cpp.html).
- [**About composition**](https://docs.ros.org/en/humble/Concepts/About-Composition.html)**:** Introduction to the components distributed paradigm.
- [**Composing multiple nodes in a single process**](https://docs.ros.org/en/humble/Tutorials/Composition.html)**:** Introduction to the components introspection tools and different loading methods, with links to some source code examples.
- [**Managed nodes**](https://design.ros2.org/articles/node_lifecycle.html)**:** Design document about nodes with lifecycle.
- [**Lifecycle**](https://github.com/ros2/demos/blob/humble/lifecycle/README.rst)**:** Quick example that shows how nodes with lifecycle can be created and managed.
- [**`image_transport`**](http://wiki.ros.org/image_transport)**:** Useful package that optimizes middleware communications when handling images, which would easily cause streams of large packets to occur (linked docs are still about ROS, but the interface is almost the same in ROS 2).
- [**`image_pipeline`**](http://wiki.ros.org/image_pipeline)**:** Useful package to perform basic tasks on images (linked docs are still about ROS, but the interface is almost the same in ROS 2).
- [**`message_filters`**](http://wiki.ros.org/message_filters)**:** A message filter is defined as something which a message arrives into and may or may not be spit back out of at a later point in time; this package is a collection of algorithms fully integrated in the middleware that solve common synchronization issues involving multiple different topics (linked docs are still about ROS, but the interface is almost the same in ROS 2) (suggested read: the [`ApproximateTime`](http://wiki.ros.org/message_filters/ApproximateTime) adaptive algorithm).

## Feedback

If you have any questions or suggestions, please open an issue or contact us here on GitHub.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
