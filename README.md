# ROS 2 Examples

Collection of examples about the main ROS 2 features and core subsystems. These are meant to be used as tutorials or class materials. The format is simple: all code is thoroughly documented, and comments marked with an exclamation mark are meant to explain what is happening in detail.

Examples were built and tested on ROS 2 Galactic Geochelone. The repository offers also installation scripts and a shell file that wraps many useful ROS 2 commands in a simpler format (also solving some sourcing path problems, especially for Galactic). Other useful notes are available in the additional Markdown files. Where it is deemed necessary, links to official documentation pages are provided.

The repository is built around a Visual Studio Code template that you can find [here](https://github.com/robmasocco/vscode_ros2_workspace), which automates many tasks that concern ROS 2 workspace organization and maintenance. It's suggested that you use this inside such IDE.

**Currently this repository hosts only C++ examples, meant to be developed, built and executed on Linux machines only. Ubuntu 20.04 LTS Focal Fossa was used and is the currently supported and suggested platform.**

## List of examples

- **hello_ros2:** What `ros2 pkg create` generates by default.
- **topic_pubsub:** Examples about topics and messages:
  - `pub` and `sub` show how a node can subscribe or publish to a topic;
  - `periodic_sub` shows how to dynamically deactivate and reactivate topic subscriptions;
  - `intrapc_*` tests demonstrate ROS 2 intra-process communication capabilities, showing how nodes must be set up and used to enforce such behaviour.
- **simple_service:** Classic ROS 2 client/server example.
- **smp_example:** This example shows what must be done in order to write a multithreaded ROS 2 application.
- **ros2_examples_interfaces:** Example of an interfaces-only package (see `interfaces.md`).
- **custom_topic:** Similar to the above's `pub` and `sub`, the only difference is that they now use custom interfaces for messages defined in `ros2_examples_interfaces`.
- **ros2_examples_headers:** Example of an headers-only package, to provide to other packages in the same workspace, that will depend on this.
- **parameters_example:** This example offers an in-depth coverage of node parameters.
- **ros2_examples_bringup:** This package shows some conventions and best practices about launch files (see `launch_files.md`).
- **namespaces_examples:** This package contains some examples to show what remapping rules are and which features they offer.
- **actions_example:** This package shows how ROS 2 actions can be implemented, providing two example `client` and `server` nodes that compute the Fibonacci sequence. Its a rework of the official example provided [here](https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html).
- **custom_context:** Quick example that shows how an RCL context can be created, initialized, and finally shut down; not using the global context allows one to specify ad-hoc signal handlers and cleanup routines but, as is comprehensively explained in the `termination` example, this has to be done keeping in mind how the middleware works and needs to be managed.
- **polygon_base, polygons and polygons_tester:** These three packages form a variation of the official example about plugins and `pluginlib` (see the reference below), with the main differences being that:
  - optimal dynamic shared objects generation is enforced through the use of CMake target properties and GCC visibility macros;
  - the set of packages reflects what could happen in a common use-case scenario, where a ROS 2 plugin enables the developer to define different implementations of a same object or algorithm, or simply separate the implementation from the code that actually uses it; in particular:
    - **polygon_base** contains only headers that define a base class, so a common interface to all plugins that application code should use;
    - **polygons** uses those headers to create two shared libraries, which represent two different implementations of a same object, and registers them with `pluginlib`;
    - **polygons_tester** is an example of a package that uses plugins, including base class headers and loading specialized shared libraries through `pluginlib`.

## Additional resources

- **interfaces.md:** Describes best practices and conventions about interface files and packages.
- **launch_files.md:** Describes best practices and convetions about launch files and their packages.
- **ros2_debugging.md:** Gives some hints about how a ROS 2 application could be debugged.
- **ros2_cmds.sh:** Contains many shell commands to ease ROS 2 workspace sourcing and maintenance.
- **cli_cheats_sheet.pdf:** Quick summary of all CLI introspection tools commands.
- **vscode_cheat_sheet_linux.pdf:** Quick summary of useful Visual Studio Code keyboard commands on Linux.

## Useful references (links to Galactic docs)

- [**ROS/Patterns/Conventions**](http://wiki.ros.org/ROS/Patterns/Conventions)**:** Naming and measurement units conventions to respect when developing ROS 2 applications (the article is about ROS but still meaningful).
- [**`rqt_console`**](https://docs.ros.org/en/galactic/Tutorials/Rqt-Console/Using-Rqt-Console.html)**:** GUI to view, filter, save and reload log messages from multiple nodes at the same time.
- [**Creating and Using Plugins (C++)**](https://docs.ros.org/en/galactic/Tutorials/Pluginlib.html)**:** Basic tutorial about ROS 2 plugins.
- [**Efficient intra-process communication**](https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html)**:** Notes about how intra-process communication can be enforced in compliant situations (design document available [here](https://design.ros2.org/articles/intraprocess_communications.html)).
- [**Monitoring for parameter changes (C++)**](https://docs.ros.org/en/galactic/Tutorials/Monitoring-For-Parameter-Changes-CPP.html)**:** Tutorial about the `ParameterEventHandler` class, to monitor and respond to parameter changes taking place in all nodes.
- [**The `ROS_DOMAIN_ID`**](https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html)**:** What it is and what it is useful for.
- [**Recording and playing back data**](https://docs.ros.org/en/galactic/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)**:** How ROS 2 bags work and how they could be useful to log data. It is also possible to record them from an appropriate node, like [here](https://docs.ros.org/en/galactic/Tutorials/Ros2bag/Recording-A-Bag-From-Your-Own-Node-Cpp.html).
- [**About composition**](https://docs.ros.org/en/galactic/Concepts/About-Composition.html)**:** Introduction to the components distributed paradigm.
- [**Composing multiple nodes in a single process**](https://docs.ros.org/en/galactic/Tutorials/Composition.html)**:** Introduction to the components introspection tools and different loading methods, with links to some source code examples.
- [**Managed nodes**](https://design.ros2.org/articles/node_lifecycle.html)**:** Design document about nodes with lifecycle.
- [**Lifecycle**](https://github.com/ros2/demos/blob/galactic/lifecycle/README.rst)**:** Quick example that shows how nodes with lifecycle can be created and managed.
- [**`image_transport`**](http://wiki.ros.org/image_transport)**:** Useful package that optimizes middleware communications when handling images, which would easily cause streams of large packets to occur (linked docs are still about ROS, but the interface is almost the same in ROS 2).
- [**`image_pipeline`**](http://wiki.ros.org/image_pipeline)**:** Useful package to perform basic tasks on images (linked docs are still about ROS, but the interface is almost the same in ROS 2).
- [**`message_filters`**](http://wiki.ros.org/message_filters)**:** A message filter is defined as something which a message arrives into and may or may not be spit back out of at a later point in time; this package is a collection of algorithms fully integrated in the middleware that solve common synchronization issues involving multiple different topics (linked docs are still about ROS, but the interface is almost the same in ROS 2) (suggested read: the [`ApproximateTime`](http://wiki.ros.org/message_filters/ApproximateTime) adaptive algorithm).

## Docker Containers

The `docker_install.sh` script automates the installation of the Docker Engine, Docker Compose utility and Nvidia runtime on systems running Ubuntu 20.04.

**Results of such tests will be published in the `galactic` branch only, so example development containers will be provided only for the Galactic Geochelone ROS 2 distribution.**

The `config` directory contains a `Dockerfile` and some other configuration files necessary for the builds. Containers are built from generic development images found in the `docker` folder, and the provided targets are as follows:

1. `dev` contains some system utilities and full desktop ROS 2 installation;
2. `armv8` is like `dev`, but aimed at ARM devices;
3. `nvidia` adds correct support for Nvidia GPUs, in order to run GUI-based tools and GPU-enabled programs inside the container (this also requires the Nvidia runtime to be installed on the host system, on Ubuntu 20.04 this can be done by installing the `nvidia-docker2` package with `apt`).
4. `rpi` is a full development container aimed at Raspberry Pi boards.

You can build such containers manually from inside the `config` folder and manage them manually through the Docker Engine, they are fully functional and allow to test ROS 2 capabilities without installing anything on the host system. In both containers, the default shell is Zsh with some custom prompts and plugins preinstalled. The command line arguments necessary to run them correctly are specified in the Dockerfile header for convenience.

### Docker Compose and Visual Studio Code

It is incredibly easy to develop inside running containers with Visual Studio Code, and this repository is already configured for that by default. The `docker-compose.yml` file specifies how to build and run containers for the two development targets described above. Such targets can also be embedded in a VS Code-enabled container if one installs the required extensions to work with containers and choses to `Open Folder in Container`, then selects one of `container-dev`, `container-nvidia`, and `container-rpi`. Those folders each refer to the aforementioned targets, and contain only a `.devcontainer.json` file that instructs VS Code on how the container must be built on top of what the `docker-compose.yml` says:

- which extensions to install and load in the remote VS Code instance;
- which folder to open once the container is up and running;
- which shell to use in the remote instance's integrated terminal.

In `docker-compose.yml`, containers are configured as follows:

- build context is moved to the `config` directory;
- host network is used at both build and run-time;
- capabilities and security policies are relaxed to allow for some debugging inside the container;
- the terminal is supposed to handle colors;
- the non-root internal user is selected;
- an interactive shell is allocated for the container, allowing for signals to be passed, but `stdin` is closed;
- **current user's `.ssh` directory is mounted inside the container internal user's home directory**, to allow for remote services such as GitHub repositories to be used from inside the container too;
- **configuration files and command history files are mounted inside the container**, where they are expected to be, in order to allow for changes to be implemented quickly and for command history to be preserved;
- **the current host workspace folder is mounted inside the container**, to be reopened by VS Code;
- **for `nvidia` containers**, the GPU is exposed to processes running inside the container, and the `nvidia` runtime is used;
- **for `rpi` containers**, the hardware is fully exposed, to allow access to GPIO and similar devices.

In this way no change is lost since everything is written on the host file system, builds are preserved even if the image is not saved, and `git` integrations work from both inside and outside the container.

**Host network must be used at least at run-time to be able to communicate with ROS 2 nodes running inside the container from outside.**

To speed up the development process once the container is up and running, the use of VS Code's integrated terminal is highly suggested. To improve its usage, you may want to have a look at the following keyboard shorcuts:

- `workbench.action.terminal.newInActiveWorkspace`
- `workbench.action.terminal.kill`
- `workbench.action.terminal.splitInActiveWorkspace`

Then, you can navigate between multiple terminals using a combination of `Ctrl` and `Pg` keys, and multiple split subterminals with `Alt` and `Arrow` keys.

However, you can also start shells from outside the container using a combination of `docker ps` and `docker exec`. It should also be straightforward to set up aliases for this. Since container names are predefined in the `.devcontainer.json` files, the following should work as an example for the `ros2vsc-dev` target and can be used to set up an alias:

```bash
docker exec -it $(docker ps -q -f "name=dev") zsh
```

assuming that there are no name conflicts.

Once you close the remote connection or the VS Code window, the container should be stopped automatically as is specified in the `.devcontainer.json` files, which are adequately commented.
