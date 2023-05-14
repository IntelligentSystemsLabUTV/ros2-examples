# dua-template

Project template for repositories based on the Distributed Unified Architecture.

## Abstract

This repository contains a set of files that make up a structural template for robotics development projects. Once appropriately forked on GitHub, it can be configured as the starting point for a new project as hereby described.

The main goal of this project is to offer a common, shared work environment that minimizes the time and effort required to set up prototyping and experimenting of new solutions, as well as to provide a common framework for teams that can be versatile enough to support any development workflow. It aims at achieving so by providing three main features: workspace organization for both development and deployment, hardware and operating system abstraction via Docker and middlewares like Data Distribution Services (DDS) and Robot Operating System 2 (ROS 2), modular structure for easy integration of multiple components into a single *macro-project* or control architecture, hence the name *Distributed Unified Architecture*.

This project creates a common development environment making use of three tools, which are also the main dependencies:

- Docker
- Git
- Visual Studio Code

**Please read the rest of this document carefully to assess whether DUA is the right tool for your project, and to get all the information required to set up a new project based on it.**

### Why Docker?

Docker is a tool that allows to create and run containers, which are isolated environments that can be used to run software in a reproducible way, with no additional overhead. It is a very powerful tool that can be used to create and store reproducible environments for any software, and that can be used to run software in a way that is independent of the operating system and hardware it is running on. This is particularly useful for robotics, where the same software can be run on different machines, with different operating systems and hardware, and still work as expected, implying that one can develop code inside an environment that imitates or is even an extension of the one that will run on the target machine, including development tools.

DUA ultimately allows a developer to build Docker images that include all the software required to run a project, starting from predefined environments that include common tools for robotics. Such environments are Docker images, denoted in the following as *base units*, that include:

- The operating system: **Ubuntu 20.04/22.04 LTS**.
- The Robot Operating System 2 robotics middleware: **ROS 2 Humble Hawksbill**.
- **C/C++ toolchains and debuggers**.
- A **Python 3** installation with basic packages.
- The **[`OpenCV`](opencv.org) computer vision library**.
- Additional development tools and libraries, depending on the platform at which the base unit is targeted.

The quantity of development tools installed in every base unit depends on the target platform, and is meant to be as minimal as possible. The idea is that, once a base unit is built, it can be used as a starting point for other Docker images that include only the software required to run a specific project. This allows to create a Docker image that is as small as possible, and that can be easily shared and deployed on different machines.

#### On the use of Docker in robotics

Docker has been originally developed to provide *microservices*, *i.e.*, to package software in small, replicable environments that could be completely isolated from the rest of the system while experiencing little to no overhead. **This is evidently not the case for robotics: while you can still have single software modules packaged individually, such modules are often meant to access the hardware and communicate with each other, possibly accessing the network stack directly, which is not possible in a completely isolated environment.**

So, to benefit from Docker in robotics and to fully support the tools listed in [Why Docker?](#why-docker) that come preinstalled in base units, some compromises must be made about Docker security features. In particular, as the template [`docker-compose.yaml`](bin/dua-templates/docker-compose.yaml.template) file testifies, by default

- Containers run in *privileged mode*, which means that they can access all the capabilities of the host machine; this is particularly necessary for software modules that require access to the hardware, like cameras, sensors, and communication interfaces, and for debuggers to work properly.
- Containers are allowed to access the host network stack, which means that they can access the network and communicate with other containers and the host machine; this is necessary for ROS 2 to work properly, and for DDS middlewares to work at all.
- Containers are granted full access to the hardware, since `/dev` and `/sys` are mounted inside the container; this is necessary for software modules that require access to the hardware.
- The internal user of the container is a member of the `sudo` group inside the container; however, it has a password, that must be specified when configuring the project as explained in [Creating a target](#creating-a-target) and that is only copied as `sha-512` salted hash in the `Dockerfile`, so its plain text is never stored anywhere.

**Please note that it is the end user's responsibility to ensure that the Docker configuration files are not modified in a way that would compromise the security of the system, and that the internal user password is not lost.** If the password is lost, the end user can always change it by modifying the `Dockerfile` and rebuilding the image.

If the end user requires a different configuration, they have full control over the Docker configuration files, and can modify them as they see fit.

### Why Git?

Git is a version control system that allows to keep track of changes to a set of files, and to share them with others. It is particularly useful for robotics, where it can be used to keep track of changes to source code, configuration files, and other files that are used in the development of a project. It is also useful to share code with other developers, and to keep track of the history of a project. Moreover, as explained in [Integrating other DUA-based projects](#integrating-other-dua-based-projects), Git features allow to easily integrate other projects into a single *macro-project*, to both resolve dependencies sharing code and to continue development in parallel, provided that the structure of both projects is compatible, which is exactly what this template is meant to provide.

### Why Visual Studio Code?

Visual Studio Code is a free, open-source, cross-platform IDE that can be used to develop software in many different languages. It is particularly useful for robotics, where it can be used to develop code in C/C++, Python, and other languages, and where it can be used to manage Git repositories and Docker containers. It is also very customizable, and can be extended with plugins that add support for many different languages and tools. As the contents of this repository and the following sections show, this template is meant to be used with Visual Studio Code, and it is configured to work with it out of the box, providing many useful features for robotics software development.

## Support and limitations

This work is developed on, and supposed to run on, Linux-based systems. It can work, and has been independently tested, on different machines running Windows or macOS, although not all features might be available due to limitations of the Docker engine on those platforms.

The officially supported platforms are:

- [x] Linux
- [x] Windows 10
- [x] Windows 11
- [x] macOS Ventura, on both Intel and Apple Silicon

Suggestions and improvements are welcome, and can be submitted as issues or pull requests.

### General dependencies

The following tools are required to set up and run a DUA project on all supported platforms:

- Git
- Docker
- Visual Studio Code
- One of the following two tools to generate hashed passwords:
  - `mkpasswd`, usually found in the `whois` package on Ubuntu.
  - The `passlib` Python package, which can be installed with `pip install passlib`.
- A working installation of [`GNU sed`](https://www.gnu.org/software/sed/), which is usually the default on Linux systems, but not on macOS where it can be installed with `brew install gnu-sed`.

### Linux support

Linux is the officially supported platform, and the one on which this template is developed. It is also the platform on which the Docker Engine is natively supported, and on which it is possible to run Docker containers with full hardware access and network stack access. This means that all features are available on Linux, and that the template is fully supported on Linux.

### Windows support

Windows is supported by the three main dependencies of this project, namely Git, Docker, and Visual Studio Code. However, to run DUA containers properly, the following dependencies are also required:

- [Windows Subsystem for Linux 2](https://docs.microsoft.com/en-us/windows/wsl/install), or WSL 2, which is a compatibility layer for running Linux binary executables natively on Windows 10 and Windows 11.
- [Docker Desktop](https://www.docker.com/products/docker-desktop), which is a tool that allows to run Docker containers on Windows and manage images.

To run a DUA container, follow the steps below:

1. Open Docker Desktop.
2. Open Visual Studio Code and connect the window to WSL 2.
3. Open a DUA project as explained below.

The following limitations apply:

- **Hardware access**: Docker containers running on Windows cannot access the hardware directly. This means that, while it is possible to run ROS 2 nodes inside a container, they will not be able to access the hardware connected to the host. This is a limitation of the Docker Engine on Windows, and it is not possible to overcome it.
- **Network stack access**: Docker containers running on Windows cannot access the network stack directly, relying instead on a virtualized one. This means that, while it is possible to run ROS 2 nodes inside a container, they cannot communicate with other nodes running on machines connected to the same network as the host. This is a limitation of the Docker Engine on Windows, and it is not possible to overcome it.

### macOS support

macOS is supported by the three main dependencies of this project, namely Git, Docker, and Visual Studio Code. However, to run DUA containers properly, the following dependencies are also required:

- [Docker Desktop](https://www.docker.com/products/docker-desktop), which is a tool that allows to run Docker containers on macOS and manage images.
- [XQuartz](https://www.xquartz.org/), which is an open-source X11 server for macOS, necessary to run GUI-based applications inside Docker containers; it can be installed with `brew install --cask xquartz`, and after the installation is complete you should also run `defaults write org.xquartz.X11 enable_iglx -bool true` in the terminal to ensure that some OpenGL applications can run.

The only supported targets on macOS are currently:

- [x] `x86-dev` for Intel Macs.
- [x] `arm64-dev` for Apple Silicon Macs.

To run a DUA container, follow the steps below:

1. Open Docker Desktop.
2. Open XQuartz and run `xhost + 127.0.0.1` in the terminal.
3. Open the project in Visual Studio Code as explained below.

The following limitations apply on both Intel and Apple Silicon Macs:

- **Hardware access**: Docker containers running on macOS cannot access the hardware directly. This means that, while it is possible to run ROS 2 nodes inside a container, they will not be able to access the hardware connected to the host. This is a limitation of the Docker Engine on macOS, and it is not possible to overcome it.
- **Network stack access**: Docker containers running on macOS cannot access the network stack directly, relying instead on a virtualized one. This means that, while it is possible to run ROS 2 nodes inside a container, they cannot communicate with other nodes running on machines connected to the same network as the host. This is a limitation of the Docker Engine on macOS, and it is not possible to overcome it.

The following limitations apply to Apple Silicon Macs only:

- **3D hardware acceleration**: The Docker Engine cannot access the graphics card properly, meaning that either the drivers available in the container are not appropriate or the performance is extremely poor. This implies that tools like Gazebo and RViz will not work. This is a limitation of the Docker Engine on Apple Silicon Macs, and it is not possible to overcome it.

## Sibling repositories

This template is one of three parallel projects that, together, form the DUA framework. The other two are:

- [`dua-foundation`](https://github.com/IntelligentSystemsLabUTV/dua-foundation): `Dockerfiles` and other configuration files that are required to build the Docker images used as base by DUA projects, known as *base units*.
- [`dua-utils`](https://github.com/IntelligentSystemsLabUTV/dua-utils): A collection of libraries and other software packages that can be used in robotics projects, and that are well integrated in DUA-based projects.

Finally, the directory tree is partially organized as a ROS 2 workspace that can be easily managed with Visual Studio Code, and the basic layout for this is defined in [`robmasocco/vscode_ros2_workspace`](https://github.com/robmasocco/vscode_ros2_workspace).

## Template structure

The template is organized as follows, as can be inferred following the directory tree starting from this file.

### `.vscode`

Contains a bunch of configuration files for Visual Studio Code, which is the recommended IDE for DUA projects. They can be modified and new ones can be added. They are meant to configure IDE behavior once it is opened inside a DUA container, and are not meant to store user-specific configurations but rather to be as generic as possible.

[`c_cpp_properties.json`](.vscode/c_cpp_properties.json) can be modified to add include paths for this or other projects, which position in the filesystem is known. It is also configured to store the IntelliSense database in this directory rather than holding it entirely in RAM.

[`tasks.json`](.vscode/tasks.json) includes the definition of many useful commands that automate the management of a ROS 2 workspace, like the creation of packages of different types.

### `bin`

The `bin` directory is meant to contain executable files such as configuration scripts, third-party portable tools, and so on. By default, it contains the two project configuration scripts [`dua_setup.sh`](bin/dua_setup.sh) and [`dua_subtrees.sh`](bin/dua_subtrees.sh), described below.

### `config`

This directory should contain project-wide configuration files of any sort.

### `docker`

This directory, once a project has been configured using setup scripts, will contain a subdirectory for each Docker image that is used in the project. Each of these subdirectories will contain a `Dockerfile` and a `docker-compose.yml` file, which are used to build and run the image, respectively, as well as many configuration files that are used during container creation and functioning.

### `logs`

This directory is meant to be the place where to dump all experimental data gathered during test runs, simulations, field tests, and so on. Everything in it is ignored by Git so as to avoid unnecessarily pushing hefty log files on remotes.

### `src`

As a ROS 2 workspace, this directory should host all source code, appropriately organized in packages and subdirectories as in any other ROS 2/`colcon` workspace.

**Other smaller projects based on DUA should be placed here to integrate them in a full control architecture.**

### `tools`

This directory is meant to store third-party tools, and their source code, that are not based on DUA, are not portable, or simply cannot follow the modular structure of DUA projects. This directory is ignored by `colcon` so the ROS 2 build system will not interfere with its contents.

## Configuring a new project

The first maintenance tasks to perform when creating a new DUA-based project must be carried on on GitHub: check that the repository name and settings are correct, rename this file as something like `dua-help.txt` to add your own `README.md`, and so on.

Then, the repository contents can be configured. For this, the [`dua_setup.sh`](bin/dua_setup.sh) script is provided. It can be invoked like this

```bash
dua_setup.sh create [-a UNIT1,UNIT2,...] NAME TARGET
dua_setup.sh modify [-a UNIT1,UNIT2,...] [-r UNIT1,UNIT2,...] TARGET
dua_setup.sh delete TARGET
dua_setup.sh clear TARGET
```

and an explanation of all relevant commands follows in the next sections. Before going further, it is important to note the meaning of some terms used to describe the parts of a DUA-based project:

- **Project**: indicates a single Git repository, independently of its role or purpose.
- **Unit**: indicates a software package, library, or collection thereof, that is developed as a DUA-based project; this name implies that while it is self-sufficient, can be executed inside its own container in parallel to other ones and share data with them, it can also be integrated as a sub-project into a larger one to form a control architecture.
- **Target**: indicates a single Docker image that is used in a project, and that is built from a single `Dockerfile` and `docker-compose.yaml` pair plus many other configuration files; a target can be used to run a single unit, or multiple ones, or even a whole control architecture; the name resembles the meaning of the term *target* in the context of a `Makefile`, and is used to refer to the same concept in this context: each unit or architecture can have multiple targets, *i.e.*, Docker images, each one supporting a different hardware platform, System on Chip (SoC), and so on. The software that makes up a project is ensured to run appropriately and similarly in all targets, since their images are always based on *base units* provided in [`dua-foundation`](https://github.com/IntelligentSystemsLabUTV/dua-foundation).

**[`dua_setup.sh`](bin/dua_setup.sh) can be executed both inside and outside a DUA container, but must always be executed from a project's root directory, *i.e.*, like this**

```bash
./bin/dua_setup.sh ...
```

### Creating a target

The command

```bash
dua_setup.sh create [-a UNIT1,UNIT2,...] NAME TARGET
```

creates a new target, *i.e.*, a Docker image, for a project. The `-a` option can be used to specify a list of units that will be integrated in the new target, and the `NAME` argument is the name of the project.

The `TARGET` argument is the name of the target. `TARGET` must be a valid target name, *i.e.*, one of the image tags in [`dua-foundation`](https://github.com/IntelligentSystemsLabUTV/dua-foundation).

**A password is required for all projects since the container's internal user runs with elevated privileges and has full access to the host's network stack and hardware devices.** It will be asked for during the creation of the target, and will be stored in hashed form in the target's `Dockerfile`.

`create` will create a new `container-TARGET` directory inside `docker/`, and will copy and configure all the template files stored in `bin/dua-templates`. These include `Dockerfile` and `docker-compose.yaml` files, as well as many shell configuration scripts and other configuration files that are used to build and run the image. Once copied, they can be modified to fully support the specific features of the project at hand. In particular:

- [`aliases.sh`](bin/dua-templates/context/aliases.sh) is meant to contain shell aliases.
- [`commands.sh`](bin/dua-templates/context/commands.sh) is meant to contain shell functions that execute commands that involve the software package that make up the project, or control architecture, at hand.
- [`p10k.sh`](bin/dua-templates/context/p10k.sh) contains configuration options for the [`powerlevel10k`](https://github.com/romkatv/powerlevel10k) that powers the Zsh shell, which is the default in every DUA container.
- [`ros2.sh`](bin/dua-templates/context/ros2.sh) contains configuration options and functions for the ROS 2 installation that is provided in every DUA image.

Once a target has been created, it can be managed using Docker and

- Visual Studio Code, in the case of development targets: install the `Remote - Containers` extension and open the `container-TARGET` folder "in container" using the integrated commands, and the IDE will be set up automatically after the container is built.
- Compose, in the case of deployment containers to be run on SoCs: just modify the `container-TARGET/devcontainer/docker-compose.yaml` file to add the necessary configuration options, and then run `docker-compose up` from the `container-TARGET` directory.

Containers are meant to be run in detached mode, *i.e.*, with the `-d` option, and can be accessed using the `docker exec` command. For example,

```bash
docker-compose exec NAME-TARGET zsh
```

opens a shell in the container. They can be normally stopped, which is also what happens when the VS Code window is closed or disconnected.

#### On units integration

As per the contents of the generic [`Dockerfile.template`](bin/dua-templates/Dockerfile.template), the `Dockerfile` of a target can be modified to include additional build steps specific to the project, *i.e.*, unit at hand. Suppose that the new unit is called, *e.g.*, `test`. Given one of the possible targets for `test`, **the section of the target's `Dockerfile` in which the additional build steps must be added is marked as**

```dockerfile
# IMAGE SETUP START #
# IMAGE SETUP END #
```

and may already contain additional build steps related to the units that are already integrated in the target, depending on how the setup was carried out and on the dependencies of the project. Then, **to ensure that the `dua_setup.sh` script can properly include the new unit into targets of other projects, wrap the build steps specific to `test` in the following lines**

```dockerfile
# test START #
# test END #
```

The rest of the `Dockerfile` adds an interal user to the container, configures mount points for configuration files so that they can be modified in real time from both inside and outside a container, and so on. The `docker-compose.yaml` file is also meant to be modified to add additional configuration options, such as the ones that specify the hardware devices that must be made available to the container. By default, the whole hardware is mounted via `/dev` and `/sys`. **The network mode must be set to `host` to ensure that units running inside a container can communicate with the outside world.**

To ensure that DUA units are properly integrated in a target when the [`dua_setup.sh`](bin/dua_setup.sh) script is invoked with the `-a` option, **they must be placed as subdirectories of the `src/` directory**. When an addition is carried out, contents of a unit's `Dockerfile` enclosed in the above lines is copied into the same section of the one of the target that is being configured. This allows one to add build steps specific to the project at hand, and to ensure that the unit is properly integrated in the target, achieving full modularity while automating the setup and integration process of a control architecture.

Please note that these rules imply that **build steps specific to dependencies of a unit are not included when such unit is integrated in another target**. Suppose that units `dep1` and `dep2` are dependecies of `test`, and that you have set up a target using `dua_setup.sh` with the `-a` option to include them before writing the section relevant to `test`. After your additions, the `IMAGE SETUP` section of the corresponding `Dockerfile` will look, *e.g.*, like this:

```dockerfile
# IMAGE SETUP START #
# dep1 START #
RUN echo "dep1"
# dep1 END #
# dep2 START #
RUN echo "dep2"
# dep2 END #
# test START #
RUN echo "test"
# test END #
# IMAGE SETUP END #
```

Now suppose that you integrate the `test` unit in a target of another unit, named, *e.g.*, `test2`, as a dependency. **If you specify only `test` as a unit to integrate when running `dua_setup.sh` with the `-a` option, the `IMAGE SETUP` section of the `Dockerfile` of `test2` will look like this (after `test2`-specific additions):**

```dockerfile
# IMAGE SETUP START #
# test START #
RUN echo "test"
# test END #
# test2 START #
RUN echo "test2"
# test2 END #
# IMAGE SETUP END #
```

**that is, the dependencies `dep1` and `dep2` of `test` will NOT be included.** This is because the `dua_setup.sh` script does not know which dependencies are already integrated in the target, and which are not, nor does it care, to ensure that the end user always has full control over the contents of the `Dockerfile` of a target and can always include the most up-to-date version of every unit, avoiding the risk of carrying over old code. **If you want to include the dependencies of `test` in the `Dockerfile` of `test2`, you must specify them explicitly when running `dua_setup.sh` with the `-a` option, in the correct order.**

### Modifying a target

The call

```bash
dua_setup.sh modify [-a UNIT1,UNIT2,...] [-r UNIT1,UNIT2,...] TARGET
```

allows one to modify an existing target, by adding new units or removing existing ones. **Removals are always performed before additions**.

Additions work as illustrated in [On units integration](#on-units-integration). If a single unit is specified for addition, then it will simply be appended to the existing ones in the target's `Dockerfile`. Instead, if multiple units are specified, the `Dockerfile` portion will be completely rewritten, and **will include only the units that were specified, in the order in which they where specified**. This allows one to specify an order in which the units are integrated in the target, which is useful when one needs to ensure that a specific unit is built before another one. Moreover, if one of the units specified in the command is already present in the target's `Dockerfile`, its section will be copied from this target's `Dockerfile` and not from the original one of the unit. This allows one to modify the integration of a unit in a target without having to modify the unit itself, adding build steps concerning it that are specific to the project at hand, being sure that they will be preserved when the target is updated.

### Clearing a target

The command

```bash
dua_setup.sh clear TARGET
```

simply removes all units from a target, leaving the `IMAGE SETUP` section of the Dockerfile empty.

### Deleting a target

The command

```bash
dua_setup.sh delete TARGET
```

removes a target, *i.e.*, the `container-TARGET` directory and all its contents.

## Integrating other DUA-based projects

The previous sections explained how a project can be configured and managed, hinting at how other DUA-based projects can be integrated as sub-portions, *i.e.*, units, of a greater one, from the point of view of the Docker engine. However, this is not enough to ensure that the units are properly integrated in the project at hand, and that they can be used as intended.

Given an additional unit to be integrated in a project, either as a full DUA unit in `src/` or an external tool in `tools/`, the project repository and git history must both contain a full copy of the unit, a task that can be carried on with Git itsef. This is to ensure simple management of dependencies, as well as to be able to fix specific versions of all dependent units.

The Git feature that makes this possible in DUA is that of **subtrees**. An excellent tutorial on the subject can be found [here](https://www.atlassian.com/git/tutorials/git-subtree). Essentially, integrating another Git repository as a subtree ensures

- the possibility of specifing the commit, branch, or tag of the repository to be integrated;
- a simple management, since files and Git histories will be merged in those of the parent (project) repository, with little to no additional commands required to keep them in sync;
- the possibility of carrying on development on the subtree, pushing and pulling changes to and from it, or just modifying it locally within the scope of the current project.

Even if subtrees require very few commands to be managed, there is still room for mistakes. To minimize their possibility, the [`dua_subtrees.sh`](bin/dua_subtrees.sh) script has been provided. It can be executed from wherever, from both inside and outside containers, and it will provide a simple interface to manage subtrees. It wraps subtree commands by specifying default options. Its helper and source code constitute its best documentation, so go check it out. **It is highly recommended to include DUA units as subtrees using this script.**

### On Git submodules incompatibility

**DUA is not compatible with Git submodules**, by a precise design choice.

This is because submodules are meant to be used to integrate external projects, *i.e.*, projects that are not part of the same repository. This is not the case in DUA, where the units are meant to be part of the same repository, to make management as easy and painless as possible. Moreover, submodules are exceptionally hard to manage among large teams, and even a simple usage generally results in Git history corruption or worse.

In particular, if a project that has submodules is included as a subtree, **submodules will not be cloned since they are managed externally from the Git source tree**. This means that the subtree's `.gitmodules` file must be added to the current project, ita paths and other settings appropriately modified, and only then submodule commands will work. This is a very cumbersome process, and it is not worth the effort, since it makes the easy modularity that DUA strives to achieve much more difficult to manage.

As such, the DUA framework will never support submodules. If a third-party or older project that uses submodules is to be converted into a DUA project, it must either be restructured to use subtrees or simply not use submodules, or **it will never be possible to integrate it as a unit into other DUA projects without tinkering with `.gitmodules` files**, thus spreading submodules to projects that have nothing to do with them. **It is, however, possible to use submodules in a DUA project, but only if it is not meant to be integrated in other projects as a unit.**

## Feedback

If you have any questions or suggestions, please open an issue or contact us here on GitHub.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
