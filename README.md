# VS Code ROS 2 Workspace Template

This template will get you set up using ROS 2 with VS Code as your IDE.

It's a reduced version of [athackst/vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace).

**CURRENTLY SUPPORTED ROS 2 VERSIONS:**

- ROS 2 Galactic Geochelone on branch `galactic`
- ROS 2 Foxy Fitzroy on branch `foxy`

**CURRENTLY SUPPORTED OS:** Ubuntu 20.04

## Features

### Style

ROS 2-approved formatters are included in the IDE:

* **C++** `uncrustify`, configured from `ament_uncrustify`;
* **Python** `autopep8`, with VS Code settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/).

### Tasks

There are many pre-defined tasks, see [`.vscode/tasks.json`](.vscode/tasks.json) for a complete listing.  Feel free to adjust them to suit your needs. They include and automate many common operations such as:

- Creation of packages inside `src/` with proper options, for both `ament_cmake` and `ament_python` build types.
- Building of whole workspaces or sets of packages, with or without debugging support.
- Workspace cleaning.
- Code formatting.

### Workspace organization

`.gitignore` ignores everything but the source code root directory by default. A second one is present in `src/`, ready to be populated if need be.
