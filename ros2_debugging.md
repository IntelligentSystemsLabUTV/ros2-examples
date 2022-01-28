# Debugging ROS 2 C++ Applications with Visual Studio Code
ROS 2 C++ applications are common executables, with the only difference being that the Launch System is involved in their startup to bring the rest of the middleware up before them if required. Still, common debugging tools such as GDB or Valgrind can be used by adding appropriate prefixes to the `run` command; see its helper for details, a possible command line would be:
```bash
ros2 run --prefix "gdb ..." PACKAGE_NAME EXECUTABLE_NAME
```

## Debugging with Visual Studio Code
It's actually possible to debug ROS 2 nodes and applications in Visual Studio Code, although since the Launch System is involved a little even when `ros2 run` is invoked, some extra work is required.

### Requirements
This tutorial was done using:
- [ROS 2 Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/) (but Galactic Geochelone should work the same way)
- [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
- [Visual Studio Code 1.55.2](https://code.visualstudio.com/)
- [C/C++ VSCode extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)

### Instructions
Once you have your C++ code ready to be compiled, the first thing to do is to **compile the package exporting the symbols** (allow the breakpoints where you want to stop the code) by instructing CMake:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
```
Second, you have to **launch the [GDB](https://en.wikipedia.org/wiki/GNU_Debugger) Server when launching the application. Here, we will use a `localhost:port` address for creating the server. Choose any free port that you prefer:
```bash
ros2 run --prefix 'gdbserver localhost:3000' package_name executable_name
```
Third, you have to **create a launch.json for VS Code**. In other words, we will create a custom debugging configuration. In our case, create a GDB client and connect to the server:
1. open your workspace in VS Code;
2. go to your side bar, 'Run and Debug' section;
3. add a new configuration (select C++ enviroment or any other);
4. on your launch.json file, put the following information:
    ```json
    {
      "version": "0.2.0",
      "configurations": [
        {
          "name": "C++ Debugger",
          "request": "launch",
          "type": "cppdbg",
          "miDebuggerServerAddress": "localhost:3000",
          "cwd": "/",
          "program": "[build-path-executable]"
        }
      ]
    }
    ```
    - __name__ - Custom name of your debugger configuration.
    - __request__ - In this case we want to _launch_ the client.
    - __type__ - _cppdbg_ for C++ debugging.
    - __miDebuggerServerAddress__ - `server:port` address.
    - __cwd__ - Where to find all the required files; we use root because ROS 2, the package, and other required files are distributed along the entire PC.
    - __program__ - Replace `[build-path-executable]` with your executable file path; you can find this path on the console when you launch the server.

Then, you should be able to use VS Code's interface to debug your program.
