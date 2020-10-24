# Debug ROS Nodes

One of the key goals of `vscode-ros` is to provide a streamlined debugging experience for ROS nodes.
To achieve this, this extension aims to help developers utilize the debugging capabilities provided by Visual Studio Code.
This document covers instructions of how to use such functionalities.

Read more about the design and related discussions on the debugging functionalities in our [design document][spec_debug_ros_nodes].

## Attach

`vscode-ros` enables a bootstrapped debugging experience for debugging a ROS (Python or C++) node by attaching to the process.

To get started, create a `ros`-type debug configuration with an `attach` request: (use <kbd>Ctrl</kbd>-<kbd>Space</kbd> to bring up the autocomplete dropdown)

![create attach debug configuration][create_attach_debug_configuration]

### Attaching to a Python node

![attach to a python node][attach_to_python]

### Attaching to a C++ node

![attach to a cpp node][attach_to_cpp]

## Launch

`vscode-ros` enables a streamlined debugging experience for debugging a ROS (Python or C++) node in a ROS launch file similar to a native debug flow.

To get started, create a `ros`-type debug configuration with a `launch` request:

![create launch debug configuration][create_launch_debug_configuration]

### Prerequisite

There needs to be a running instance of `rosmaster`.
The launch-debug flow provided by `vscode-ros` will not spawn a `rosmaster`.

![check roscore status][check_roscore_status]

### Launch and debug Python and C++ nodes

![launch and debug Python and C++ nodes][launch_and_debug_nodes]

### <a name="build_tasks"></a> Use tasks to automatically build before starting debug session

The first thing you need to do is to create build task for your package(s) with enabled debug symbols. 
In the example below you can see a `catkin_make` build task that passes additional `-DCMAKE_BUILD_TYPE=Debug`  argument that switches build to use `Debug` configuration, which is the most suitable configuration for debugging, because it has 0 optimization level and includes debug symbols. Another option is to use `-DCMAKE_BUILD_TYPE=RelWithDebInfo` that also enables debug symbols, but uses `Release` settings for everything else and has optimizations enabled. `RelWithDebInfo` might be a go to build configuration for Windows users in order to avoid slowness of the debug CRT on Windows OS. You can read more about [CMAKE_BUILD_TYPE here][stackoverflow-cmake_build_type]

**Note: you might need to remove the old `build` folder to force rebuild in new configuraiton.**

```json5
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "make_debug",
            "type": "catkin_make",
            "args": [
                "--directory",
                "${workspaceFolder}",
                "-DCMAKE_BUILD_TYPE=Debug", // This extra argument enables built with debug symbols
            ],
            "problemMatcher": [
                "$catkin-gcc"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
        },
    ]
}
```

The next step would be to configure `.vscode/launch.json` and customize `preLaunchTask` to use `make_debug` task we created above.

```json5
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS: Launch",
            "type": "ros",
            "request": "launch",
            "target": "${workspaceFolder}/launch/some.launch", // <<< Configure path to your launch file
            "preLaunchTask": "make_debug", // <<< This is the task that will run before debugging starts
        }
    ]
}

```




### Use tasks to automatically build and start rosmaster

**This is current BLOCKED BY VSCode Bug [70283][ms-vscode.background_bug]. This bug will prevent the second debugging session from starting if roscore background task is already running**

This section continues setup that was described [above](#build_tasks), so please complete that section and ensure you can build and debug with manually started roscore 

We are going to define a new task named `make_debug_and_core` that is going to start both `make_debug` and `roscore: roscore` tasks. `roscore: roscore` is a background task that will continue running even after debuging session is over

```json5
{
    "version": "2.0.0",
    "tasks": [
        /// ... `make_debug` task definition as before
        {
            "label": "make_debug_and_core",
            "dependsOn": [
                "make_debug",
                "roscore: roscore", // This task is provided by vscode-ros
            ]
        },
    ]
}
```

The next step would be to switch `preLaunchTask` to use `make_debug_and_core` task we created above.

```json5
{
    "version": "0.2.0",
    "configurations": [
        {
            // ... same as before
            "preLaunchTask": "make_debug_and_core",
        }
    ]
}

```

## Note

1. Debugging functionality provided by `vscode-ros` has dependencies on VS Code’s [C++][ms-vscode.cpptools] and [Python][ms-python.python] extensions, and those have dependencies on the version of VS Code. To ensure everything works as expected, please make sure to have everything up-to-date.
2. To debug a C++ executable, please make sure the binary is [built with debug symbols][ros_answers_debug_symbol] (e.g. `-DCMAKE_BUILD_TYPE=RelWithDebInfo`, read more about [CMAKE_BUILD_TYPE here][stackoverflow-cmake_build_type]).
3. To use VS Code's C++ extension with MSVC on Windows, please make sure the VS Code instance is launched from a Visual Studio command prompt.

<!-- link to files -->
[create_attach_debug_configuration]: ../media/documentation/debug-support/create-attach-debug-config.gif
[attach_to_cpp]: ../media/documentation/debug-support/attach-to-cpp.gif
[attach_to_python]: ../media/documentation/debug-support/attach-to-python.gif
[create_launch_debug_configuration]: ../media/documentation/debug-support/create-launch-debug-config.gif
[check_roscore_status]: ../media/documentation/debug-support/check-roscore-status.gif
[launch_and_debug_nodes]: ../media/documentation/debug-support/launch-and-debug-nodes.gif

[spec_debug_ros_nodes]: ./spec/debug-ros-nodes.md

<!-- external links -->
[ros_answers_debug_symbol]: https://answers.ros.org/question/200155/how-to-debug-executable-built-with-catkin_make-without-roslaunch/

[ms-python.python]: https://marketplace.visualstudio.com/items?itemName=ms-python.python
[ms-vscode.cpptools]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
[ms-vscode.background_bug]: https://github.com/microsoft/vscode/issues/70283
[stackoverflow-cmake_build_type]: https://stackoverflow.com/a/59314670/888545
