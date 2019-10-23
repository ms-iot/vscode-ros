# Debug ROS Nodes

One of the key goals of `vscode-ros` is to provide a streamlined debugging experience for ROS nodes.
To achieve this, this extension aims to help developers utilize the debugging capabilities provided by Visual Studio Code.
This document covers instructions of how to use such functionalities.

Read more about the design and related discussions on the debugging functionalities in our [design document][spec_debug_ros_nodes].

## Attach

`vscode-ros` enables a bootstrapped debugging experience for debugging a ROS (Python or C++) node by attaching to the process.

To get started, create a `ros`-type debug configuration with an `attach` request:

![create attach debug configuration][create_debug_configuration]

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

## Note

1. Debugging functionality provided by `vscode-ros` has dependencies on VS Code’s [C++][ms-vscode.cpptools] and [Python][ms-python.python] extensions, and those have dependencies on the version of VS Code. To ensure everything works as expected, please make sure to have everything up-to-date.
2. To debug a C++ executable, please make sure the binary is [built with debug symbols][ros_answers_debug_symbol] (e.g. `-DCMAKE_BUILD_TYPE=RelWithDebInfo`).
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
