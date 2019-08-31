# Debug ROS Nodes

## Attach

`vscode-ros` enables a bootstrapped debugging experience for debugging a ROS (Python or C++) node by attaching to the process.

To get started, create a `ros`-type debug configuration with an `attach` request:

![create_debug_configuration][create_debug_configuration]

### Attaching to a Python node

![attach_to_python][attach_to_python]

### Attaching to a C++ node

![attach_to_cpp][attach_to_cpp]

## Note

1. Debugging functionality provided by `vscode-ros` has dependencies on VS Code’s [C++][ms-vscode.cpptools] and [Python][ms-python.python] extensions, and those have dependencies on the version of VS Code. To ensure everything works as expected, please make sure to have everything up-to-date.
2. To debug a C++ executable, please make sure the binary is [built with debug symbols][ros_answers_debug_symbol] (e.g. ` -DCMAKE_BUILD_TYPE=Debug`).
3. To use VS Code's C++ extension with MSVC on Windows, please make sure the VS Code instance is launched from a Visual Studio command prompt.

<!-- link to files -->
[create_debug_configuration]: ../media/documentation/debug-support/create-attach-debug-config.gif
[attach_to_cpp]: ../media/documentation/debug-support/attach-to-cpp.gif
[attach_to_python]: ../media/documentation/debug-support/attach-to-python.gif

<!-- external links -->
[ros_answers_debug_symbol]: https://answers.ros.org/question/200155/how-to-debug-executable-built-with-catkin_make-without-roslaunch/

[ms-python.python]: https://marketplace.visualstudio.com/items?itemName=ms-python.python
[ms-vscode.cpptools]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
