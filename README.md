# Visual Studio Code Extension for ROS

The [Visual Studio Code][vscode] Extension for ROS provides support for [Robot Operating System (ROS)][ros] development for ROS1 and ROS2 on Windows and Linux. 


## Features

* Automatic ROS environment configuration.
* Allows starting, stopping and viewing the ROS core status.
* Automatically create `catkin_make` or `catkin build` build tasks.
* Create catkin packages using `catkin_create_pkg` script or `catkin create pkg`.
* Run `rosrun` or `roslaunch`
* Resolve dependencies with `rosdep` shortcut
* Syntax highlighting for `.msg`, `.urdf` and other ROS files.
* Automatically add the ROS C++ include and Python import paths.
* Format C++ using the ROS `clang-format` style.
* Preview URDF and Xacro files.
* Debug a single ROS node (C++ or Python) by [attaching to the process][debug_support-attach].
* Debug ROS nodes (C++ or Python) [launched from a `.launch` file][debug_support-launch].

## Commands

You can access the following commands from the [Visual Studio Code command pallet](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette), typically accessed by pressing `ctrl` + `shift` + `p` and typing the command name you'd like to use from the table below.

| Name | Description |
|---|:---|
| ROS: Create Catkin Package | Create a catkin package. You can right click on a folder in the explorer to create it in a specific location. |
| ROS: Create Terminal | Create a terminal with the ROS environment. |
| ROS: Show Status | Open a detail view showing ROS core runtime status. |
| ROS: Start | Start ROS1 core or ROS2 Daemon. |
| ROS: Stop  | Terminate ROS core or ROS2 Daemon. |
| ROS: Update C++ Properties | Update the C++ IntelliSense configuration to include ROS and your ROS components. |
| ROS: Update Python Path | Update the Python IntelliSense configuration to include ROS. |
| ROS: Preview URDF | Preview URDF and Xacro files. The display will update after the root URDF changes are saved. |
| ROS: Install ROS Dependencies for this workspace using rosdep | Shortcut for `rosdep install --from-paths src --ignore-src -r -y`. |

## Tutorials and Walkthroughs

| Name | Description |
|---|:---|
| [Attaching to a running ROS Node][debug_support-attach] | Learn how to attach VSCode to a running ROS node |
| [Debugging all ROS Nodes in a launch file ][debug_support-launch] | Learn how to set up VSCode to debug the nodes in a ROS Launch file |
| [ROSCON 2019 ROS Extension Talk Video](https://vimeopro.com/osrfoundation/roscon-2019/video/379127667) | Walkthrough of VSCode from ROSCon 2019|


## Getting Started

The VSCode ROS extension will attempt to detect and automatically configure the workspace for the appropriate ROS Distro.

The extension will automatically start when you open a `catkin` or `colcon` workspace.
The build system (e.g. catkin_make or catkin build) will automatically be confirmed from the hidden files associated with
each system. 


## Launch Debugging

The Visual Studio Code extension for ROS supports launch debugging for ROS 1 and ROS 2 nodes. The VSCode extension currently supports debugging ROS written in Python and C++. The ROS node or nodes to be debugged must be placed in a ROS launch file with the extension `.launch` for ROS1 or ROS2 or with the extension `.py` for ROS2.

## Reporting Security Issues

Security issues and bugs should be reported privately, via email, to the Microsoft Security Response Center (MSRC) at [secure@microsoft.com](mailto:secure@microsoft.com). You should receive a response within 24 hours. If for some reason you do not, please follow up via email to ensure we received your original message.

Further information, including the [MSRC PGP](https://technet.microsoft.com/en-us/security/dn606155) key, can be found in the [Security TechCenter](https://technet.microsoft.com/en-us/security/default).

## Data and Telemetry

This extension collects usage data and sends it to Microsoft to help improve our products and services. Read our [privacy statement](https://privacy.microsoft.com/en-us/privacystatement) to learn more.

This extension respects the `telemetry.enableTelemetry` setting, learn more about [this option](https://code.visualstudio.com/docs/supporting/faq#_how-to-disable-telemetry-reporting).

## Contribution
Contributions are always welcome! Please see our [contributing guide][contributing] for more details!

A big ***Thank you!*** to everyone that have helped make this extension better!

* Andrew Short ([@ajshort](https://github.com/ajshort)), **original author**
* James Giller ([@JamesGiller](https://github.com/JamesGiller))

### ROS Web Tools
This extension leverages [ROS Web Tools](http://robotwebtools.org/) for URDF Previewing.

*Russell Toris, Julius Kammerl, David Lu, Jihoon Lee, Odest Chadwicke Jenkins, Sarah Osentoski, Mitchell Wills, and Sonia Chernova. [Robot Web Tools: Efficient Messaging for Cloud Robotics](http://robotwebtools.org/pdf/paper.pdf). In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2015*


<!-- link to files -->
<!-- relative links in Visual Studio Marketplace page lead to 404 error, need to use absolute link -->
[contributing]: https://github.com/ms-iot/vscode-ros/blob/master/CONTRIBUTING.md

<!-- feature documentation -->
[debug_support-attach]: https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#attach
[debug_support-launch]: https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#launch

<!-- media -->
[download_vsix_artifact]: https://raw.githubusercontent.com/ms-iot/vscode-ros/master/media/documentation/download-vsix-artifact.png

<!-- link to external sites -->
[ros]: http://ros.org
[vscode]: https://code.visualstudio.com
[vscode-ros-master-build_details]: https://github.com/ms-iot/vscode-ros/actions?query=event%3Apush
