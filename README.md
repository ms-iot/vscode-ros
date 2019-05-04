# ROS Extension for Visual Studio Code

[![Build Status](https://dev.azure.com/ros-win/ros-win/_apis/build/status/vscode-ros.ci?branchName=master)](https://dev.azure.com/ros-win/ros-win/_build/latest?definitionId=57&branchName=master)


The ROS Extension for [Visual Studio Code (VSCode)][vscode] provides support for [Robot Operating System (ROS)][ros] development. Providing an easier and more stream-lined developer experience.

***This fork from https://github.com/ajshort/vscode-ros is currently experimental.***

***This is still work in progress; it is still unstable and anything could be changed/removed.***

## Getting Started

The extension will automatically start when you open a catkin workspace.
The build system (e.g. catkin_make or catkin build) will automatically be confirmed from the hidden files associated with
each system.
The ROS distro will automatically be confirmed from the parent environment, or you will be prompted to select a ROS
distro if this can't be done automatically.

> You must build the catkin workspace at least once before the extension will recognise it.

To start ROS master, use the "ROS: Start Core" command. The "ROS master" indicator in the bottom left will show if the
master is currently running, and you can click on this to view parameters etc. If you hit F5 you can create a debug
configuration to run a `rosrun` or `roslaunch` command (*not yet supported on Windows*).

The first time you open the workspace the extension will automatically create build and test tasks and update the
C++ and Python paths. You can re-run this process later using the appropriate commands.

## Features

* Automatic ROS environment configuration.
* Allows starting, stopping and viewing the ROS master status.
* Automatically discover `catkin_make` or `catkin build` build tasks.
* Create catkin packages using `catkin_create_pkg` script or `catkin create pkg`.
* Run `rosrun` or `roslaunch` (breakpoints currently not supported).
* Syntax highlighting for `.msg`, `.urdf` and other ROS files.
* Automatically add the ROS C++ include and Python import paths.
* Format C++ using the ROS `clang-format` style.

## Commands

| Name | Command | Description | Supported on Windows? |
|:---:|:---:|:---|:---:|
| Create Catkin Package | `ros.createCatkinPackage` | Create a catkin package. You can right click on a folder in the explorer to create it in a specific location. | |
| Create Terminal | `ros.createTerminal` | Create a terminal with ROS sourced. | &check; |
| Show Master Status | `ros.showMasterStatus` | Open a detail view showing details about the ROS master. | &check; |
| Start Core | `ros.startCore` | Spawn a ROS core | &check; |
| Stop Core | `ros.stopCore` | Terminate the ROS core | &check; |
| Update C++ Properties | `ros.updateCppProperties` | Update the C++ include path to include ROS. | |
| Update Python Path | `ros.updatePythonPath` | Update the Python path to include ROS. | |

<!-- ## Roadmap -->

## Reporting Security Issues

Security issues and bugs should be reported privately, via email, to the Microsoft Security Response Center (MSRC) at [secure@microsoft.com](mailto:secure@microsoft.com). You should receive a response within 24 hours. If for some reason you do not, please follow up via email to ensure we received your original message.

Further information, including the [MSRC PGP](https://technet.microsoft.com/en-us/security/dn606155) key, can be found in the [Security TechCenter](https://technet.microsoft.com/en-us/security/default).

<!-- ## Data and Telemetry

The ROS Extension for Visual Studio Code collects usage data and sends it to Microsoft to help improve our products and services. Read our [privacy statement](https://privacy.microsoft.com/en-us/privacystatement) to learn more.

This extension respects the `telemetry.enableTelemetry` setting, learn more about [this option](https://code.visualstudio.com/docs/supporting/faq#_how-to-disable-telemetry-reporting). -->

## Contributors

A big ***Thank you!*** to everyone that have helped make this extension better!

- Andrew Short ([@ajshort](https://github.com/ajshort)), **original author**
- James Giller ([@JamesGiller](https://github.com/JamesGiller))

<!-- link to files -->
[changelog]: CHANGELOG.md
[contributing]: CONTRIBUTING.md

<!-- link to external sites -->
[ros]: http://ros.org
[vscode]: https://code.visualstudio.com
