# Changelog

## 0.6.1

* Enable support for launch-debug a ROS node
* Update environment sourcing in `ros.createTerminal` to work with user `.bashrc`, [#123](https://github.com/ms-iot/vscode-ros/pull/123)
* Update extension to source workspace environment after a ROS build task
* Fix task provider usage
* Fix debug config provider upon initialiazing a `launch.json` file

## 0.6.0

* Add support for ROS2 support
* Add support for attach-debug a ROS node
* Automate ROS distro selection
* Fix `rosrun` and `roslaunch` command execution
* Implementation task provider for `catkin_make_isolated`

## 0.5.0

* Enable previewing URDF and Xacro files
* Fix bugs in ROS core monitor

## 0.4.5

* Require `vscode` 1.26
* Enable launching and terminating `roscore` on Windows
* Update ROS core monitor implementation with webview API
* Fix `sourceRosAndWorkspace()` for workspaces built with `catkin_make_isolated`
* Fix `findPackageFiles()` for Windows
* Replace all `ROS master` instances with `ROS core`

## 0.3.0

* Automatically add workspace package include dirs to the include path.
* Fix debug configuration creation.

## 0.2.0

* Require `vscode` 1.18
* Add support for catkin tools alongside catkin_make (thanks to @JamesGiller).
* Remove some unimplemented commands.
* Add "ROS: Create Terminal" command.

## 0.1.0

* Require `vscode` 1.14
* Automatically discover catkin make tasks.
* Don't error when no args are specified (#3).

## 0.0.1

* Initial release.
