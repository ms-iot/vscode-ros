# Changelog

## 0.8.1
* Update Dependencies
* Update Docs
* Add Video Deep Dives
* [645] Fix for launch ROS1 file parameters
* [678] Ability to specify a startup script instead of inferring from distro
* [671] Prevent spurreous output and stderr with output from preventing ROS2 launch file debugging

## 0.8.0
* Update Dependencies
* [#654] Ignore Scripts such as .sh, .bash, .bat, .ps1 and .cmd
* [#655] Support Debugging on Jetson Nano by fixing the launch debugging python script
* [#656] Fix Lifecycle node debugging, by addressing a hanging background thread.

## 0.7.0
* Update 3rd party disclosure and attribution
* Update Dependencies
* [#544] Contribution by @vigiroux - Bogus Screen Size
* [#587] Enable System Symbols
* [#594] Improve IntelliSense by updating the C++ properties correctly
* [#605] Support *Experimental* launch filtering - enable 'launch only' and 'debug only' Filters. 
* [#608] Reduce annoying dialogs from the ROS extension

## 0.6.9
* [#429] Contribution by RyanDraves - Start ROS Core/Daemon if not running when launch debugging
* [#435] Contribution by RyanDraves - Support ROS Test launch file debugging
* [#470] Support VSCode Trusted Workspaces feature
* [#476] Fixes to auto-launch
* [#498] Support GDB Pretty Printing for Watch Window
* [#497] No longer droping Log folders all over
* [#499] ROS2 colcon Fixes, neutralize commands between ROS1 and ROS2
* [#501] Fixes when using ROS in a container
* Update many dependencies
* Update documentation

## 0.6.8

* [#443]https://github.com/ms-iot/vscode-ros/issues/443) URDF preview not working
* Updated dependencies
* Updated attribution

## 0.6.7

* [#391](https://github.com/ms-iot/vscode-ros/pull/391) Launch debug args is optional. (by @seanyen)

## 0.6.6

* [#372](https://github.com/ms-iot/vscode-ros/pull/372) Adding error messages on exceptions for better diagnosis (by @seanyen)
* [#371](https://github.com/ms-iot/vscode-ros/pull/371) Adding the missing version for cpp properties. (by @seanyen)
* [#368](https://github.com/ms-iot/vscode-ros/pull/368) [ROS2] Adding Initial ROS2 Launch Debug Support (by @seanyen)

## 0.6.5

* [#365](https://github.com/ms-iot/vscode-ros/pull/365) [ros2] find all launch files (by @Hexafog)
* [#310](https://github.com/ms-iot/vscode-ros/pull/310) Allow customization of build tool tasks (by @anton-matosov)
* [#321](https://github.com/ms-iot/vscode-ros/pull/321) Have executable search follow symbolic links (by @nightduck)
* [#304](https://github.com/ms-iot/vscode-ros/pull/304) Handle preLaunch task explicitly (by @anton-matosov)

## 0.6.4

* [#241](https://github.com/ms-iot/vscode-ros/pull/241) Fix task provider name mismatch (by @humanoid2050)
* [#262](https://github.com/ms-iot/vscode-ros/pull/262) Add error handling for ROS launch debug (by @ooeygui)
* [#263](https://github.com/ms-iot/vscode-ros/pull/263) Fix URDF Preview not functional with vscode v1.47 (by @seanyen)

## 0.6.3

* Enable `ros.rosdep` extension command.
* Fix roslaunch C++ node debugging on Windows.

## 0.6.2

* Maintenance release
* Display `ROS` version and distro for status

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
