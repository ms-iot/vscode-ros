# Changelog
## 0.9.3
zero change release to prime the pre-release channel for future test releases.

## 0.9.2
Hot fix for ROS1 Debugging

## 0.9.1
* Revert change to filter XML and YAML files from launching.

## 0.9.0
## Thank you!
This extension has crossed an impressive milestone - 500,000 Installs! 

## New Features
* Includes the ability to debug ROS2 launch files using the new debug_launch type. 
> Notes:
    This only debugs the launch file itself - it does not launch any ROS nodes. 
    If your launch file does execute code, these may launch during debugging. 

* Smaller runtime, Faster Startup using WebPack
* `.urdf` files are now processed with `xacro`
* Fixes to ROS2 in a container

## What's Changed
* Use Webpack to reduce loading times. by @ooeygui in https://github.com/ms-iot/vscode-ros/pull/607
* Add the python analysis paths for ROS by @dthelegend in https://github.com/ms-iot/vscode-ros/pull/795
* Bump mocha from 9.2.2 to 10.0.0 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/717
* Bump @vscode/debugadapter from 1.56.1 to 1.57.0 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/766
* Update attach.js to enable pretty-printing for "ros attach" mode by @zhh2005757 in https://github.com/ms-iot/vscode-ros/pull/815
* Bump shell-quote from 1.7.3 to 1.7.4 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/811
* Bump tslib from 2.4.0 to 2.4.1 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/823
* Bump @types/mocha from 9.1.1 to 10.0.0 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/802
* Bump typescript from 4.7.4 to 4.8.4 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/794
* Bump portfinder from 1.0.28 to 1.0.32 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/774
* Lamadio/0.9 fixes by @ooeygui in https://github.com/ms-iot/vscode-ros/pull/898
* Bump @types/node from 17.0.45 to 18.15.7 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/897
* Bump ts-loader from 9.3.1 to 9.4.2 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/831
* Feature/debug ROS 2 launch files by @ooeygui in https://github.com/ms-iot/vscode-ros/pull/900
* Bump webpack from 5.74.0 to 5.76.3 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/896
* Bump typescript from 4.8.4 to 5.0.2 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/893
* Bump webpack-cli from 4.10.0 to 5.0.1 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/833
* Bump mocha and @types/mocha by @dependabot in https://github.com/ms-iot/vscode-ros/pull/837
* Bump tslib from 2.4.1 to 2.5.0 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/866
* Bump @vscode/debugadapter from 1.57.0 to 1.59.0 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/867
* Bump @types/vscode from 1.66.0 to 1.69.0 by @dependabot in https://github.com/ms-iot/vscode-ros/pull/752

## New Contributors
* @dthelegend made their first contribution in https://github.com/ms-iot/vscode-ros/pull/795
* @zhh2005757 made their first contribution in https://github.com/ms-iot/vscode-ros/pull/815

## 0.8.4
* makes 0.8.3 public 

## 0.8.3
* Use default workspace storage for IntelliSense db to avoid frequent writes to watched .vscode directory

## 0.8.2
* Correct ROS1 launch command that does not contain arguments

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
