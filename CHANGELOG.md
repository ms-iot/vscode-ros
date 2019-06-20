# Changelog

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
