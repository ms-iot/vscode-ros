# Debug ROS nodes with `vscode-ros`

Debugging is one of the most important and needed feature of a developer's toolkit when it comes to software development.
To debug ROS nodes launched by `roslaunch`, ROS wiki suggests using the [`launch-prefix`][ros_wiki_debug] attribute defined in a `.launch` launch file with debugger commands like this:

| `launch-prefix` | usage |
| :---: | :---: |
| `gdb -ex run --args` | run node in `gdb` in the same `xterm` |
| `valgrind` | run node in `valgrind` |

Currently, this approach only provides [limited](#why-no-launch-prefix-support-for-rospy) debugger support to `roscpp` nodes.

## Goals

* enable attaching into a local C++ or Python process launched through `roslaunch` to debug that process
* enable debugging C++ and Python nodes specified in a `.launch` file at entry

### Non-goals

Language-specific debugging capabilities including:

* complicated debugging scenarios like debugging a remote process

We woult want to utilize existing language-specific tools for [C++][ms-vscode.cpptools] and [Python][ms-python.python]:

![architecture][architecture]

## Debugging in Visual Studio Code

Typically, to start a debugging sessions, a debug configuration should be created in `launch.json` first.
After that, the newly created configuration could be selected and executed by pressing the green arrow or the `F5` key.

![execute_a_debug_configuration][execute_a_debug_configuration]

What happens afterwards gets handled by 2 processes:

* the running Visual Studio Code instance (this is the running context of all extensions)
* the debug adapter instance (for each language)

A debug configuration first gets resolved by a registered debug configuration provider in VS Code's context.
Afterwards, a debug request gets sent to the extension’s debug adapter, which actually handles the debugging process.
However, the debug adapter is a separate process from the VS Code process, so no `vscode` apis could be called from there.

In our scenario, we need to do everything in the extension context to utilize the functionalities provided by other extensions.
Ideally, the workflow should be as simple as possible:

1. the user creates a `ros`-type debug configuration
2. our extension resolves the configuration
3. our extension generates debug configuration for C++ or/and Python extensions

![debug_flow][debug_flow]

As illustrated in the above flow chart, we would also [need a stub debug adapter](#why-is-a-stub-debug-adapter-needed).

## Attach

Our goal for attach-debug is to provide a way to debug a ROS node while not changing anything in an existing flow.
This means when the user executes a `.launch` file with `roslaunch` just like before, he/she could use `vscode-ros` to easily start an attach-debug session to debug a running ROS node process.
All attach-debug configurations are propagated to language-specific extensions.

![attach_debug][attach_debug]

### Attach into a C++ process

Just like using Visual Studio or windbg, attaching into a running C++ process could be achieved with `vscode-cpptools` easily.
Our extension automatically chooses `cppvsdbg` for MSVC debugger when it's running on Windows, and `cppdbg` for gdb-based debugger otherwise.

### Attach into a Python process

Our extension uses [`ptvsd`][ptvsd] to enable debugging Python scripts.
Generally, a Python script needs to be launched through the `ptvsd` module to be able to be debugged:

```bash
python -m ptvsd –host <host> --port <port> <script>
```

Once a script is launched like this, a Python attach session could be launched inside VS Code to attach to the `ptvsd` server running inside the process.

However, Python processes launched by ROS tools (`rosrun` and `roslaunch`) are usually launched natively.
To debug a Python process without a running `ptvsd` instance, a `ptvsd` instance needs to be injected into the process first with the `--pid <pid>` flag.
Therefore, for normal Python processes in ROS, the workflow would be:

1. get the pid of the Python process (we reuse the `processPicker` module from `vscode-cpptools`)
2. inject `ptvsd` using API exposed by `vscode-python` (this avoids potential `ptvsd` version mismatch)
3. start an Python-attach debug session with `vscode-python`

### Example in `launch.json`

```json
{
    "configurations": [
        {
            "name": "ROS: Attach",
            "type": "ros",
            "request": "attach"
        },
        {
            "name": "ROS: Attach to Python",
            "type": "ros",
            "request": "attach",
            "runtime": "Python"
        }
    ]
}
```

### Note

Microsoft's VS Code extensions for [C++][ms-vscode.cpptools] and [Python][ms-python.python] provide flexible mechanisims of attaching into a [C++][vscode_cpp_debug] or [Python][vscode_python_debug] process.
This extension only aims to enable a basic and generic attach-debug flow.
Please use language-specific extensions for any specific debugging requirements including:

* specifying `symbolSearchPath` or `miDebuggerPath` for C++ debugging
* attaching to and debugging a remote process

<!-- ## Launch

Our goal for launch-debug is to provide a native-like debugging experience for ROS nodes launched from roslaunch, which means enabling debugging multiple ROS nodes at the same time. Just like debugging a C++ program or a Python script with a single debug session in VS Code, we want to enable the user to start multiple debug sessions when a launch-type debug configuration is executed.

Very similar to attach-debug, we want to utilize all the language-specific debugging functionalities provided by vscode-python and vscode-cpptools. In this configuration, we are also going to provide a “stopOnEntry” flag that controls both the “stopOnEntry” flag for Python debugging and “stopAtEntry” flag for C++ debugging. If “stopOnEntry” is set to False, all processes will execute till a breakpoint is hit.
The roslaunch executable needs to be updated to support a “--fake” or “--debug” flag, which specifies the launcher not to actually start any of the ROS nodes, but only print out their launch requests (executable, arguments, work directory, environment variables). The VS Code extension would execute the roslaunch command with the new “--debug” flag ,parse the launch requests in the output, and start debug sessions based on the type of the ROS node.
For the same reason mentioned above, a stub debug adapter that self-terminates immediately is still needed. The entire workflow would look like this:

Why not use roslaunch’s non-launch options ?
A couple of reasons:
1.	roslaunch’s non-launch command does not output the exact same command that’s used to launch the node. For example, for a node defined like this:
<node pkg="pubsub" type="talker" name="talker_py_1" args="$(arg foo)" />
Calling roslaunch to get its launch command would give us:
PS C:\ros\catkin_ws\dev_ros_comm> roslaunch --nodes C:\ros\catkin_ws\dev_ros_comm\src\beginner_tutorials\launch\minimal.win.launch
/talker_py_1
PS C:\ros\catkin_ws\dev_ros_comm> roslaunch --args /talker_py_1 C:\ros\catkin_ws\dev_ros_comm\src\beginner_tutorials\launch\minimal.win.launch
python C:\ros\catkin_ws\dev_ros_comm\src\pubsub\scripts\talker.py foo __name:=talker_py_1

However, during runtime, roslaunch appends another argument "__log:=C:\Users\kejxu\.ros\log\a786f84f-bd4e-11e9-965e-480fcf49b453\talker_py_1-1.log"

2. roslaunch is needed for <param> tags
since <param> tags are only loaded onto the parameter server, the launch file still needs to be executed so that all parameters are set correctly.

3. `<env>` tags
When using the “--args” flag, environment variable configurations would show up as Linux-style env-set commands (someenv=somevalue), which would not work on Windows

Limitations

Potentially, when too many debug sessions are launched at the same time, it would be painful to terminate them one by one.
The launch-debug configuration provided by vscode-python reuses the same port number, when multiple Python processes are launched, only one of those would eventually connect. However, we would still want to use launch-debug configuration since this is the only way we could terminate the process at the same time we terminate the debug session. This requires an update from the vscode-python side.
Adding a debug=True attribute to the roslaunch/node xml specification?
Launch file in ROS could contain from 1 single node to many nodes. Launching debug sessions for all of those could be seriously resource draining and in turn easily cause performance issues. Potentially it could be helpful if the user could add a debug=True attribute to the node(s) to debug specific node(s).
However, since this means changing the ROS launch file XML specification, it potentially requires all launchers (including roslaunch) to be able to understand and consume this flag. The cost of adding this support to roslaunch is unknown and could potentially mean a much larger effort.
The work-around for this is to use multiple launch files, and only one of those is specifically for debugging purpose.
A Node.js-based ROS launcher specifically for VS Code?
Since VS Code runs in a Node.js context, another possible solution would be to implement a new ROS launcher (to replace roslaunch for debugging) in JavaScript/TypeScript.
Very similar implementation but in C++ has been attempted at https://wiki.ros.org/rosmon. Good thing about this approach is that we will not need to go through the upstreaming process, since we are creating new code; yet that is also the bad thing, any new code we publish creates maintenance burden on a already limited bandwidth. Not to mention the engineering cost of porting any potential change/update that roslaunch might get from the community in the future. -->

## Appendix

### Why no `launch-prefix` support for `rospy`

The ROS Wiki suggested [`launch-prefix`][ros_wiki_debug] attribute is only supported for `roscpp` nodes, and this is our take on why it is not for `rospy` too.

All `roscpp` nodes are compiled C++ binaries (executables), so a `roscpp` node called `talker` could be launched directly from the command line like this:

```bash
./talker
```

and launching it with a debugger is straightforward:

```bash
gdb -ex run --args ./talker

valgrind ./talker
```

At the same time, `rospy` nodes are Python scripts made executable (on Linux) by adding a [shebang line][wikipedia_shebang], the actual executable is the Python interpreter:

```bash
./talker

# when talker.py has #!/usr/bin/python as the shebang line, the above line just means
/usr/bin/python ./talker
```

Typically, to debug a Python script, one would need to use either [`pdb`][pdb] or [`ptvsd`][ptvsd]:

```bash
python -m pdb ./talker.py

# ptvsd only starts a debugging server inside the process
python -m ptvsd --host localhost  --port 5678 --wait ./talker.py
```

To have these as the final commands, everything before `./talker.py` need to be part of the shebang line.
However, that would be a bad choice since generally the script would/should not be launched in debug mode.
For this reason, there is no place to insert `launch-prefix` flags for `rospy` nodes.

This is a good example why it is suggested to keep the `.py` filename extension, and create separate a separate [console script][setuptools_console_scripts].

### Why is a stub debug adapter needed

As illustraed in the flow charts above, a stub debug adapter that self-terminates immediately is still needed by `vscode-ros` to handle debug requests.
These debug requests are generated from the `ros`-type debug configurations.
When debug configurations are executed, they are resolved into debug requests and then sent to corresponding debug adapters.
It would lead to errors generated in VS Code if no debug adapter is there to handle the debug requests.

<!-- link to files -->
[architecture]: ../media/documentation/debug-ros-nodes/architecture.png
[execute_a_debug_configuration]: ../media/documentation/debug-ros-nodes/execute-a-debug-configuration.png
[debug_flow]: ../media/documentation/debug-ros-nodes/debug-flow.png
[attach_debug]: ../media/documentation/debug-ros-nodes/attach-debug.png

<!-- external links -->
[ros_wiki_debug]: http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB

[ms-python.python]: https://marketplace.visualstudio.com/items?itemName=ms-python.python
[ms-vscode.cpptools]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
[vscode_python_debug]: https://code.visualstudio.com/docs/python/debugging
[vscode_cpp_debug]: https://code.visualstudio.com/docs/cpp/launch-json-reference

[wikipedia_shebang]: https://en.wikipedia.org/wiki/Shebang_(Unix)
[pdb]: https://docs.python.org/2/library/pdb.html
[ptvsd]: https://github.com/microsoft/ptvsd
[setuptools_console_scripts]: https://packaging.python.org/specifications/entry-points/
