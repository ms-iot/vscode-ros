# Debug ROS nodes with `vscode-ros`

Debugging is one of the most important and needed feature of a developer's toolkit when it comes to software development.
To debug ROS nodes launched by `roslaunch`, ROS wiki suggests using the [`launch-prefix`][ros_wiki_debug] attribute defined in a `.launch` launch file with debugger commands like this:

| `launch-prefix` | usage |
| :---: | :---: |
| `gdb -ex run --args` | run node in `gdb` in the same `xterm` |
| `valgrind` | run node in `valgrind` |

Currently, this approach only provides [limited](#why-launch-prefix-only-supports-roscpp) debugger support to `roscpp` nodes.

## Goals

The goal of adding debugging support is to facilitate an otherwise complicated process.
For example, debugging a ROS node launched through roslaunch, which is generally a C++ executable or a Python script, at entry.

### Non-goals

Language-specific debugging capabilities including:

- debugging a single C++ executable or a single Python script
- complex debugging scenarios (debugging a remote process)

We want to utilize existing language-specific tools for [C++][ms-vscode.cpptools] and [Python][ms-python.python].

<!-- Visual Studio Code Debug Flow
The common debugging workflow in VS Code consists of 2 separate processes: the running Visual Studio Code instance and the debug adapter instance. The typical (and most of the cases, the only) workflow is to create a debug configuration first in launch.json, then press the green arrow or press F5 to execute the debug configuration.

What happens under the surface gets divided into the 2 processes mentioned above. A debug configuration first gets parsed (a.k.a. resolved) by a registered debug configuration provider (we call it resolver in our code because of the functionality it provides), then a request gets sent to the extension’s debug adapter, which is the component that actually handles the debugging process. Since the debug adapter is a separate process from the VS Code process, no vscode apis could be called from there. Therefore, since we want to utilize the functionalities from other extensions, we need to do everything in the extension itself. Ideally, we want the workflow to be as simple as possible: the user creates a ros-type debug configuration, our extension calls into C++ or/and Python extension while resolving the configuration, and the language-specific debug adapter takes care of the rest.

Attach
Our goal for attach-debug is to provide a tool that can help debug a ROS node while not changing any existing flow. That is to say, the user could execute a launch file with roslaunch just like before, and then use vscode-ros to start a attach-debug session to debug a running ROS node process.

Attach into a C++ process
Attaching into a running process for debugging is generally simple for C++ (consider attaching into a C++ process using Visual Studio or windbg). Thanks to work from vscode-cpptools team, if vscode-cpptools is installed, an attach-type debug configuration  could be used to debug a running C++ process using the Visual C++ debugger. This could be achieved without any extra work.
Aside from that, there is still space for improvement. Since vscode-cpptools uses different debug types for different debuggers (cppdbg for gdb-based debugger and cppvsdbg for MSVC debugger), our debugging support could automatically populate the correct debug configuration depending on the platform.

Attach into a Python process
Meantime, since ROS consists of a lot of Python, it could be a bit more complicated. To debug a Python program/script, there are multiple options. Python ships with its command-line debugger (pdb) , which is harder to use compared to visual debuggers; and there is ptvsd  for Visual Studio and VS Code. Generally, to debug a Python script with ptvsd, the script needs to be launched through the ptvsd module (python -m ptvsd –host <host> --port <port> script), and then a attach-debug session could be launched in VS Code to attach to the process (ptvsd server running inside the process). To debug a native Python process, ptvsd needs to be injected  into the process first by using the “—pid <pid>” flag.
Since our goal for attach-debug is to debug a normal ROS node launched through roslaunch, we can assert that the (Python) process would be a native process without any running ptvsd instance. In this case, we need to inject the process with a ptvsd instance before we can propagate the debug request to vscode-python and utilize their debugging functionalities. There are 2 things we need to do for that purpose:
1.	get the pid of the process that we want to debug (this can reuse the processPicker module from vscode-cpptools)
2.	inject ptvsd into the process with command from vscode-python (this avoids potential version mismatch for ptvsd)
After we have injected a ptvsd instance into the Python process, we can start an attach-debug session for Python and utilize vscode-python’s debugger.
To make it all work
We still need a stub implementation for debug adapter that self-terminates immediately. Not having a debug adapter that handles the debug requests would lead to errors after the debug configuration is resolved. The entire workflow would look like this:
 
Limitations
Since such an attach-debug solution (choose a running process->attach) is very generic, it would be hard to support more complicated or advanced debugging scenarios; for example, debugging a ROS node running on a remote computer.
Launch
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

### Why `launch-prefix` only supports `roscpp`

This is because `rospy` nodes are Python scripts made executable (on Linux) by adding a shebang line, the actual executable is the Python interpreter.

<!-- external links -->
[ros_wiki_debug]: http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB

[ms-python.python]: https://marketplace.visualstudio.com/items?itemName=ms-python.python
[ms-vscode.cpptools]: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
