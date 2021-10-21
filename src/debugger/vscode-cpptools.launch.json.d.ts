// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

interface ICppdbgLaunchConfiguration {
    // properties required by vscode.DebugConfiguration with hard-coded values
    type: "cppdbg";
    request: "launch";
    name: string;

    // properties defined in vscode-cpptools' schema

    /**
     * Full path to program executable
     */
    program: string;

    /**
     * Command line arguments passed to the program
     */
    args?: string[];

    /**
     * The architecture of the debuggee. This will automatically be detected unless this parameter is set. Allowed values are x86, arm, arm64, mips, x64, amd64, x86_64.
     */
    targetArchitecture?: string;

    /**
     * The working directory of the target
     */
    cwd?: string;

    /**
     * One or more GDB/LLDB commands to execute in order to setup the underlying debugger. Example: \"setupCommands\": [ { \"text\": \"-enable-pretty-printing\", \"description\": \"Enable GDB pretty printing\", \"ignoreFailures\": true }].
     */
    setupCommands?: Array<{ text: string; description: string; ignoreFailures: boolean; }>;

    /**
     * If provided, this replaces the default commands used to launch a target with some other commands. For example, this can be \"-target-attach\" in order to attach to a target process. An empty command list replaces the launch commands with nothing, which can be useful if the debugger is being provided launch options as command line options. Example: \"customLaunchSetupCommands\": [ { \"text\": \"target-run\", \"description\": \"run target\", \"ignoreFailures\": false }].
     */
    customLaunchSetupCommands?: Array<{ text: string; description: string; ignoreFailures: boolean; }>;

    /**
     * The command to execute after the debugger is fully setup in order to cause the target process to run. Allowed values are \"exec-run\", \"exec-continue\", \"None\". The default value is \"exec-run\".
     */
    launchCompleteCommand?: "exec-run" | "exec-continue" | "None";

    /**
     * .natvis file to be used when debugging this process. This option is not compatible with GDB pretty printing. Please also see \"showDisplayString\" if using this setting.
     */
    visualizerFile?: string;

    /**
     * When a visualizerFile is specified, showDisplayString will enable the display string. Turning this option on can cause slower performance during debugging.
     */
    showDisplayString?: boolean;

    /**
     * Environment variables to add to the environment for the program. Example: [ { \"name\": \"squid\", \"value\": \"clam\" } ].
     */
    environment?: Array<{ name: string; value: string; }>;

    /**
     * Absolute path to a file containing environment variable definitions. This file has key value pairs separated by an equals sign per line. E.g. KEY=VALUE
     */
    envFile?: string;

    /**
     * Semicolon separated list of directories to use to search for .so files. Example: \"c:\\dir1;c:\\dir2\".
     */
    additionalSOLibSearchPath?: string;

    /**
     * Indicates the console debugger that the MIDebugEngine will connect to. Allowed values are \"gdb\" \"lldb\".
     */
    MIMode?: string;

    /**
     * The path to the mi debugger (such as gdb). When unspecified, it will search path first for the debugger.
     */
    miDebuggerPath?: string;

    /**
     * Arguments for the mi debugger (such as gdb) to use. When unspecified.
     */
    miDebuggerArgs?: string;

    /**
     * Network address of the MI Debugger Server to connect to (example: localhost:1234).
     */
    miDebuggerServerAddress?: string;

    /**
     * Optional parameter. If true, the debugger should stop at the entrypoint of the target. If processId is passed, has no effect.
     */
    stopAtEntry?: boolean;

    /**
     * Optional full path to debug server to launch. Defaults to null.
     */
    debugServerPath?: string;

    /**
     * Optional debug server args. Defaults to null.
     */
    debugServerArgs?: string;

    /**
     * Optional server-started pattern to look for in the debug server output. Defaults to null.
     */
    serverStarted?: string;

    /**
     * Search stdout stream for server-started pattern and log stdout to debug output. Defaults to true.
     */
    filterStdout?: boolean;

    /**
     * Search stderr stream for server-started pattern and log stderr to debug output. Defaults to false.
     */
    filterStderr?: boolean;

    /**
     * Optional time, in milliseconds, for the debugger to wait for the debugServer to start up. Default is 10000.
     */
    serverLaunchTimeout?: number;

    /**
     * Optional full path to a core dump file for the specified program. Defaults to null.
     */
    coreDumpPath?: string;

    /**
     * If true, a console is launched for the debuggee. If false, on Linux and Windows, it will appear in the Integrated Console.
     */
    externalConsole?: boolean;

    /**
     * If true, disables debuggee console redirection that is required for Integrated Terminal support.
     */
    avoidWindowsConsoleRedirection?: boolean;

    /**
     * Optional source file mappings passed to the debug engine. Example: '{ \"/original/source/path\":\"/current/source/path\" }'
     */
    sourceFileMap?: { [key: string]: string; };

    /**
     * Optional flags to determine what types of messages should be logged to the Debug Console.
     */
    logging?: {
        exceptions: boolean;
        moduleLoad: boolean;
        programOutput: boolean;
        engineLogging: boolean;
        trace: boolean;
        traceResponse: boolean;
    };

    /**
     * When present, this tells the debugger to connect to a remote computer using another executable as a pipe that will relay standard input/output between VS Code and the MI-enabled debugger backend executable (such as gdb).
     */
    pipeTransport?: {
        pipeCwd: string;
        pipeProgram: string;
        pipeArgs: string[];
        debuggerPath: string;
        pipeEnv: { [key: string]: string; };
    }
}

interface ICppdbgAttachConfiguration {
    // properties required by vscode.DebugConfiguration with hard-coded values
    type: "cppdbg";
    request: "attach";
    name: string;

    // properties defined in vscode-cpptools' schema

    /**
     * Full path to program executable.
     */
    program: string;

    /**
     * Optional process id to attach the debugger to. Use \"${command:pickProcess}\" to get a list of local running processes to attach to. Note that some platforms require administrator privileges in order to attach to a process.
     */
    processId: "${command:pickProcess}" | number;

    /**
     * The architecture of the debuggee. This will automatically be detected unless this parameter is set. Allowed values are x86, arm, arm64, mips, x64, amd64, x86_64.
     */
    targetArchitecture?: string;

    /**
     * .natvis file to be used when debugging this process. This option is not compatible with GDB pretty printing. Please also see \"showDisplayString\" if using this setting.
     */
    visualizerFile?: string;

    /**
     * When a visualizerFile is specified, showDisplayString will enable the display string. Turning this option on can cause slower performance during debugging.
     */
    showDisplayString?: boolean;

    /**
     * Semicolon separated list of directories to use to search for .so files. Example: \"c:\\dir1;c:\\dir2\".
     */
    additionalSOLibSearchPath?: string;

    /**
     * Indicates the console debugger that the MIDebugEngine will connect to. Allowed values are \"gdb\" \"lldb\".
     */
    MIMode?: string;

    /**
     * The path to the mi debugger (such as gdb). When unspecified, it will search path first for the debugger.
     */
    miDebuggerPath?: string;

    /**
     * Network address of the MI Debugger Server to connect to (example: localhost:1234).
     */
    miDebuggerServerAddress?: string;

    /**
     * Search stdout stream for server-started pattern and log stdout to debug output. Defaults to true.
     */
    filterStdout?: boolean;

    /**
     * Search stderr stream for server-started pattern and log stderr to debug output. Defaults to false.
     */
    filterStderr?: boolean;

    /**
     * Optional source file mappings passed to the debug engine. Example: '{ \"/original/source/path\":\"/current/source/path\" }'
     */
    sourceFileMap?: { [key: string]: string; };

    /**
     * Optional flags to determine what types of messages should be logged to the Debug Console.
     */
    logging?: {
        exceptions: boolean;
        moduleLoad: boolean;
        programOutput: boolean;
        engineLogging: boolean;
        trace: boolean;
        traceResponse: boolean;
    };

    /**
     * When present, this tells the debugger to connect to a remote computer using another executable as a pipe that will relay standard input/output between VS Code and the MI-enabled debugger backend executable (such as gdb).
     */
    pipeTransport?: {
        pipeCwd: string;
        pipeProgram: string;
        pipeArgs: string[];
        debuggerPath: string;
        pipeEnv: { [key: string]: string; };
    };

    /**
     * One or more GDB/LLDB commands to execute in order to setup the underlying debugger. Example: \"setupCommands\": [ { \"text\": \"-enable-pretty-printing\", \"description\": \"Enable GDB pretty printing\", \"ignoreFailures\": true }].
     */
    setupCommands?: Array<{
        text: string;
        description: string;
        ignoreFailures: boolean;
    }>;
}

interface ICppvsdbgLaunchConfiguration {
    // properties required by vscode.DebugConfiguration with hard-coded values
    type: "cppvsdbg";
    request: "launch";
    name: string;

    // properties defined in vscode-cpptools' schema

    /**
     * Full path to program executable.
     */
    program: string;

    /**
     * The working directory of the target.
     */
    cwd: string;

    /**
     * Command line arguments passed to the program.
     */
    args?: string[];

    /**
     * Environment variables to add to the environment for the program. Example: [ { \"name\": \"squid\", \"value\": \"clam\" } ].
     */
    environment?: Array<{
        name: string;
        value: string;
    }>;

    /**
     * Absolute path to a file containing environment variable definitions. This file has key value pairs separated by an equals sign per line. E.g. KEY=VALUE
     */
    envFile?: string;

    /**
     * Semicolon separated list of directories to use to search for symbol (that is, pdb) files. Example: \"c:\\dir1;c:\\dir2\".
     */
    symbolSearchPath?: string;

    /**
     * Optional parameter. If true, the debugger should stop at the entrypoint of the target. If processId is passed, has no effect.
     */
    stopAtEntry?: boolean;

    /**
     * Optional full path to a dump file for the specified program. Example: \"c:\\temp\\app.dmp\". Defaults to null.
     */
    dumpPath?: string;

    /**
     * .natvis file to be used when debugging this process.
     */
    visualizerFile?: string;

    /**
     * If true, a console is launched for the debuggee. If false, no console is launched.
     */
    externalConsole?: boolean;

    /**
     * Optional source file mappings passed to the debug engine. Example: '{ \"/original/source/path\":\"/current/source/path\" }'
     */
    sourceFileMap?: { [key: string]: string };

    /**
     * If false, the process will be launched with debug heap disabled. This sets the environment variable '_NO_DEBUG_HEAP' to '1'.
     */
    enableDebugHeap?: boolean;

    /**
     * Optional flags to determine what types of messages should be logged to the Debug Console.
     */
    logging?: {
        exceptions: boolean;
        moduleLoad: boolean;
        programOutput: boolean;
        engineLogging: boolean;
    };
}

interface ICppvsdbgAttachConfiguration {
    // properties required by vscode.DebugConfiguration with hard-coded values
    type: "cppvsdbg";
    request: "attach";
    name: string;

    // properties defined in vscode-cpptools' schema

    /**
     * Optional process id to attach the debugger to. Use \"${command:pickProcess}\" to get a list of local running processes to attach to. Note that some platforms require administrator privileges in order to attach to a process.
     */
    processId: "${command:pickProcess}" | number;

    /**
     * Semicolon separated list of directories to use to search for symbol (that is, pdb) files. Example: \"c:\\dir1;c:\\dir2\".
     */
    symbolSearchPath?: string;

    /**
     * .natvis file to be used when debugging this process.
     */
    visualizerFile?: string;

    /**
     * Optional source file mappings passed to the debug engine. Example: '{ \"/original/source/path\":\"/current/source/path\" }'
     */
    sourceFileMap?: { [key: string]: string; };

    /**
     * Optional flags to determine what types of messages should be logged to the Debug Console.
     */
    logging?: {
        exceptions: boolean;
        moduleLoad: boolean;
        programOutput: boolean;
        trace: boolean;
    };
}
