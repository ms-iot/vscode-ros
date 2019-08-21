// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

interface IPythonLaunchConfiguration {
    // properties required by vscode.DebugConfiguration with hard-coded values
    type: "python";
    request: "launch";
    name: string;

    // properties defined in vscode-python's schema

    /**
     * Name of the module to be debugged
     */
    module?: string;

    /**
     * Absolute path to the program
     */
    program?: string;

    /**
     * Path (fully qualified) to python executable. Defaults to the value in settings.json
     */
    pythonPath?: string;

    /**
     * Command line arguments passed to the program
     */
    args?: string[];

    /**
     * Automatically stop after launch
     */
    stopOnEntry?: boolean;

    /**
     * Show return value of functions when stepping
     */
    showReturnValue?: boolean;

    /**
     * // Where to launch the debug target: internal console, integrated terminal, or external terminal
     */
    console?: "internalConsole" | "integratedTerminal" | "externalTerminal";

    /**
     * Absolute path to the working directory of the program being debugged. Default is the root directory of the file (leave empty)
     */
    cwd?: string;

    /**
     * Environment variables defined as a key value pair. Property ends up being the Environment Variable and the value of the property ends up being the value of the Env Variable
     */
    env?: { [key: string]: string; };

    /**
     * Absolute path to a file containing environment variable definitions
     */
    envFile?: string;

    /**
     * Debug port (default is 0, resulting in the use of a dynamic port)
     */
    port?: number;

    /**
     * IP address of the of the local debug server (default is localhost)
     */
    host?: string;

    /**
     * Enable logging of debugger events to a log file
     */
    logToFile?: boolean;

    /**
     * Redirect output
     */
    redirectOutput?: boolean;

    /**
     * Debug only user-written code
     */
    justMyCode?: boolean;

    /**
     * Enable debugging of gevent monkey-patched code
     */
    gevent?: boolean;

    /**
     * Django debugging
     */
    django?: boolean;

    /**
     * Jinja template debugging (e.g. Flask)
     */
    jinja?: true | false | null;

    /**
     * Running debug program under elevated permissions (on Unix)
     */
    sudo?: boolean;

    /**
     * Whether debugging Pyramid applications
     */
    pyramid?: boolean;

    /**
     * Whether to enable Sub Process debugging
     */
    subProcess?: boolean;
}

interface IPythonAttachConfiguration {
    // properties required by vscode.DebugConfiguration with hard-coded values
    type: "python";
    request: "attach";
    name: string;

    // properties defined in vscode-python's schema

    /**
     * Debug port to attach
     */
    port: number;

    /**
     * IP Address of the of remote server (default is localhost or use 127.0.0.1)
     */
    host?: string;

    /**
     * Path mappings
     */
    pathMappings?: Array<{ localRoot: string; remoteRoot: string; }>;

    /**
     * Enable logging of debugger events to a log file
     */
    logToFile?: boolean;

    /**
     * Redirect output
     */
    redirectOutput?: boolean;

    /**
     * Debug only user-written code
     */
    justMyCode?: boolean;

    /**
     * Django debugging
     */
    django?: boolean;

    /**
     * Jinja template debugging (e.g. Flask)
     */
    jinja?: true | false | null;

    /**
     * Whether to enable Sub Process debugging
     */
    subProcess?: boolean;

    /**
     * Show return value of functions when stepping
     */
    showReturnValue?: boolean;
}
