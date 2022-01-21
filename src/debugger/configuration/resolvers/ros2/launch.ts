// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as yaml from "js-yaml";
import * as os from "os";
import * as path from "path";
import * as readline from "readline";
import * as shell_quote from "shell-quote";
import * as tmp from "tmp";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../../../extension";
import * as requests from "../../../requests";
import * as utils from "../../../utils";
import { rosApi } from "../../../../ros/ros";

const promisifiedExec = util.promisify(child_process.exec);

interface ILaunchRequest {
    nodeName: string;
    executable: string;
    arguments: string[];
    cwd: string;
    env: { [key: string]: string };
    symbolSearchPath?: string;
    additionalSOLibSearchPath?: string;
    sourceFileMap?: { [key: string]: string };
    launch?: string[];    // Scripts or executables to just launch without attaching a debugger
    attachDebugger?: string[];    // If specified, Scripts or executables to debug; otherwise attaches to everything not ignored
}

function getExtensionFilePath(extensionFile: string): string {
    return path.resolve(extension.extPath, extensionFile);
}

export class LaunchResolver implements vscode.DebugConfigurationProvider {
    // tslint:disable-next-line: max-line-length
    public async resolveDebugConfigurationWithSubstitutedVariables(folder: vscode.WorkspaceFolder | undefined, config: requests.ILaunchRequest, token?: vscode.CancellationToken) {
        if (!path.isAbsolute(config.target)) {
            throw new Error("Launch request requires an absolute path as target.");
        }
        else if (path.extname(config.target) !== ".py" && path.extname(config.target) !== ".xml") {
            throw new Error("Launch request requires an extension '.py' or '.xml' as target.");
        }

        const delay = ms => new Promise(res => setTimeout(res, ms));

        // Manage the status of the ROS2 Daemon, starting one if not present
        if (await rosApi.getCoreStatus() == false) {
            console.log("ROS Daemon is not active, attempting to start automatically");
            rosApi.startCore();

            // Wait for the core to start up to a timeout
            const timeout_ms: number = 30000;
            const interval_ms: number = 100;
            let timeWaited: number = 0;
            while (await rosApi.getCoreStatus() == false && 
                timeWaited < timeout_ms) {
                timeWaited += interval_ms;
                await delay(interval_ms);
            }

            console.log("Waited " + timeWaited + " for ROS2 Daemon to start");

            if (timeWaited >= timeout_ms) {
                throw new Error('Timed out (' + timeWaited / 1000 + ' seconds) waiting for ROS2 Daemon to start. Start ROS2 Daemon manually to avoid this error.');
            }
        }

        const rosExecOptions: child_process.ExecOptions = {
            env: {
                ...await extension.resolvedEnv(),
                ...config.env,
            },
        };

        let ros2_launch_dumper = getExtensionFilePath(path.join("assets", "scripts", "ros2_launch_dumper.py"));

        let args = []
        if (config.arguments) {
            for (let arg of config.arguments) {
                args.push(`"${arg}"`);
            }
        }
        let flatten_args = args.join(' ')
        let ros2_launch_dumper_cmdLine = (process.platform === "win32") ?
            `python ${ros2_launch_dumper} "${config.target}" ${flatten_args}` :
            `/usr/bin/env python3 ${ros2_launch_dumper} "${config.target}" ${flatten_args}`;

        let result = await promisifiedExec(ros2_launch_dumper_cmdLine, rosExecOptions);
        if (result.stderr) {
            throw (new Error(`Error from ROS2 launch dumper:\r\n ${result.stderr}`));
        } else if (result.stdout.length == 0) {
            throw (new Error(`ROS2 launch dumper unexpectedly produced no output.`));
        }

        let commands = result.stdout.split(os.EOL);
        commands.forEach(async (command) => {
            if (!command) {
                return;
            }

            let process = command.split(' ')[0];
            const launchRequest = this.generateLaunchRequest(process, command, config);
            if (launchRequest != null) {
              this.executeLaunchRequest(launchRequest, false);
            } else {
                const process = child_process.exec(command, rosExecOptions, (err, out) => {
                    if (err) {
                        throw (new Error(`Error from ${command}:\r\n ${err}`));
                    }
                })
            }
        });

        // @todo: error handling for Promise.all

        // Return null as we have spawned new debug requests
        return null;
    }


    private generateLaunchRequest(nodeName: string, command: string, config: requests.ILaunchRequest): ILaunchRequest {
        let parsedArgs: shell_quote.ParseEntry[];

        parsedArgs = shell_quote.parse(command);

        let executable = parsedArgs.shift().toString();

         // return rviz instead of rviz.exe, or spawner instead of spawner.py
         // This allows the user to run filter out genericly. 
        let executableName = path.basename(executable, path.extname(executable));

        // If this executable is just launched, don't attach a debugger.
        if (config.launch && 
            config.launch.indexOf(executableName) != -1) {
          return null;
        }

        // Filter shell scripts - just launch them
        //  https://github.com/ms-iot/vscode-ros/issues/474 
        let executableExt = path.extname(executable);
        if (executableExt && 
            ["bash", "sh", "bat", "cmd", "ps1"].includes(executableExt)) {
          return null;
        }

        // If a specific list of nodes is specified, then determine if this is one of them.
        // If no specific nodes specifed, attach to all unless specifically ignored.
        if (config.attachDebugger == null ||
          config.attachDebugger.indexOf(executableName) != -1) {

          const envConfig: { [key: string]: string; } = config.env;

          const request: ILaunchRequest = {
              nodeName: nodeName,
              executable: executable,
              arguments: parsedArgs.map((arg) => {
                  return arg.toString();
              }),
              cwd: ".",
              env: {
                  ...extension.env,
                  ...envConfig,
              },
              symbolSearchPath: config.symbolSearchPath, 
              additionalSOLibSearchPath: config.additionalSOLibSearchPath, 
              sourceFileMap: config.sourceFileMap
          };

          return request;
        }

        return null;
    }

    private async executeLaunchRequest(request: ILaunchRequest, stopOnEntry: boolean) {
        let debugConfig: ICppvsdbgLaunchConfiguration | ICppdbgLaunchConfiguration | IPythonLaunchConfiguration;

        if (os.platform() === "win32") {
            if (request.executable.toLowerCase().endsWith(".py")) {
                const pythonLaunchConfig: IPythonLaunchConfiguration = {
                    name: request.nodeName,
                    type: "python",
                    request: "launch",
                    program: request.executable,
                    args: request.arguments,
                    env: request.env,
                    stopOnEntry: stopOnEntry,
                    justMyCode: false,
                };
                debugConfig = pythonLaunchConfig;
            } else if (request.executable.toLowerCase().endsWith(".exe")) {
                interface ICppEnvConfig {
                    name: string;
                    value: string;
                }
                const envConfigs: ICppEnvConfig[] = [];
                for (const key in request.env) {
                    if (request.env.hasOwnProperty(key)) {
                        envConfigs.push({
                            name: key,
                            value: request.env[key],
                        });
                    }
                }
                const cppvsdbgLaunchConfig: ICppvsdbgLaunchConfiguration = {
                    name: request.nodeName,
                    type: "cppvsdbg",
                    request: "launch",
                    cwd: ".",
                    program: request.executable,
                    args: request.arguments,
                    environment: envConfigs,
                    stopAtEntry: stopOnEntry,
                    symbolSearchPath: request.symbolSearchPath,
                    sourceFileMap: request.sourceFileMap

                };
                debugConfig = cppvsdbgLaunchConfig;
            }

            if (!debugConfig) {
                throw (new Error(`Failed to create a debug configuration!`));
            }
            const launched = await vscode.debug.startDebugging(undefined, debugConfig);
            if (!launched) {
                throw (new Error(`Failed to start debug session!`));
            }
        } else {
            try {
                // this should be guaranteed by roslaunch
                fs.accessSync(request.executable, fs.constants.X_OK);
            } catch (errNotExecutable) {
                throw (new Error(`Error! ${request.executable} is not executable!`));
            }

            try {
                // need to be readable to check shebang line
                fs.accessSync(request.executable, fs.constants.R_OK);
            } catch (errNotReadable) {
                throw (new Error(`Error! ${request.executable} is not readable!`));
            }

            const fileStream = fs.createReadStream(request.executable);
            const rl = readline.createInterface({
                input: fileStream,
                crlfDelay: Infinity,
            });

            // we only want to read 1 line to check for shebang line
            let linesToRead: number = 1;
            rl.on("line", async (line) => {
                if (linesToRead <= 0) {
                    return;
                }
                linesToRead--;
                if (!linesToRead) {
                    rl.close();
                }

                // look for Python in shebang line
                if (line.startsWith("#!") && line.toLowerCase().indexOf("python") !== -1) {
                    const pythonLaunchConfig: IPythonLaunchConfiguration = {
                        name: request.nodeName,
                        type: "python",
                        request: "launch",
                        program: request.executable,
                        args: request.arguments,
                        env: request.env,
                        stopOnEntry: stopOnEntry,
                        justMyCode: false,
                    };
                    debugConfig = pythonLaunchConfig;
                } else {
                    interface ICppEnvConfig {
                        name: string;
                        value: string;
                    }
                    const envConfigs: ICppEnvConfig[] = [];
                    for (const key in request.env) {
                        if (request.env.hasOwnProperty(key)) {
                            envConfigs.push({
                                name: key,
                                value: request.env[key],
                            });
                        }
                    }
                    const cppdbgLaunchConfig: ICppdbgLaunchConfiguration = {
                        name: request.nodeName,
                        type: "cppdbg",
                        request: "launch",
                        cwd: ".",
                        program: request.executable,
                        args: request.arguments,
                        environment: envConfigs,
                        stopAtEntry: stopOnEntry,
                        additionalSOLibSearchPath: request.additionalSOLibSearchPath,
                        sourceFileMap: request.sourceFileMap,
                        setupCommands: [
                            {
                                text: "-enable-pretty-printing",
                                description: "Enable pretty-printing for gdb",
                                ignoreFailures: true
                            }
                        ]
                    };
                    debugConfig = cppdbgLaunchConfig;
                }

                if (!debugConfig) {
                    throw (new Error(`Failed to create a debug configuration!`));
                }
                const launched = await vscode.debug.startDebugging(undefined, debugConfig);
                if (!launched) {
                    throw (new Error(`Failed to start debug session!`));
                }
            });
        }
    }
}
