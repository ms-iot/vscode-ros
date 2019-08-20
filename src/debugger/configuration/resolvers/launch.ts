// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as yaml from "js-yaml";
import * as os from "os";
import * as path from "path";
import * as port_finder from "portfinder";
import * as shell_quote from "shell-quote";
import * as tmp from "tmp";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../../extension";
import * as requests from "../../requests";

const promisifiedExec = util.promisify(child_process.exec);

interface IRoslaunchRequest {
    executable: string;
    arguments: string[];
    cwd: string;
    env: { [key: string]: string };
}

export class LaunchResolver implements vscode.DebugConfigurationProvider {
    // tslint:disable-next-line: max-line-length
    public async resolveDebugConfiguration(folder: vscode.WorkspaceFolder | undefined, config: requests.ILaunchRequest, token?: vscode.CancellationToken) {
        if (!path.isAbsolute(config.target) || path.extname(config.target) !== ".launch") {
            throw new Error("Launch request requires an absolute path as target.");
        }

        const rosExecOptions: child_process.ExecOptions = {
            env: extension.env,
        };

        let result = await promisifiedExec(`roslaunch --dump-params ${config.target}`, rosExecOptions);
        const parameters = Object.keys(yaml.load(result.stdout));
        if (parameters && parameters.length) {
            // only call into rosparam when necessary
            const tmpFile = tmp.fileSync();
            fs.writeFile(`${tmpFile.name}`, result.stdout, async (error) => {
                if (error) {
                    throw error;
                }
                await promisifiedExec(`rosparam load ${tmpFile.name}`, rosExecOptions);
                tmpFile.removeCallback();
            });
        }

        result = await promisifiedExec(`roslaunch --nodes ${config.target}`, rosExecOptions);
        const nodes = result.stdout.trim().split(os.EOL);
        await Promise.all(nodes.map((node: string) => {
            return promisifiedExec(`roslaunch --args ${node} ${config.target}`, rosExecOptions);
        })).then((commands: Array<{ stdout: string; stderr: string; }>) => {
            for (const command of commands) {
                const roslaunchRequest = this.parseRoslaunchCommand(command.stdout);
                this.executeRoslaunchRequest(roslaunchRequest);
            }
        });
        return config;
    }

    private parseRoslaunchCommand(command: string): IRoslaunchRequest {
        // escape backslash in file path
        const parsedArgs = shell_quote.parse(command.replace(/[\\]/g, "\\$&"));

        const envConfig: { [key: string]: string; } = {};
        while (parsedArgs) {
            if (parsedArgs[0].toString().includes("=")) {
                const arg = parsedArgs.shift().toString();
                envConfig[arg.substring(0, arg.indexOf("="))] = arg.substring(arg.indexOf("=") + 1);
            } else {
                break;
            }
        }
        const request: IRoslaunchRequest = {
            executable: parsedArgs.shift().toString(),
            arguments: parsedArgs.map((arg) => {
                return arg.toString();
            }),
            cwd: ".",
            env: { ...extension.env, ...envConfig },
        };
        return request;
    }

    private async executeRoslaunchRequest(request: IRoslaunchRequest) {
        request.executable = request.executable.toLowerCase();
        if (request.executable.endsWith("python") || request.executable.endsWith("python.exe")) {
            const pythonScript: string = request.arguments.shift();
            const pythonlaunchdebugconfiguration: IPythonLaunchConfiguration = {
                name: `Python: launch`,
                type: "python",
                request: "launch",
                program: pythonScript,
                args: request.arguments,
                env: request.env,
                stopOnEntry: true,
                port: await port_finder.getPortPromise(),
            };
            const launched = await vscode.debug.startDebugging(undefined, pythonlaunchdebugconfiguration);
            if (!launched) {
                throw (new Error(`Failed to start debug session!`));
            }
        } else if (os.platform() === "win32" && request.executable.endsWith(".exe")) {
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
            const cpplaunchdebugconfiguration: ICppvsdbgLaunchConfiguration = {
                name: "C++: launch",
                type: "cppvsdbg",
                request: "launch",
                cwd: ".",
                program: request.executable,
                args: request.arguments,
                environment: envConfigs,
                stopAtEntry: true,
            };
            const launched = await vscode.debug.startDebugging(undefined, cpplaunchdebugconfiguration);
            if (!launched) {
                throw (new Error(`Failed to start debug session!`));
            }
        }
    }
}
