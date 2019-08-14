// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as vscode from "vscode";

import * as ros from "../ros";

export class ROS2 implements ros.ROSApi {
    private _context: vscode.ExtensionContext;
    private _env: any;

    public setContext(context: vscode.ExtensionContext, env: any) {
        this._context = context;
        this._env = env;
    }

    public getPackageNames(): Promise<string[]> {
        return new Promise((resolve, reject) => child_process.exec("ros2 pkg list", { env: this._env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    return line;
                }));
                resolve(lines);
            } else {
                reject(err);
            }
        }));
    }

    public getPackages(): Promise<{ [name: string]: () => string }> {
        return new Promise((resolve, reject) => child_process.exec("ros2 pkg list", { env: this._env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    return line;
                }));

                const packageInfoReducer = (acc: object, cur: string) => {
                    const k: string = cur;
                    acc[k] = async () => {
                        const { stdout } = await child_process.exec(`ros2 pkg prefix --share ${k}`, { env: this._env });
                        for await (const line of stdout) {
                            return line.trim();
                        }
                        return "";
                    };
                    return acc;
                };
                resolve(lines.reduce(packageInfoReducer, {}));
            } else {
                reject(err);
            }
        }));
    }

    public getIncludeDirs(): Promise<string[]> {
        return new Promise((resolve, reject) => {
            reject("not yet implemented");
        });
    }

    public findPackageExecutables(packageName: string): Promise<string[]> {
        return new Promise((resolve, reject) => child_process.exec(`ros2 pkg executables ${packageName}`, { env: this._env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    const info: string[] = line.split(" ");
                    if (info.length === 2) {
                        // each line should contain exactly 2 strings separated by 1 space
                        return info;
                    }
                }));
    
                const packageInfoReducer = (acc: string[], cur: string[]) => {
                    const executableName: string = cur[1] as string;
                    acc.push(executableName);
                    return acc;
                };
                resolve(lines.reduce(packageInfoReducer, []));
            } else {
                reject(err);
            }
        }));
    }

    public async findPackageLaunchFiles(packageName: string): Promise<string[]> {
        let packages = await this.getPackages();
        let packageBasePath = await packages[packageName]();
        let command:string;
        if (process.platform === "win32") {
            command = `where /r "${packageBasePath}" *.launch.py`;
        }
        else {
            command = `find $(${packageBasePath}) -type f -name *.launch.py`;
        }

        return new Promise((c, e) => child_process.exec(command, { env: this._env }, (err, out) => {
            err ? e(err) : c(out.trim().split(os.EOL));
        }));
    }

    public startCore() {
        // not yet implemented.
        return;
    }

    public stopCore() {
        // not yet implemented.
        return;
    }

    public activateCoreMonitor(): vscode.Disposable {
        // not yet implemented.
        return null;
    }

    public showCoreMonitor() {
        // not yet implemented.
        return;
    }

    public activateRosrun(packageName: string, executableName:string, argument: string): vscode.Terminal {
        let terminal = vscode.window.createTerminal({
            env: this._env,
            name: "ros2 run",
        });
        terminal.sendText(`ros2 run ${packageName} ${executableName} ${argument}`);
        return terminal;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
        let terminal = vscode.window.createTerminal({
            env: this._env,
            name: "ros2 launch",
        });
        terminal.sendText(`ros2 launch ${launchFilepath} ${argument}`);
        return terminal;
    }
}
