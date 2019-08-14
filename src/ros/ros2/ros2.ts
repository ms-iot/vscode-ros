// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as vscode from "vscode";

import * as ros from "../ros";

export class ROS2 implements ros.ROSApi {
    private context: vscode.ExtensionContext;
    private env: any;

    public setContext(context: vscode.ExtensionContext, env: any) {
        this.context = context;
        this.env = env;
    }

    public getPackageNames(): Promise<string[]> {
        return new Promise((resolve, reject) => child_process.exec("ros2 pkg list", { env: this.env }, (err, out) => {
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

    public async getPackages(): Promise<{ [name: string]: () => Promise<string> }> {
        const packages: { [name: string]: () => Promise<string> } = {};
        const { stdout } = child_process.exec("ros2 pkg list", { env: this.env });
        let chucks = "";
        for await (const chuck of stdout) {
            chucks += chuck;
        }

        chucks.split(os.EOL).map(((line) => {
            const packageName: string = line.trim();
            packages[packageName] = async (): Promise<string> => {
                const { stdout } = await child_process.exec(
                    `ros2 pkg prefix --share ${packageName}`, { env: this.env });
                let innerChucks = "";
                for await (const chuck of stdout) {
                    innerChucks += chuck;
                }
                return innerChucks.trim();
            };
        }));

        return packages;
    }

    public getIncludeDirs(): Promise<string[]> {
        return new Promise((resolve, reject) => {
            reject("not yet implemented");
        });
    }

    public findPackageExecutables(packageName: string): Promise<string[]> {
        return new Promise((resolve, reject) => child_process.exec(
            `ros2 pkg executables ${packageName}`, { env: this.env }, (err, out) => {
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
        const packages = await this.getPackages();
        const packageBasePath = await packages[packageName]();
        const command: string = (process.platform === "win32") ?
            `where /r "${packageBasePath}" *.launch.py` :
            `find $(${packageBasePath}) -type f -name *.launch.py`;

        return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
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

    public activateRosrun(packageName: string, executableName: string, argument: string): vscode.Terminal {
        const terminal = vscode.window.createTerminal({
            env: this.env,
            name: "ros2 run",
        });
        terminal.sendText(`ros2 run ${packageName} ${executableName} ${argument}`);
        return terminal;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
        const terminal = vscode.window.createTerminal({
            env: this.env,
            name: "ros2 launch",
        });
        terminal.sendText(`ros2 launch ${launchFilepath} ${argument}`);
        return terminal;
    }
}
