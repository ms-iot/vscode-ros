// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import * as util from "util";
import * as vscode from "vscode";

import * as ros from "../ros";
import * as daemon from "./daemon";
import * as ros2_monitor from "./ros2-monitor";
import * as ros_utils from "../utils";

const promisifiedExists = util.promisify(fs.exists);
const promisifiedExec = util.promisify(child_process.exec);

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

    public async getIncludeDirs(): Promise<string[]> {
        const prefixPaths: string[] = [];
        if (this.env.AMENT_PREFIX_PATH) {
            prefixPaths.push(...this.env.AMENT_PREFIX_PATH.split(path.delimiter));
        }

        const includeDirs: string[] = [];
        for (const dir of prefixPaths) {
            const include = path.join(dir, "include");
            if (await promisifiedExists(include)) {
                includeDirs.push(include);
            }
        }

        return includeDirs;
    }

    public async getWorkspaceIncludeDirs(workspaceDir: string): Promise<string[]> {
        const includes: string[] = [];
        const opts = {
            env: this.env,
            cwd: workspaceDir
        };
        const result = await promisifiedExec(`colcon list -p --base-paths "${workspaceDir}"`, opts);

        // error out if we see anything from stderr.
        if (result.stderr) {
            return includes;
        }

        // each line should be a path like `c:\ros2_ws\src\demos\demo_nodes_cpp`
        for (const line of result.stdout.split(os.EOL)) {
            const include = path.join(line, "include");

            if (await promisifiedExists(include)) {
                includes.push(include);
            }
        }

        return includes;
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
            `where /r "${packageBasePath}" *launch.py` :
            `find -L "${packageBasePath}" -type f -name *launch.py`;

        return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
            err ? e(new Error('No launch files are found.')) : c(out.trim().split(os.EOL));
        }));
    }

    public async findPackageTestFiles(packageName: string): Promise<string[]> {
        // TODO: ROS2 rostest equivalent not implemented yet
        return new Promise((resolve, reject) => {
            resolve([]);
        });
    }

    public async startCore() {
        daemon.startDaemon();
    }

    public async stopCore() {
        daemon.stopDaemon();
    }

    public getCoreStatus(): Promise<boolean> {
        // TODO(#431): Core status checking not implemented for ROS2
        return;
    }

    public rosdep(): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
      terminal.sendText(`rosdep install --from-paths src --ignore-src -r -y`);
      return terminal;
  }

    public activateCoreMonitor(): vscode.Disposable {
        const coreStatusItem = new daemon.StatusBarItem();
        coreStatusItem.activate();
        return coreStatusItem;
    }

    public async showCoreMonitor() {
        return ros2_monitor.launchMonitor(this.context);
    }

    public activateRosrun(packageName: string, executableName: string, argument: string): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
        terminal.sendText(`ros2 run ${packageName} ${executableName} ${argument}`);
        return terminal;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
        terminal.sendText(`ros2 launch ${launchFilepath} ${argument}`);
        return terminal;
    }

    public activateRostest(launchFilepath: string, argument: string): vscode.Terminal {
        console.error("ROS2 rostest equivalent not implemented yet");
        return;
    }
}
