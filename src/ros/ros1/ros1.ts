// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import * as util from "util";
import * as vscode from "vscode";

import * as ros from "../ros";
import * as ros_core from "./core-helper";
import * as ros_utils from "../utils";

const promisifiedExists = util.promisify(fs.exists);

export class ROS1 implements ros.ROSApi {
    private context: vscode.ExtensionContext;
    private env: any;
    private xmlRpcApi: ros_core.XmlRpcApi = null;

    public setContext(context: vscode.ExtensionContext, env: any) {
        this.context = context;
        this.env = env;
    }

    public getPackageNames(): Promise<string[]> {
        return this.getPackages().then((packages: { [name: string]: () => Promise<string> }) => {
            return Object.keys(packages);
        });
    }

    public getPackages(): Promise<{ [name: string]: () => Promise<string> }> {
        return new Promise((resolve, reject) => child_process.exec("rospack list", { env: this.env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    const info: string[] = line.split(" ");
                    if (info.length === 2) {
                        // each line should contain exactly 2 strings separated by 1 space
                        return info;
                    }
                }));

                const packageInfoReducer = (acc: object, cur: string[]) => {
                    const k: string = cur[0] as string;
                    const v: string = cur[1] as string;
                    acc[k] = async () => {
                        return v;
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
        const cmakePrefixPaths: string[] = [];
        if (this.env.hasOwnProperty("CMAKE_PREFIX_PATH")) {
            cmakePrefixPaths.push(...this.env.CMAKE_PREFIX_PATH.split(path.delimiter));
        }

        const includeDirs: string[] = [];
        const fsPromises = cmakePrefixPaths.map((dir: string) => {
            const include = path.join(dir, "include");
            return fs.promises.access(include, fs.constants.F_OK)
                .then(() => {
                    includeDirs.push(include);
                })
                .catch(() => {
                    // suppress exception if include folder does not exist
                });
        });
        return Promise.all(fsPromises).then(() => {
            return includeDirs;
        });
    }

    public async getWorkspaceIncludeDirs(workspaceDir: string): Promise<string[]> {
        // Get all packages within the workspace that have an include directory
        const packages = await this.getPackages();
        const filteredPackages = await Object.values(packages).filter(async (packagePath: () => Promise<string>) => {
            const packageBasePath = await packagePath();
            return packageBasePath.startsWith(workspaceDir);
        });

        const includes: string[] = [];
        for (const pkg of filteredPackages) {
            const packageBasePath = await pkg();
            const include = path.join(packageBasePath, "include");

            if (await promisifiedExists(include)) {
                includes.push(include);
            }
        }

        return includes;
    }

    public findPackageExecutables(packageName: string): Promise<string[]> {
        let command: string;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--libexec`, `*.exe`);
        } else {
            const dirs = `catkin_find --without-underlays --libexec --share '${packageName}'`;
            command = `find -L $(${dirs}) -type f -executable`;
            return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) =>
                err ? e(err) : c(out.trim().split(os.EOL)),
            ));
        }
    }

    public findPackageLaunchFiles(packageName: string): Promise<string[]> {
        let command: string;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--share`, `*.launch`);
        } else {
            const dirs = `catkin_find --without-underlays --share '${packageName}'`;
            command = `find -L $(${dirs}) -type f -name *.launch`;
        }

        return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
            err ? e(err) : c(out.trim().split(os.EOL));
        }));
    }

    public findPackageTestFiles(packageName: string): Promise<string[]> {
        let command: string;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--share`, `*.test`);
        } else {
            const dirs = `catkin_find --without-underlays --share '${packageName}'`;
            command = `find -L $(${dirs}) -type f -name *.test`;
        }
        
        return new Promise((c, e) => child_process.exec(command, { env: this.env }, (err, out) => {
            err ? e(err) : c(out.trim().split(os.EOL));
        }));
    }

    public startCore() {
        if (typeof this.env.ROS_MASTER_URI === "undefined") {
            return;
        }
        ros_core.startCore(this.context);
    }

    public stopCore() {
        if (typeof this.env.ROS_MASTER_URI === "undefined") {
            return;
        }
        ros_core.stopCore(this.context, this._getXmlRpcApi());
    }

    public getCoreStatus(): Promise<boolean> {
        return this._getXmlRpcApi().check();
    }

    public activateCoreMonitor(): vscode.Disposable {
        if (typeof this.env.ROS_MASTER_URI === "undefined") {
            return null;
        }
        const coreStatusItem = new ros_core.StatusBarItem(this._getXmlRpcApi());
        coreStatusItem.activate();
        return coreStatusItem;
    }

    public rosdep(): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
      this.setTerminalEnv(terminal,this.env);
      terminal.sendText(`rosdep install --from-paths src --ignore-src -r -y`);
      return terminal;
  }


    public showCoreMonitor() {
        ros_core.launchMonitor(this.context);
    }

    public activateRosrun(packageName: string, executableName: string, argument: string): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal,this.env);
        terminal.sendText(`rosrun ${packageName} ${executableName} ${argument}`);
        return terminal;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal,this.env);
        terminal.sendText(`roslaunch ${launchFilepath} ${argument}`);
        return terminal;
    }

    public activateRostest(launchFilepath: string, argument: string): vscode.Terminal {
      const terminal = ros_utils.createTerminal(this.context);
        this.setTerminalEnv(terminal,this.env);
        terminal.sendText(`rostest ${launchFilepath} ${argument}`);
        return terminal;
    }

    public setTerminalEnv(terminal:vscode.Terminal,env: any) {
        if (process.platform === "linux"){
            for(var item in env){
                terminal.sendText(`export ${item}=${env[item]} >/dev/null`);
            }
        }
    }

    private _findPackageFiles(packageName: string, filter: string, pattern: string): Promise<string[]> {
        return new Promise((c, e) => child_process.exec(`catkin_find --without-underlays ${filter} ${packageName}`,
            { env: this.env }, (err, out) => {
                const findFilePromises = [];
                const paths = out.trim().split(os.EOL);
                paths.forEach((foundPath) => {
                    const normalizedPath = path.win32.normalize(foundPath);
                    findFilePromises.push(new Promise((found) => child_process.exec(
                        `where /r "${normalizedPath}" ` + pattern, { env: this.env }, (innerErr, innerOut) =>
                            innerErr ? found(null) : found(innerOut.trim().split(os.EOL)),
                    )));
                });

                return Promise.all(findFilePromises).then((values) => {
                    // remove null elements
                    values = values.filter((s) => s != null) as string[];

                    // flatten
                    values = [].concat(...values);
                    c(values);
                });
            },
        ));
    }

    private _getXmlRpcApi(): ros_core.XmlRpcApi {
        if (this.xmlRpcApi === null) {
            this.xmlRpcApi = new ros_core.XmlRpcApi(this.env.ROS_MASTER_URI);
        }
        return this.xmlRpcApi;
    }
}
