// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as os from "os";
import * as path from "path";
import * as vscode from "vscode";

import * as ros from "../ros";
import * as ros_core from "./core-helper";

export class ROS1 implements ros.ROSApi {
    private _context: vscode.ExtensionContext;
    private _env: any;
    private _xmlRpcApi: ros_core.XmlRpcApi = null;

    public setContext(context: vscode.ExtensionContext, env: any) {
        this._context = context;
        this._env = env;
    }

    public getPackageNames(): Promise<string[]> {
        return this.getPackages().then((packages: { [name: string]: () => string }) => {
            return Object.keys(packages);
        });
    }

    public getPackages(): Promise<{ [name: string]: () => string }> {
        return new Promise((resolve, reject) => child_process.exec("rospack list", { env: this._env }, (err, out) => {
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
                        return `${v}`;
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
        if (this._env.hasOwnProperty("CMAKE_PREFIX_PATH")) {
            cmakePrefixPaths.push(...this._env.CMAKE_PREFIX_PATH.split(path.delimiter));
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

    public findPackageExecutables(packageName: string): Promise<string[]> {
        let command: string;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--libexec`, `*.exe`);
        } else {
            const dirs = `catkin_find --without-underlays --libexec --share '${packageName}'`;
            command = `find $(${dirs}) -type f -executable`;
            return new Promise((c, e) => child_process.exec(command, { env: this._env }, (err, out) =>
                err ? e(err) : c(out.trim().split(os.EOL))
            ));
        }
    }

    public findPackageLaunchFiles(packageName: string): Promise<string[]> {
        let command: string;
        if (process.platform === "win32") {
            return this._findPackageFiles(packageName, `--share`, `*.launch`);
        }
        else {
            const dirs = `catkin_find --without-underlays --share '${packageName}'`;
            command = `find $(${dirs}) -type f -name *.launch`;
        }

        return new Promise((c, e) => child_process.exec(command, { env: this._env }, (err, out) => {
            err ? e(err) : c(out.trim().split(os.EOL));
        }));
    }

    public startCore() {
        if (typeof this._env.ROS_MASTER_URI === "undefined") {
            return; 
        }
        ros_core.startCore(this._context);
    }

    public stopCore() {
        if (typeof this._env.ROS_MASTER_URI === "undefined") {
            return; 
        }
        ros_core.stopCore(this._context, this._getXmlRpcApi());
    }

    public activateCoreMonitor(): vscode.Disposable {
        if (typeof this._env.ROS_MASTER_URI === "undefined") {
            return null; 
        }
        const coreStatusItem = new ros_core.StatusBarItem(this._getXmlRpcApi());
        coreStatusItem.activate();
        return coreStatusItem;
    }

    public showCoreMonitor() {
        ros_core.launchMonitor(this._context);
    }

    public activateRosrun(packageName: string, executableName:string, argument: string): vscode.Terminal {
        let terminal = vscode.window.createTerminal({
            env: this._env,
            name: "rosrun",
        });
        terminal.sendText(`rosrun ${packageName} ${executableName} ${argument}`);
        return terminal;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
        let terminal = vscode.window.createTerminal({
            env: this._env,
            name: "roslaunch",
        });
        terminal.sendText(`roslaunch ${launchFilepath} ${argument}`);
        return terminal;
    }

    private _findPackageFiles(packageName: string, filter: string, pattern: string): Promise<string[]> {
        return new Promise((c, _e) => child_process.exec(`catkin_find --without-underlays ${filter} ${packageName}`,
            { env: this._env }, (_err, out) => {
                let findFilePromises = [];
                let paths = out.trim().split(os.EOL);
                paths.forEach(foundPath => {
                    let normalizedPath = path.win32.normalize(foundPath);
                    findFilePromises.push(new Promise((found) => child_process.exec(`where /r "${normalizedPath}" ` + pattern,
                        { env: this._env }, (err, out) =>
                            err ? found(null) : found(out.trim().split(os.EOL))
                    )));
                });

                return Promise.all(findFilePromises).then(values => {
                    // remove null elements
                    values = values.filter(s => s != null) as string[];

                    // flatten
                    values = [].concat(...values);
                    c(values);
                });
            }
        ));
    }

    private _getXmlRpcApi(): ros_core.XmlRpcApi {
        if (this._xmlRpcApi === null) {
            this._xmlRpcApi = new ros_core.XmlRpcApi(this._env.ROS_MASTER_URI);
        }
        return this._xmlRpcApi;
    }
}
