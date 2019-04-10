import * as child_process from "child_process";
import * as _ from "underscore";
import * as vscode from "vscode";
import * as path from "path";
import * as os from "os";

import * as extension from "./extension";
import * as pfs from "./promise-fs";

/**
 * Gets the ROS config section.
 */
export function getConfig(): vscode.WorkspaceConfiguration {
    return vscode.workspace.getConfiguration("ros");
}

/**
 * Executes a setup file and returns the resulting env.
 */
export function sourceSetupFile(filename: string, env?: any): Promise<any> {
    return new Promise((resolve, reject) => {
        let exportEnvCommand: string;
        if (process.platform === "win32") {
            exportEnvCommand = `cmd /c "\"${filename}\" && set"`;
        }
        else {
            exportEnvCommand = `bash -c "source '${filename}' && env"`;
        }

        let processOptions = {
            cwd: extension.baseDir,
            env: env,
            shell: "cmd",
            windowsHide: false,
        };
        child_process.exec(exportEnvCommand, processOptions, (error, stdout, _stderr) => {
            if (!error) {
                resolve(stdout.split(os.EOL).reduce((env, line) => {
                    const index = line.indexOf("=");

                    if (index !== -1) {
                        env[line.substr(0, index)] = line.substr(index + 1);
                    }

                    return env;
                }, {}));
            } else {
                reject(error);
            }
        });
    });
}

/**
 * Gets the names of installed distros.
 */
export function getDistros(): Promise<string[]> {
    return pfs.readdir("/opt/ros");
}

/**
 * Gets a map of package names to paths.
 */
export function getPackages(): Promise<{ [name: string]: string }> {
    return new Promise((resolve, reject) => child_process.exec("rospack list", { env: extension.env }, (err, out) => {
        if (!err) {
            resolve(_.object(out.trim().split(os.EOL).map(line => line.split(" ", 2))));
        } else {
            reject(err);
        }
    }));
}

/**
 * Gets include dirs using `catkin_find`.
 */
export function getIncludeDirs(): Promise<string[]> {
    return new Promise((c, e) => child_process.exec("catkin_find --include", { env: extension.env }, (err, out) =>
        err ? e(err) : c(out.trim().split("\n"))
    ));
}

/**
 * list full paths to all executables inside a package
 */
export function findPackageExecutables(packageName: string): Promise<string[]> {
    let command: string;
    if (process.platform === "win32") {
        const commandFindPackagePaths = `catkin_find --without-underlays --libexec ${packageName}`;
        let paths = child_process.execSync(commandFindPackagePaths, { env: extension.env }).toString();
        let normalizedPath = path.win32.normalize(paths.trim().split(os.EOL)[0]);
        command = `where /r "${normalizedPath}" *.exe`;
    }
    else {
        const dirs = `catkin_find --without-underlays --libexec --share '${packageName}'`;
        command = `find $(${dirs}) -type f -executable`;
    }

    return new Promise((c, e) => child_process.exec(command, { env: extension.env }, (err, out) =>
        err ? e(err) : c(out.trim().split(os.EOL))
    ));
}

/**
 * list all .launch files inside a package
 */
export function findPackageLaunchFiles(packageName: string): Promise<string[]> {
    let command: string;
    if (process.platform === "win32") {
        const commandFindPackagePaths = `catkin_find --without-underlays --share ${packageName}`;
        let paths = child_process.execSync(commandFindPackagePaths, { env: extension.env }).toString();
        let normalizedPath = path.win32.normalize(paths.trim().split(os.EOL)[0]);
        command = `where /r "${normalizedPath}" *.launch`;
    }
    else {
        const dirs = `catkin_find --without-underlays --share '${packageName}'`;
        command = `find $(${dirs}) -type f -name *.launch`;
    }

    return new Promise((c, e) => child_process.exec(command, { env: extension.env }, (err, out) => {
        err ? e(err) : c(out.trim().split(os.EOL));
    }));
}

/**
 * Creates and shows a ROS-sourced terminal.
 */
export function createTerminal() {
    vscode.window.createTerminal({ name: 'ROS', env: extension.env }).show();
}
