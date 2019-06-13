// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as utils from "../../../ros/utils";

/**
 * Interacts with the user to create a `roslaunch` or `rosrun` configuration.
 */
export class RosDebugConfigurationProvider implements vscode.DebugConfigurationProvider {
    public provideDebugConfigurations(folder: vscode.WorkspaceFolder | undefined, token?: vscode.CancellationToken) {
        return [];
    }

    public async resolveDebugConfiguration(
        folder: vscode.WorkspaceFolder | undefined,
        config: vscode.DebugConfiguration,
        token?: vscode.CancellationToken) {
        const packages = utils.getPackages();

        const command = await vscode.window.showQuickPick(["roslaunch", "rosrun"], { placeHolder: "Launch command" });
        const packageName = await vscode.window.showQuickPick(packages.then(Object.keys), { placeHolder: "Package" });

        let target: string;

        if (packageName) {
            let basenames = (files: string[]) => files.map(file => path.basename(file));

            if (command === "roslaunch") {
                const launches = utils.findPackageLaunchFiles(packageName).then(basenames);
                target = await vscode.window.showQuickPick(launches, { placeHolder: "Launch file" });
            } else {
                const executables = utils.findPackageExecutables(packageName).then(basenames);
                target = await vscode.window.showQuickPick(executables, { placeHolder: "Executable" });
            }
        } else {
            target = await vscode.window.showInputBox({ placeHolder: "Target" });
        }

        config.type = "ros";
        config.request = "launch";
        config.command = command;
        config.package = packageName;
        config.target = target;
        config.args = [];
        config.debugSettings = "${command:debugSettings}";

        return config;
    }
}
