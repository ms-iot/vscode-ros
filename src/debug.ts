// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as extension from "./extension";
import * as utils from "./utils";
import { basename } from "path";
import {
    CancellationToken,
    DebugConfiguration,
    DebugConfigurationProvider,
    window,
    ProviderResult,
    WorkspaceFolder,
} from "vscode";

/**
 * Gets stringified settings to pass to the debug server.
 */
export async function getDebugSettings() {
    return JSON.stringify({ env: extension.env });
}

/**
 * Interacts with the user to create a `roslaunch` or `rosrun` configuration.
 */
export class RosDebugConfigProvider implements DebugConfigurationProvider {
    provideDebugConfigurations(folder: WorkspaceFolder | undefined, token?: CancellationToken) {
        return [];
    }

    async resolveDebugConfiguration(folder: WorkspaceFolder | undefined, config: DebugConfiguration, token?: CancellationToken) {
        const packages = utils.getPackages();

        const command = await window.showQuickPick(["roslaunch", "rosrun"], { placeHolder: "Launch command" });
        const packageName = await window.showQuickPick(packages.then(Object.keys), { placeHolder: "Package" });

        let target: string;

        if (packageName) {
            let basenames = (files: string[]) => files.map(file => basename(file));

            if (command === "roslaunch") {
                const launches = utils.findPackageLaunchFiles(packageName).then(basenames);
                target = await window.showQuickPick(launches, { placeHolder: "Launch file" });
            } else {
                const executables = utils.findPackageExecutables(packageName).then(basenames);
                target = await window.showQuickPick(executables, { placeHolder: "Executable" });
            }
        } else {
            target = await window.showInputBox({ placeHolder: "Target" });
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
