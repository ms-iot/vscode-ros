// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import * as utils from "./utils";

export async function rosrun(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.Rosrun);

    const terminal = await preparerosrun();
    terminal.show();
}

async function preparerosrun(): Promise<vscode.Terminal> {
    const packages = utils.getPackages();
    const packageName = await vscode.window.showQuickPick(packages.then(Object.keys), {
        placeHolder: "Choose a package",
    });
    if (packageName !== undefined) {
        let basenames = (files: string[]) => files.map((file) => path.basename(file));

        const executables = utils.findPackageExecutables(packageName).then(basenames);
        let target = await vscode.window.showQuickPick(executables, { placeHolder: "Choose an executable" });
        let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        let terminal = vscode.window.createTerminal({
            env: extension.env,
            name: "rosrun",
        });
        terminal.sendText(`rosrun ${packageName} ${target} ${argument}`);
        return terminal;
    } else {
        // none of the packages selected, error!
    }
}

export async function roslaunch(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.Roslaunch);

    let terminal = await prepareroslaunch();
    terminal.show();
}

async function prepareroslaunch(): Promise<vscode.Terminal> {
    const packages = utils.getPackages();
    const packageName = await vscode.window.showQuickPick(packages.then(Object.keys), {
        placeHolder: "Choose a package",
    });
    if (packageName !== undefined) {
        const launchFiles = await utils.findPackageLaunchFiles(packageName);
        const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
        let target = await vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
        let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        let terminal = vscode.window.createTerminal({
            env: extension.env,
            name: "roslaunch",
        });
        terminal.sendText(`roslaunch ${launchFiles[launchFileBasenames.indexOf(target)]} ${argument}`);
        return terminal;
    } else {
        // none of the packages selected, error!
    }
}
