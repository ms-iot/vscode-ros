// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import { rosApi } from "./ros"

export async function rosrun(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.Rosrun);

    const terminal = await preparerosrun();
    if (!terminal) {
        return;
    }
    terminal.show();
}

async function preparerosrun(): Promise<vscode.Terminal> {
    const packageName = await vscode.window.showQuickPick(rosApi.getPackageNames(), {
        placeHolder: "Choose a package",
    });
    if (!packageName) {
        return;
    }
    let basenames = (files: string[]) => files.map((file) => path.basename(file));
    const executables = rosApi.findPackageExecutables(packageName).then(basenames);
    let target = await vscode.window.showQuickPick(executables, { placeHolder: "Choose an executable" });
    if (!target) {
        return;
    }
    let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
    if (argument == undefined) {
        return;
    }
    return rosApi.activateRosrun(packageName, target, argument);
}

export async function roslaunch(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.Roslaunch);

    let terminal = await prepareroslaunch();
    if (!terminal) {
        return;
    }
    terminal.show();
}

async function prepareroslaunch(): Promise<vscode.Terminal> {
    const packageName = await vscode.window.showQuickPick(rosApi.getPackageNames(), {
        placeHolder: "Choose a package",
    });
    if (!packageName) {
        return;
    }
    const launchFiles = await rosApi.findPackageLaunchFiles(packageName);
    const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
    let target = await vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
    const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
    if (!launchFilePath) {
        return;
    }
    let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
    if (argument == undefined) {
        return;
    }
    return rosApi.activateRoslaunch(launchFilePath, argument);
}

export async function rostest(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.Rostest);

    let terminal = await preparerostest();
    if (!terminal) {
        return;
    }
    terminal.show();
}

async function preparerostest(): Promise<vscode.Terminal> {
    const packageName = await vscode.window.showQuickPick(rosApi.getPackageNames(), {
        placeHolder: "Choose a package",
    });
    if (!packageName) {
        return;
    }
    const launchFiles = (await rosApi.findPackageLaunchFiles(packageName)).concat(await rosApi.findPackageTestFiles(packageName));
    const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
    let target = await vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
    const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
    if (!launchFilePath) {
        return;
    }
    let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
    if (argument == undefined) {
        return;
    }
    return rosApi.activateRostest(launchFilePath, argument);
}
