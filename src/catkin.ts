// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as cp from "child_process";
import * as vscode from "vscode";

import * as extension from "./extension";
import * as telemetry from "./telemetry-helper";

/**
 * Interacts with the user to run a `catkin_create_pkg` command.
 */
export async function createPackage(context: vscode.ExtensionContext, uri?: vscode.Uri) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.CreateCatkinPackage);

    const name = await vscode.window.showInputBox({
        prompt: "Package name",
        validateInput: val => val.match(/^\w+$/) ? "" : "Invalid name",
    });

    if (!name) {
        return;
    }

    const dependencies = await vscode.window.showInputBox({
        prompt: "Dependencies",
        validateInput: val => val.match(/^\s*(\w+\s*)*$/) ? "" : "Invalid dependencies",
    });

    if (typeof dependencies === "undefined") {
        return;
    }

    const cwd = typeof uri !== "undefined" ? uri.fsPath : `${extension.baseDir}/src`;
    const opts = { cwd, env: extension.env };

    let createPkgCommand: string;

    if (extension.buildSystem === extension.BuildSystem.CatkinMake) {
        createPkgCommand = `catkin_create_pkg ${name} ${dependencies}`;
    } else if (extension.buildSystem === extension.BuildSystem.CatkinTools) {
        createPkgCommand = `catkin create pkg --catkin-deps ${dependencies} -- ${name}`;
    }

    cp.exec(createPkgCommand, opts, (err, stdout, stderr) => {
        if (!err) {
            vscode.workspace.openTextDocument(`${cwd}/${name}/package.xml`).then(vscode.window.showTextDocument);
        } else {
            let message = "Could not create package";
            let index = stderr.indexOf("error:");

            if (index !== -1) {
                message += ": " + stderr.substr(index);
            }

            vscode.window.showErrorMessage(message);
        }
    });
}
