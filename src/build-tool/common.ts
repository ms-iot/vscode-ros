// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode"
import * as child_process from "child_process";

import * as extension from "../extension"

/**
 * Interacts with the user to run a create package command.
 */
export async function _createPackage(createPkgCommand: (dependencies: string, name:string) => string, uri?: vscode.Uri) {
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

    const cwd = typeof uri !== "undefined" ? uri.fsPath : `${vscode.workspace.rootPath}/src`;
    const opts = { cwd, env: extension.env };

    child_process.exec(createPkgCommand(dependencies, name), opts, (err, stdout, stderr) => {
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
