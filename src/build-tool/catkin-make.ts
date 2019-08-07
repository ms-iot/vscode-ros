// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import * as common from "./common";

/**
 * Provides catkin_make build and test tasks.
 */
export class CatkinMakeProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        let buildCommand: string;
        let testCommand: string;

        buildCommand = `catkin_make --directory "${extension.baseDir}"`;
        testCommand = `${buildCommand} run_tests`;

        const make = new vscode.Task({ type: "catkin" }, "make", "catkin");
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];

        const test = new vscode.Task({ type: "catkin", target: "run_tests" }, "run_tests", "catkin");
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env,
        });
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return undefined;
    }
}

/**
 * Interacts with the user to run a `catkin_create_pkg` command.
 */
export async function createPackage(uri?: vscode.Uri) {
    const createPkgCommand = (dependencies: string, name: string): string => {
        return `catkin_create_pkg ${name} ${dependencies}`;
    };
    return common._createPackage(createPkgCommand);
}
