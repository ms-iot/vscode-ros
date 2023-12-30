// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import * as common from "./common";
import * as rosShell from "./ros-shell";

function makeCatkin(name: string, command: string, args: string[], category?: string): vscode.Task {
    const task = rosShell.make(name, {type: command, command, args: ['--workspace', vscode.workspace.rootPath, ...args]}, category)
    task.problemMatchers = ["$catkin-gcc"];

    return task;
}

/**
 * Provides catkin tools build and test tasks.
 */
export class CatkinToolsProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        const make = makeCatkin('Catkin Tools - Build', 'catkin', [], 'build');
        make.group = vscode.TaskGroup.Build;

        const test = makeCatkin('Catkin Tools - Test', 'catkin', ['--catkin-make-args', 'run_tests'], 'run_tests');
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return rosShell.resolve(task);
    }
}

/**
 * Interacts with the user to run a `catkin create pkg` command.
 */
export async function createPackage(uri?: vscode.Uri) {
    const createPkgCommand = (dependencies: string, name: string): string => {
        return `catkin create pkg --catkin-deps ${dependencies} -- ${name}`;
    };
    return common._createPackage(createPkgCommand);
}
