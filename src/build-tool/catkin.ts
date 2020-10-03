// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import * as common from "./common";
import * as rosShell from "./ros-shell";

function makeCatkin(command: string, args: string[], category?: string): vscode.Task {
    const task = rosShell.make({type: command, command, args: ['--directory', extension.baseDir, ...args]}, category)
    task.problemMatchers = ["$catkin-gcc"];

    return task;
}
/**
 * Provides catkin_make build and test tasks
 */
export class CatkinMakeProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        const make = makeCatkin('catkin_make', [], 'build');
        make.group = vscode.TaskGroup.Build;

        const test = makeCatkin('catkin_make', ['run_tests'], 'run_tests');
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return rosShell.resolve(task);
    }
}
/**
 * Provides catkin_make_isolated build and test tasks
 */
export class CatkinMakeIsolatedProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        const make = makeCatkin('catkin_make_isolated', [], 'build');
        make.group = vscode.TaskGroup.Build;

        const test = makeCatkin('catkin_make_isolated', ['--catkin-make-args', 'run_tests'], 'run_tests');
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return rosShell.resolve(task);
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
