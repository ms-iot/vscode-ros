// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as child_process from "child_process";
import * as extension from "../extension";
import * as common from "./common";
import * as rosShell from "./ros-shell";

function makeColcon(command: string, verb: string, args: string[], category?: string): vscode.Task {
    const task = rosShell.make({type: command, command, args: [verb, '--base-paths', extension.baseDir, ...args]},
                               category)

    return task;
}

/**
 * Provides colcon build and test tasks.
 */
export class ColconProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        const make = makeColcon('colcon', 'build', [], 'build');
        make.group = vscode.TaskGroup.Build;

        const test = makeColcon('colcon', 'test', [], 'test');
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return rosShell.resolve(task);
    }
}

export async function isApplicable(dir: string): Promise<boolean> {
    const opts = { dir, env: extension.env };
    const { stdout, stderr } = await child_process.exec("colcon -h", opts);
    for await (const line of stderr) {
        return false;
    }
    return true;
}
