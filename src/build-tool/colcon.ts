// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as vscode from "vscode";

import * as extension from "../extension";

/**
 * Provides colcon build and test tasks.
 */
export class ColconProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        const buildCommand = "colcon build";
        const testCommand = "colcon test";

        const build = new vscode.Task(
            { type: "colcon" },
            vscode.TaskScope.Workspace,
            "build",
            "colcon",
            new vscode.ShellExecution(buildCommand, {
                env: extension.env,
            }),
            []);
        build.group = vscode.TaskGroup.Build;

        const test = new vscode.Task(
            { type: "colcon" },
            vscode.TaskScope.Workspace,
            "test",
            "colcon",
            new vscode.ShellExecution(testCommand, {
                env: extension.env,
            }),
            []);
        test.group = vscode.TaskGroup.Test;

        return [build, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return undefined;
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
