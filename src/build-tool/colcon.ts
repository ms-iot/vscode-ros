// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode"
import * as child_process from "child_process";

import * as extension from "../extension"
import * as common from "./common"
import { appendFileSync } from "fs";

/**
 * Provides colcon build and test tasks.
 */
export class ColconProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        let buildCommand: string;
        let testCommand: string;

        buildCommand = `colcon build`;
        testCommand = `colcon test`;

        const make = new vscode.Task({ type: "colcon" }, "build", "colcon");
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env
        });
        make.group = vscode.TaskGroup.Build;

        const test = new vscode.Task({ type: "colcon" }, "test", "colcon");
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env
        });
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return undefined;
    }
}

export async function isApplicable(dir: string): Promise<boolean> {
    const opts = { dir, env: extension.env };
    const { stdout, stderr } = await child_process.exec('colcon -h', opts);
    for await (const line of stderr)
    {
        return false;
    }
    return true;
}
