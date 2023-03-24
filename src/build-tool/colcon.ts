// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as path from "path";
import * as child_process from "child_process";
import * as extension from "../extension";
import * as common from "./common";
import * as rosShell from "./ros-shell";

function makeColcon(command: string, verb: string, args: string[], category?: string): vscode.Task {
    let installType = '--symlink-install';
    if (process.platform === "win32") {

        // Use Merge Install on Windows to support adminless builds and deployment.
        installType = '--merge-install';
    }

        const task = rosShell.make({type: command, command, args: [verb, installType, '--event-handlers', 'console_cohesion+', '--base-paths', extension.baseDir, `--cmake-args`, `-DCMAKE_BUILD_TYPE=RelWithDebInfo`,...args]},
                               category)
    task.problemMatchers = ["$catkin-gcc"];

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
    let colconCommand: string;
    const srcDir = path.join(dir, "src")

    if (process.platform === "win32") {
        colconCommand = `colcon --log-base nul list --base-paths \"${srcDir}\"`;
    } else {
        colconCommand = `colcon --log-base /dev/null list --base-paths ${srcDir}`;
    }

    const { stdout, stderr } = await child_process.exec(colconCommand);

    // Does this workspace have packages?
    for await (const line of stdout) {
        // Yes.
        return true;
    }

    // no.
    return false;
}
