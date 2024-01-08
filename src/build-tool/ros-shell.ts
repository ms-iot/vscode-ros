// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";

export interface RosTaskDefinition extends vscode.TaskDefinition {
    command: string;
    args?: string[];
}

export class RosShellTaskProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        return this.defaultRosTasks();
    }

    public defaultRosTasks(): vscode.Task[] {
        const rosCore = make('ros core', {type: 'ros', command: 'roscore'}, 'roscore');
        rosCore.isBackground = true;
        rosCore.problemMatchers = ['$roscore'];

        const rosLaunch = make('ros launch', {type: 'ros', command: 'roslaunch', args: ['package_name', 'launch_file.launch']}, 'roslaunch');
        rosLaunch.isBackground = true;
        rosLaunch.problemMatchers = ['$roslaunch'];

        return [rosCore, rosLaunch];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return resolve(task);
    }
}

export function registerRosShellTaskProvider(): vscode.Disposable[] {
    return [
        vscode.tasks.registerTaskProvider('ros', new RosShellTaskProvider()),
    ];
}

export function resolve(task: vscode.Task): vscode.Task {
    let definition = task.definition as RosTaskDefinition
    const resolvedTask = make(definition.command, definition);

    resolvedTask.isBackground = task.isBackground;
    resolvedTask.problemMatchers = task.problemMatchers;
    return resolvedTask;
}

export function make(name: string, definition: RosTaskDefinition, category?: string): vscode.Task {
    definition.command = definition.command || definition.type; // Command can be missing in build tasks that have type==command

    const args = definition.args || [];
    const task = new vscode.Task(definition, vscode.TaskScope.Workspace, name, definition.command);

    task.execution = new vscode.ShellExecution(definition.command, args, {
        env: extension.env,
    });
    return task;
}
