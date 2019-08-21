// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import * as common from "./common";

/**
 * Provides catkin build and test tasks
 * including catkin_make and catkin_make_isolated
 */
export class CatkinProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        const tasksCatkinMake = this.provideCatkinMakeTasks();
        const tasksCatkinMakeIsolated = this.provideCatkinMakeIsolatedTasks();
        return [...tasksCatkinMake, ...tasksCatkinMakeIsolated];
    }

    public resolveTask(task: vscode.Task, token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task> {
        return undefined;
    }

    private provideCatkinMakeTasks(): vscode.Task[] {
        const catkinMakeDefinition: vscode.TaskDefinition = {
            type: "catkin_make",
            target: "build",
        };
        const make = new vscode.Task(catkinMakeDefinition, "build", "catkin_make");
        const buildCommand = `catkin_make --directory "${extension.baseDir}"`;
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];

        const catkinMakeRunTestsDefinition: vscode.TaskDefinition = {
            type: "catkin_make",
            target: "run_tests",
        };
        const test = new vscode.Task(catkinMakeRunTestsDefinition, "run_tests", "catkin_make");
        const testCommand = `${buildCommand} run_tests`;
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env,
        });
        test.group = vscode.TaskGroup.Test;

        return [make, test];
    }

    private provideCatkinMakeIsolatedTasks(): vscode.Task[] {
        const catkinMakeIsolatedDefinition: vscode.TaskDefinition = {
            type: "catkin_make_isolated",
            target: "build",
        };
        const make = new vscode.Task(catkinMakeIsolatedDefinition, "build", "catkin_make_isolated");
        const buildCommand = `catkin_make_isolated --directory "${extension.baseDir}"`;
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env,
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];

        const catkinMakeIsolatedRunTestsDefinition: vscode.TaskDefinition = {
            type: "catkin_make_isolated",
            target: "run_tests",
        };
        const test = new vscode.Task(catkinMakeIsolatedRunTestsDefinition, "run_tests", "catkin_make_isolated");
        const testCommand = `${buildCommand} --catkin-make-args run_tests`;
        test.execution = new vscode.ShellExecution(testCommand, {
            env: extension.env,
        });
        test.group = vscode.TaskGroup.Test;

        return [make, test];
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
