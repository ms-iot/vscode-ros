import * as extension from "./extension";
import * as vscode from "vscode";

/**
 * Provides catkin build and test tasks.
 */
export default class CatkinTaskProvider implements vscode.TaskProvider {
    public provideTasks(token?: vscode.CancellationToken): vscode.ProviderResult<vscode.Task[]> {
        let buildCommand: string;
        let testCommand: string;

        if (extension.buildSystem === extension.BuildSystem.CatkinMake) {
            buildCommand = `catkin_make --directory "${extension.baseDir}"`;
            testCommand = `${buildCommand} run_tests`;
        } else if (extension.buildSystem === extension.BuildSystem.CatkinTools) {
            buildCommand = `catkin build --workspace "${extension.baseDir}"`;
            testCommand = `${buildCommand} --catkin-make-args run_tests`;
        }

        const make = new vscode.Task({ type: "catkin" }, "make", "catkin");
        make.execution = new vscode.ShellExecution(buildCommand, {
            env: extension.env
        });
        make.group = vscode.TaskGroup.Build;
        make.problemMatchers = ["$catkin-gcc"];

        const test = new vscode.Task({ type: "catkin", target: "run_tests" }, "run_tests", "catkin");
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
