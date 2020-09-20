// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as util from "util";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";

export async function oneTimePromiseFromEvent(eventCall, filter = undefined) : Promise<any> {
    return new Promise(resolve => {
        let disposable: vscode.Disposable;
        disposable = eventCall(event => {
            if (filter && !filter(event)) {
                return;
            }

            disposable.dispose();
            resolve(event);
        });
    });
}

async function onDidEndTaskProcess(task) : Promise<vscode.TaskProcessEndEvent> {
    return oneTimePromiseFromEvent(vscode.tasks.onDidEndTaskProcess, event => event.execution === task);
}

/**
 * Gets stringified settings to pass to the debug server.
 */
export async function getDebugSettings(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.GetDebugSettings);

    return JSON.stringify({ env: extension.env });
}

export async function getPtvsdInjectCommand(host: string, port: number, pid: number): Promise<string> {
    // instead of requiring presence of correctly versioned ptvsd from pip (https://github.com/Microsoft/ptvsd)
    // use ptvsd shipped with vscode-python to avoid potential version mismatch

    const pyExtensionId: string = "ms-python.python";
    const pyExtension: vscode.Extension<IPythonExtensionApi> = vscode.extensions.getExtension(pyExtensionId);
    if (pyExtension) {
        if (!pyExtension.isActive) {
            await pyExtension.activate();
        }

        // tslint:disable-next-line:strict-boolean-expressions
        if (pyExtension.exports && pyExtension.exports.debug) {
            // hack python extension's api to get command for injecting ptvsd

            // pass false for waitForDebugger so the --wait flag won't be added
            const waitForDebugger: boolean = false;
            const ptvsdCommand = await pyExtension.exports.debug.getRemoteLauncherCommand(host, port, waitForDebugger);

            // prepend python interpreter
            ptvsdCommand.unshift("python");
            // append the --pid flag
            ptvsdCommand.push("--pid", pid.toString());
            return ptvsdCommand.join(" ");
        } else {
            throw new Error(`Update extension [${pyExtensionId}] to debug Python projects.`);
        }
    }
    throw new Error("Failed to retrieve ptvsd from Python extension!");
}

export async function launchFirstTaskMatchingName(name: string) {
    if (typeof(name) !== "string") {
        return;
    }
    return vscode.tasks.fetchTasks()
        .then(tasks => {
            const taskToExecute = tasks.filter((task, i) => {
                return task.name === name;
            });
            
            if (taskToExecute.length === 0) {
                throw new Error(`Could not find pre-launch task "${name}".`);
            }
            
            return vscode.tasks.executeTask(taskToExecute[0]);
        })
        .then(taskExecution => onDidEndTaskProcess(taskExecution))
        .then(event => {
            if (event && event.exitCode !== 0) {
                throw new Error(`Pre-launch task "${name}" failed with exit code ${event.exitCode}.`);
            }
        });
}
