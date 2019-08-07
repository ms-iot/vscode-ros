// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";

/**
 * Gets stringified settings to pass to the debug server.
 */
export async function getDebugSettings(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.GetDebugSettings);

    return JSON.stringify({ env: extension.env });
}

export async function getPtvsdFromPythonExtension(): Promise<string>
{
    // instead of requiring presence of correctly versioned ptvsd from pip (https://github.com/Microsoft/ptvsd)
    // use ptvsd shipped with vscode-python to avoid potential version mismatch

    const pyExtensionId: string = 'ms-python.python';
    const pyExtension: vscode.Extension<IPythonExtensionApi> = vscode.extensions.getExtension(pyExtensionId);
    if (pyExtension) {
        if (!pyExtension.isActive) {
            await pyExtension.activate();
        }

        // tslint:disable-next-line:strict-boolean-expressions
        if (pyExtension.exports && pyExtension.exports.debug) {
            const mockHost: string = "localhost";
            const mockPort: number = 8888;
            const mockWait: boolean = true;

            let ptvsdLaunchCommand = await pyExtension.exports.debug.getRemoteLauncherCommand(mockHost, mockPort, mockWait);

            // ptvsdLaunchCommand[0] is the absolute path to the ptvsd launcher script
            let ptvsdModule: string = ptvsdLaunchCommand[0];

            // ptvsd from vscode-python seems to require an extra --default flag (as in ptvsdLaunchCommand)
            // manually add it here
            ptvsdModule = [ptvsdModule, "--default"].join(" ");
            return ptvsdModule;
        } else {
            throw new Error(`Update extension [${pyExtensionId}] to debug Python projects.`);
        }
    }
    throw new Error("Failed to retrieve ptvsd from Python extension!");
}
