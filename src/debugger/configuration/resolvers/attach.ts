// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/* tslint:disable */

import * as vscode from "vscode";
import * as child_process from "child_process";
import * as os from "os";
import * as port_finder from "portfinder";

import * as extension from "../../../extension";

import * as process_picker from "../../process-picker/process-picker";
import * as picker_items_provider_factory from "../../process-picker/process-quick-pick-items-provider-factory";
import * as requests from "../../requests";

export class AttachResolver implements vscode.DebugConfigurationProvider {
    private readonly supportedRuntimeTypes = [
        "C++",
        "Python",
    ];

    public async resolveDebugConfiguration(folder: vscode.WorkspaceFolder | undefined, config: requests.IAttachRequest, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration> {
        // ${command:} variables only get resolved before passed to debug adapter, need to be manually resolve here
        // all ${action:} variables need to be resolved before our resolver propagates the configuration to actual debugger
        await this.resolveRuntimeIfNeeded(this.supportedRuntimeTypes, config);
        await this.resolveProcessIdIfNeeded(config);

        // propagate debug configuration to Python or C++ debugger depending on the chosen runtime type
        this.launchAttachSession(config);
        return config;
    }

    private async launchAttachSession(config: requests.IAttachRequest) {
        if (!config.runtime || !config.processId) {
            return;
        }

        if (config.runtime === "C++") {
            if (os.platform() === "win32") {
                let cppattachdebugconfiguration: vscode.DebugConfiguration = {
                    name: `C++: ${config.processId}`,
                    type: "cppvsdbg",
                    request: "attach",
                    processId: config.processId
                };
                vscode.debug.startDebugging(undefined, cppattachdebugconfiguration);
            }
        }
        else if (config.runtime === "Python") {
            const host = "localhost";
            const port = await port_finder.getPortPromise();
            let injectPtvsdCmd = `python -m ptvsd --host ${host} --port ${port} --pid ${config.processId}`;
            let processOptions: child_process.ExecOptions = {
                cwd: extension.baseDir,
                env: extension.env,
            };

            child_process.exec(injectPtvsdCmd, processOptions, (error, stdout, stderr) => {
                if (!error) {
                    const statusMsg = `New ptvsd instance running on ${host}:${port} injected into process [${config.processId}].`;
                    extension.outputChannel.appendLine(statusMsg);
                    extension.outputChannel.show(true);
                    vscode.window.showInformationMessage(statusMsg);

                    let pythonattachdebugconfiguration: vscode.DebugConfiguration = {
                        name: `Python: ${config.processId}`,
                        type: "python",
                        request: "attach",
                        port: port,
                        host: host,
                    };
                    vscode.debug.startDebugging(undefined, pythonattachdebugconfiguration);
                }
            });
        }
    }

    private async resolveRuntimeIfNeeded(supportedRuntimeTypes: string[], config: requests.IAttachRequest) {
        if (config.runtime && config.runtime !== "${action:pick}") {
            return;
        }

        const chooseRuntimeOptions: vscode.QuickPickOptions = {
            placeHolder: "Choose runtime type of node to attach to."
        };
        config.runtime = await vscode.window.showQuickPick(supportedRuntimeTypes, chooseRuntimeOptions).then((runtime): string => {
            if (!runtime) {
                throw new Error("Runtime type not chosen!");
            }
            return runtime;
        });
    }

    private async resolveProcessIdIfNeeded(config: requests.IAttachRequest) {
        if (config.processId && config.processId !== "${action:pick}") {
            return;
        }

        let processItemsProvider = picker_items_provider_factory.NativeAttachItemsProviderFactory.Get();
        let processPicker = new process_picker.LocalProcessPicker(processItemsProvider);
        config.processId = await processPicker.pick();
    }
}
