// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as child_process from "child_process";
import * as os from "os";
import * as port_finder from "portfinder";
import * as sudo from "sudo-prompt";
import * as util from "util";

import * as extension from "../../../extension";

import * as process_picker from "../../process-picker/process-picker";
import * as picker_items_provider_factory from "../../process-picker/process-items-provider";
import * as requests from "../../requests";
import * as utils from "../../utils";

const promisifiedExec = util.promisify(child_process.exec);
const promisifiedSudoExec = util.promisify(
    (command: any, options: any, cb: any) =>
        sudo.exec(command, options,
            (error, stdout, stderr) => cb(error, stdout)));

export interface IResolvedAttachRequest extends requests.IAttachRequest {
    runtime: string;
    processId: number;
    commandLine: string;
}

export class AttachResolver implements vscode.DebugConfigurationProvider {
    private readonly supportedRuntimeTypes = [
        "C++",
        "Python",
    ];

    public async resolveDebugConfigurationWithSubstitutedVariables(folder: vscode.WorkspaceFolder | undefined, config: requests.IAttachRequest, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration> {
        // ${command:} variables only get resolved before passed to debug adapter, need to be manually resolve here
        // all ${action:} variables need to be resolved before our resolver propagates the configuration to actual debugger

        await this.resolveRuntimeIfNeeded(this.supportedRuntimeTypes, config);
        await this.resolveProcessIdIfNeeded(config);
        await this.resolveCommandLineIfNeeded(config);

        // propagate debug configuration to Python or C++ debugger depending on the chosen runtime type
        this.launchAttachSession(config as IResolvedAttachRequest);

        // Return null as we have spawned new debug session
        return null;
    }

    private async launchAttachSession(config: IResolvedAttachRequest) {
        if (!config.runtime || !config.processId) {
            return;
        }

        let debugConfig: ICppvsdbgAttachConfiguration | ICppdbgAttachConfiguration | IPythonAttachConfiguration;
        if (config.runtime === "C++") {
            if (os.platform() === "win32") {
                const cppvsdbgAttachConfig: ICppvsdbgAttachConfiguration = {
                    name: `C++: ${config.processId}`,
                    type: "cppvsdbg",
                    request: "attach",
                    processId: config.processId,
                };
                debugConfig = cppvsdbgAttachConfig;
            } else {
                const cppdbgAttachConfig: ICppdbgAttachConfiguration = {
                    name: `C++: ${config.processId}`,
                    type: "cppdbg",
                    request: "attach",
                    program: config.commandLine,
                    processId: config.processId,
                    setupCommands: [
                        {
                            text: "-enable-pretty-printing",
                            description: "Enable pretty-printing for gdb",
                            ignoreFailures: true
                        }
                    ]
                };
                debugConfig = cppdbgAttachConfig;
            }

        } else if (config.runtime === "Python") {
            const host = "localhost";
            const port = await port_finder.getPortPromise();
            const ptvsdInjectCommand = await utils.getPtvsdInjectCommand(host, port, config.processId);
            try {
                if (os.platform() === "win32") {
                    const processOptions: child_process.ExecOptions = {
                        cwd: vscode.workspace.rootPath,
                        env: await extension.resolvedEnv(),
                    };

                    // "ptvsd --pid" works with child_process.exec() on Windows
                    const result = await promisifiedExec(ptvsdInjectCommand, processOptions);
                } else {
                    const processOptions = {
                        name: "ptvsd",
                    };

                    // "ptvsd --pid" requires elevated permission on Ubuntu
                    const result = await promisifiedSudoExec(ptvsdInjectCommand, processOptions);
                }

            } catch (error) {
                const errorMsg = `Command [${ptvsdInjectCommand}] failed!`;
                throw (new Error(errorMsg));
            }

            let statusMsg = `New ptvsd instance running on ${host}:${port} `;
            statusMsg += `injected into process [${config.processId}].` + os.EOL;
            statusMsg += `To re-attach to process [${config.processId}] after disconnecting, `;
            statusMsg += `please create a separate Python remote attach debug configuration `;
            statusMsg += `that uses the host and port listed above.`;
            extension.outputChannel.appendLine(statusMsg);
            extension.outputChannel.show(true);
            vscode.window.showInformationMessage(statusMsg);

            const pythonattachdebugconfiguration: IPythonAttachConfiguration = {
                name: `Python: ${config.processId}`,
                type: "python",
                request: "attach",
                port: port,
                host: host,
            };
            debugConfig = pythonattachdebugconfiguration;
        }

        if (!debugConfig) {
            return;
        }
        const launched = await vscode.debug.startDebugging(undefined, debugConfig);
        if (!launched) {
            throw (new Error(`Failed to start debug session!`));
        }
    }

    private async resolveRuntimeIfNeeded(supportedRuntimeTypes: string[], config: requests.IAttachRequest) {
        if (config.runtime && config.runtime !== "${action:pick}") {
            return;
        }

        const chooseRuntimeOptions: vscode.QuickPickOptions = {
            placeHolder: "Choose runtime type of node to attach to.",
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

        const processItemsProvider = picker_items_provider_factory.LocalProcessItemsProviderFactory.Get();
        const processPicker = new process_picker.LocalProcessPicker(processItemsProvider);
        const process = await processPicker.pick();
        config.processId = process.pid;
    }

    private async resolveCommandLineIfNeeded(config: requests.IAttachRequest) {
        // this step is only needed on Ubuntu when user has specified PID of C++ executable to attach to
        if (os.platform() === "win32" || config.commandLine || config.runtime !== "C++") {
            return;
        }

        if (!config.processId) {
            throw (new Error("No PID specified!"));
        }
        try {
            const result = await promisifiedExec(`ls -l /proc/${config.processId}/exe`);

            // contains a space
            const searchTerm = "-> ";
            const indexOfFirst = result.stdout.indexOf(searchTerm);
            config.commandLine = result.stdout.substring(indexOfFirst + searchTerm.length).trim();
        } catch (error) {
            throw (new Error(`Failed to resolve command line for process [${config.processId}]!`));
        }
    }
}
