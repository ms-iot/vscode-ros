// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as fs from "fs";
import * as yaml from "js-yaml";
import * as os from "os";
import * as path from "path";
import * as readline from "readline";
import * as shell_quote from "shell-quote";
import * as tmp from "tmp";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../../../extension";
import * as requests from "../../../requests";
import * as utils from "../../../utils";
import { rosApi } from "../../../../ros/ros";

const promisifiedExec = util.promisify(child_process.exec);

interface ILaunchRequest {
    nodeName: string;
    executable: string;
    arguments: string[];
    cwd: string;
    env: { [key: string]: string };
    symbolSearchPath?: string;
    additionalSOLibSearchPath?: string;
    sourceFileMap?: { [key: string]: string };
    launch?: string[];    // Scripts or executables to just launch without attaching a debugger
    attachDebugger?: string[];    // If specified, Scripts or executables to debug; otherwise attaches to everything not ignored
}

function getExtensionFilePath(extensionFile: string): string {
    return path.resolve(extension.extPath, extensionFile);
}

export class LaunchResolver implements vscode.DebugConfigurationProvider {
    // tslint:disable-next-line: max-line-length
    public async resolveDebugConfigurationWithSubstitutedVariables(folder: vscode.WorkspaceFolder | undefined, config: requests.ILaunchRequest, token?: vscode.CancellationToken) {
        if (!path.isAbsolute(config.target)) {
            throw new Error("Launch request requires an absolute path as target.");
        }
        else if (path.extname(config.target) !== ".py" && path.extname(config.target) !== ".xml") {
            throw new Error("Launch request requires an extension '.py' or '.xml' as target.");
        }

        const rosExecOptions: child_process.ExecOptions = {
            env: {
                ...await extension.resolvedEnv(),
                ...config.env,
            },
        };

        extension.outputChannel.appendLine("Executing dumper with the following environment:");
        extension.outputChannel.appendLine(JSON.stringify(rosExecOptions.env, null, 2));

        let ros2_launch_dumper = getExtensionFilePath(path.join("assets", "scripts", "ros2_launch_dumper.py"));

        let args = [config.target]
        if (config.arguments) {
            for (let arg of config.arguments) {
                args.push(`"${arg}"`);
            }
        }

        const pythonLaunchConfig: IPythonLaunchConfiguration = {
            name: ros2_launch_dumper,
            type: "python",
            request: "launch",
            program: ros2_launch_dumper,
            args: args,
            env: rosExecOptions.env,
            stopOnEntry: true,
            justMyCode: false,
        };

        const launched = await vscode.debug.startDebugging(undefined, pythonLaunchConfig);
        if (!launched) {
            throw (new Error(`Failed to start debug session!`));
        }

        // Return null as we have spawned new debug requests
        return null;
    }
}
