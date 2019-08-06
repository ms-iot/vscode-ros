// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

/* tslint:disable */

import * as vscode from "vscode";

// interact with the user to create a roslaunch or rosrun configuration
export class RosDebugConfigurationProvider implements vscode.DebugConfigurationProvider {
    public async provideDebugConfigurations(folder: vscode.WorkspaceFolder | undefined, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {
        const configs: vscode.DebugConfiguration[] = undefined;
        return configs;
    }
}
