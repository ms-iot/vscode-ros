// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as path from "path";

export interface IPackageInfo {
    name: string;
    version: string;
    aiKey: string;
}

export function getPackageInfo(context: vscode.ExtensionContext): IPackageInfo {
    const metadataFile: string = "package.json";
    const metadata = require(path.join(context.extensionPath, metadataFile));
    if (metadata && ("name" in metadata) && ("version" in metadata) && ("aiKey" in metadata)) {
        return {
            name: metadata.name,
            version: metadata.version,
            aiKey: metadata.aiKey,
        };
    }
    return undefined;
}

export function getExtensionConfiguration(): vscode.WorkspaceConfiguration {
    const rosConfigurationName: string = "ros";
    return vscode.workspace.getConfiguration(rosConfigurationName);
}

export function createOutputChannel(): vscode.OutputChannel {
    return vscode.window.createOutputChannel("ROS");
}
