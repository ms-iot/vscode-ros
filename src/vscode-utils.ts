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
    const extensionMetadata = require(path.join(context.extensionPath, metadataFile));
    if ("name" in extensionMetadata &&
        "version" in extensionMetadata &&
        "aiKey" in extensionMetadata) {
        return {
            name: extensionMetadata.name,
            version: extensionMetadata.version,
            aiKey: extensionMetadata.aiKey,
        };
    }
    return undefined;
}
