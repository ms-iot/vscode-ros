// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import TelemetryReporter from "vscode-extension-telemetry";

import * as vscodeUtils from "./vscode-utils"

export enum EventName {
    Activate = "activate",
    Command = "command",
}

export interface ITelemetryReporter {
    sendTelemetryEvent(eventName: string, properties?: {
        [key: string]: string;
    }, measurements?: {
        [key: string]: number;
    }): void;
};

let reporter: ITelemetryReporter = undefined;

export function getReporter(context: vscode.ExtensionContext): ITelemetryReporter {
    if (reporter)
    {
        return reporter;
    }

    const packageInfo = vscodeUtils.getPackageInfo(context);
    const newReporter = new TelemetryReporter(packageInfo.name, packageInfo.version, packageInfo.aiKey);
    context.subscriptions.push(newReporter);
    return newReporter;
}
