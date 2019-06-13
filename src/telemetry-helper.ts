// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import TelemetryReporter from "vscode-extension-telemetry";

import * as vscode_utils from "./vscode-utils";

let reporterSingleton: TelemetryReporter;

function getTelemetryReporter(context: vscode.ExtensionContext): TelemetryReporter {
    if (reporterSingleton) {
        return reporterSingleton;
    }

    const packageInfo = vscode_utils.getPackageInfo(context);
    if (packageInfo) {
        reporterSingleton = new TelemetryReporter(packageInfo.name, packageInfo.version, packageInfo.aiKey);
        context.subscriptions.push(reporterSingleton);
    }
    return reporterSingleton;
}

enum TelemetryEvent {
    activate = "activate",
    command = "command",
}

export interface ITelemetryReporter {
    sendTelemetryActivate(): void;
    sendTelemetryCommand(commandName: string): void;
}

class SimpleReporter implements ITelemetryReporter {
    private telemetryReporter: TelemetryReporter;

    constructor(context: vscode.ExtensionContext) {
        this.telemetryReporter = getTelemetryReporter(context);
    }

    public sendTelemetryActivate(): void {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.activate);
    }

    public sendTelemetryCommand(commandName: string): void {
        if (!this.telemetryReporter) {
            return;
        }
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.command, {
            name: commandName,
        });
    }
}

export function getReporter(context: vscode.ExtensionContext): ITelemetryReporter {
    return (new SimpleReporter(context));
}
