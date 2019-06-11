// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import TelemetryReporter from "vscode-extension-telemetry";

import * as vscodeUtils from "./vscode-utils"

interface ITelemetryReporter {
    sendTelemetryEvent(
        eventName: string,
        properties?: {
            [key: string]: string;
        },
        measurements?: {
            [key: string]: number;
        }): void;
};

let reporter: TelemetryReporter;

function getTelemetryReporter(context: vscode.ExtensionContext): ITelemetryReporter {
    if (reporter) {
        return reporter;
    }

    const packageInfo = vscodeUtils.getPackageInfo(context);
    reporter = new TelemetryReporter(packageInfo.name, packageInfo.version, packageInfo.aiKey);
    context.subscriptions.push(reporter);
    return reporter;
}

enum TelemetryEvent {
    activate = "activate",
    command = "command",
}

export interface ILogger {
    logCommand(commandName: string): void;
    logActivate(result: string): void;
}

class Logger {
    private telemetryReporter: ITelemetryReporter;

    constructor(context: vscode.ExtensionContext) {
        this.telemetryReporter = getTelemetryReporter(context);
    }

    public logCommand(commandName: string): void {
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.command, {
            name: commandName,
        });
    }

    public logActivate(result: string): void {
        this.telemetryReporter.sendTelemetryEvent(TelemetryEvent.activate, {
            result,
        });
    }
}

export function getLogger(context: vscode.ExtensionContext): ILogger {
    const logger: Logger = new Logger(context);
    return logger;
}
