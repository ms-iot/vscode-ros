// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as debug_protocol from "vscode-debugprotocol";

// tslint:disable-next-line: max-line-length
export interface IAttachRequest extends vscode.DebugConfiguration, debug_protocol.DebugProtocol.AttachRequestArguments {
    runtime?: string;
    processId?: number | string;
}

// tslint:disable-next-line: max-line-length
export interface ILaunchRequest extends vscode.DebugConfiguration, debug_protocol.DebugProtocol.LaunchRequestArguments {
    target: string;
    args: Array<string>;
}
