// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";
import * as debug_protocol from "vscode-debugprotocol";

// tslint:disable-next-line: max-line-length
export interface IAttachRequest extends debug_protocol.DebugProtocol.AttachRequestArguments, vscode.DebugConfiguration {
    runtime?: string;
    processId?: number | string;
}

// tslint:disable-next-line: max-line-length
export interface IResolvedAttachRequest extends IAttachRequest {
    runtime: string;
    processId: number;
}

// tslint:disable-next-line: max-line-length
export interface ILaunchRequest extends debug_protocol.DebugProtocol.AttachRequestArguments, vscode.DebugConfiguration {
    command?: string;
    package?: string;
    target?: string;
}
