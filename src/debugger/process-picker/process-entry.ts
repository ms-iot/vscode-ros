// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as vscode from "vscode";

export interface IProcessEntry {
    commandLine: string;
    pid: string;
}

export class ProcessEntry implements IProcessEntry {
    // constructor(readonly name: string, readonly pid: string, readonly commandLine: string) {}
    constructor(public name: string, public pid: string, public commandLine: string) {}
}

export interface IProcessQuickPickItem extends vscode.QuickPickItem {
    pid: string;
}

export function createProcessQuickPickItem(entry: ProcessEntry): IProcessQuickPickItem
{
    return {
        label: entry.name,
        description: entry.pid,
        detail: entry.commandLine,
        pid: entry.pid,
    };
}
