// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as path from "path";
import * as vscode from 'vscode';

import * as extension from "../../extension";

import * as process_entry from "./process-entry";

export function getExtensionFilePath(extensionfile: string): string {
    return path.resolve(extension.extPath, extensionfile);
}

class RefreshButton implements vscode.QuickInputButton {
    get iconPath(): { dark: vscode.Uri; light: vscode.Uri } {
        const refreshImagePathDark: string = getExtensionFilePath(path.join("assets", "process-picker", "refresh_inverse.svg"));
        const refreshImagePathLight: string = getExtensionFilePath(path.join("assets", "process-picker", "refresh.svg"));

        return {
            dark: vscode.Uri.file(refreshImagePathDark),
            light: vscode.Uri.file(refreshImagePathLight)
        };
    }

    get tooltip(): string {
        return "Refresh process list";
    }
}

export async function showQuickPick(getAttachItems: () => Promise<process_entry.IProcessQuickPickItem[]>): Promise<process_entry.IProcessEntry> {
    return getAttachItems().then(processEntries => {
        return new Promise<process_entry.IProcessEntry>((resolve, reject) => {
            let quickPick: vscode.QuickPick<process_entry.IProcessQuickPickItem> = vscode.window.createQuickPick<process_entry.IProcessQuickPickItem>();
            quickPick.title = "Attach to process";
            quickPick.canSelectMany = false;
            quickPick.matchOnDescription = true;
            quickPick.matchOnDetail = true;
            quickPick.placeholder = "Select the process to attach to";
            quickPick.items = processEntries;
            quickPick.buttons = [new RefreshButton()];

            let disposables: vscode.Disposable[] = [];

            quickPick.onDidTriggerButton(button => {
                getAttachItems().then(processEntries => quickPick.items = processEntries);
            }, undefined, disposables);

            quickPick.onDidAccept(() => {
                if (quickPick.selectedItems.length !== 1) {
                    reject(new Error("Process not selected"));
                }

                let selected: process_entry.IProcessEntry = {
                    commandLine: quickPick.selectedItems[0].detail,
                    pid: quickPick.selectedItems[0].pid,
                }

                disposables.forEach(item => item.dispose());
                quickPick.dispose();

                resolve(selected);
            }, undefined, disposables);

            quickPick.onDidHide(() => {
                disposables.forEach(item => item.dispose());
                quickPick.dispose();

                reject(new Error("Process not selected."));
            }, undefined, disposables);

            quickPick.show();
        });
    });
}
