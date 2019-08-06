// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as os from "os";

import * as native_provider from "./local-process-items-provider";
import * as process_item from "./process-entry";
import * as utils from "./utils";

export class WmicProvider extends native_provider.LocalProcessQuickPickItemsProvider {
    // Perf numbers on Win10:
    // | # of processes | Time (ms) |
    // |----------------+-----------|
    // |            309 |       413 |
    // |            407 |       463 |
    // |            887 |       746 |
    // |           1308 |      1132 |

    protected getInternalProcessEntries(): Promise<process_item.ProcessEntry[]> {
        const wmicCommand: string = 'wmic process get Name,ProcessId,CommandLine /FORMAT:list';
        return utils.execChildProcess(wmicCommand, null).then((processes) => {
            return WmicProcessParser.ParseProcessFromWmic(processes);
        });
    }
}

// tslint:disable-next-line: max-classes-per-file
export class WmicProcessParser {
    private static get wmicNameTitle(): string { return 'Name'; }
    private static get wmicCommandLineTitle(): string { return 'CommandLine'; }
    private static get wmicPidTitle(): string { return 'ProcessId'; }

    // Only public for tests.
    public static ParseProcessFromWmic(processes: string): process_item.ProcessEntry[] {
        let lines: string[] = processes.split(os.EOL);
        let currentProcess: process_item.ProcessEntry = new process_item.ProcessEntry(null, null, null);
        let processEntries: process_item.ProcessEntry[] = [];

        for (let i: number = 0; i < lines.length; i++) {
            let line: string = lines[i];
            if (!line) {
                continue;
            }

            WmicProcessParser.parseLineFromWmic(line, currentProcess);

            // Each entry of processes has ProcessId as the last line
            if (line.lastIndexOf(WmicProcessParser.wmicPidTitle, 0) === 0) {
                processEntries.push(currentProcess);
                currentProcess = new process_item.ProcessEntry(null, null, null);
            }
        }

        return processEntries;
    }

    private static parseLineFromWmic(line: string, process: process_item.ProcessEntry): void {
        let splitter: number = line.indexOf('=');
        if (splitter >= 0) {
            let key: string = line.slice(0, line.indexOf('=')).trim();
            let value: string = line.slice(line.indexOf('=') + 1).trim();
            if (key === WmicProcessParser.wmicNameTitle) {
                process.name = value;
            } else if (key === WmicProcessParser.wmicPidTitle) {
                process.pid = value;
            } else if (key === WmicProcessParser.wmicCommandLineTitle) {
                const extendedLengthPath: string = '\\??\\';
                if (value.lastIndexOf(extendedLengthPath, 0) === 0) {
                    value = value.slice(extendedLengthPath.length);
                }

                process.commandLine = value;
            }
        }
    }
}
