// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as os from "os";

import * as process_item from "./process-entry";
import * as local_provider from "./local-process-items-provider";
import * as utils from "./utils";

export class PsProvider extends local_provider.LocalProcessQuickPickItemsProvider {
    // Perf numbers:
    // OS X 10.10
    // | # of processes | Time (ms) |
    // |----------------+-----------|
    // |            272 |        52 |
    // |            296 |        49 |
    // |            384 |        53 |
    // |            784 |       116 |
    //
    // Ubuntu 16.04
    // | # of processes | Time (ms) |
    // |----------------+-----------|
    // |            232 |        26 |
    // |            336 |        34 |
    // |            736 |        62 |
    // |           1039 |       115 |
    // |           1239 |       182 |

    // ps outputs as a table. With the option "ww", ps will use as much width as necessary.
    // However, that only applies to the right-most column. Here we use a hack of setting
    // the column header to 50 a's so that the second column will have at least that many
    // characters. 50 was chosen because that's the maximum length of a "label" in the
    // QuickPick UI in VSCode.

    protected getInternalProcessEntries(): Promise<process_item.ProcessEntry[]> {
        let processCmd: string = '';
        switch (os.platform()) {
            case 'darwin':
                processCmd = PsProcessParser.psDarwinCommand;
                break;
            case 'linux':
                processCmd = PsProcessParser.psLinuxCommand;
                break;
            default:
                return Promise.reject<process_item.ProcessEntry[]>(new Error(`Operating system "${os.platform()}" not support.`));
        }
        return utils.execChildProcess(processCmd, null).then((processes) => {
            return PsProcessParser.ParseProcessFromPs(processes);
        });
    }
}

// tslint:disable-next-line: max-classes-per-file
export class PsProcessParser {
    private static get secondColumnCharacters(): number { return 50; }
    private static get commColumnTitle(): string { return Array(PsProcessParser.secondColumnCharacters).join("a"); }
    // the BSD version of ps uses '-c' to have 'comm' only output the executable name and not
    // the full path. The Linux version of ps has 'comm' to only display the name of the executable
    // Note that comm on Linux systems is truncated to 16 characters:
    // https://bugzilla.redhat.com/show_bug.cgi?id=429565
    // Since 'args' contains the full path to the executable, even if truncated, searching will work as desired.
    public static get psLinuxCommand(): string {
        return `ps axww -o pid=,comm=${PsProcessParser.commColumnTitle},args=`;
    }
    public static get psDarwinCommand(): string {
        return `ps axww -o pid=,comm=${PsProcessParser.commColumnTitle},args= -c`;
    }

    // Only public for tests.
    public static ParseProcessFromPs(processes: string): process_item.ProcessEntry[] {
        let lines: string[] = processes.split(os.EOL);
        return PsProcessParser.ParseProcessFromPsArray(lines);
    }

    public static ParseProcessFromPsArray(processArray: string[]): process_item.ProcessEntry[] {
        let processEntries: process_item.ProcessEntry[] = [];

        // lines[0] is the header of the table
        for (let i: number = 1; i < processArray.length; i++) {
            let line: string = processArray[i];
            if (!line) {
                continue;
            }

            let processEntry: process_item.ProcessEntry = PsProcessParser.parseLineFromPs(line);
            processEntries.push(processEntry);
        }

        return processEntries;
    }

    private static parseLineFromPs(line: string): process_item.ProcessEntry {
        // Explanation of the regex:
        //   - any leading whitespace
        //   - PID
        //   - whitespace
        //   - executable name --> this is PsAttachItemsProvider.secondColumnCharacters - 1 because ps reserves one character
        //     for the whitespace separator
        //   - whitespace
        //   - args (might be empty)
        const psEntry: RegExp = new RegExp(`^\\s*([0-9]+)\\s+(.{${PsProcessParser.secondColumnCharacters - 1}})\\s+(.*)$`);
        const matches: RegExpExecArray = psEntry.exec(line);
        if (matches && matches.length === 4) {
            const pid: string = matches[1].trim();
            const executable: string = matches[2].trim();
            const cmdline: string = matches[3].trim();
            return new process_item.ProcessEntry(executable, pid, cmdline);
        }
    }
}
