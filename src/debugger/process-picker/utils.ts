// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as child_process from "child_process";

export function execChildProcess(process: string, workingDirectory: string): Promise<string> {
    return new Promise<string>((resolve, reject) => {
        child_process.exec(process, { cwd: workingDirectory, maxBuffer: 500 * 1024 }, (error: Error, stdout: string, stderr: string) => {

            if (error) {
                reject(error);
                return;
            }

            if (stderr && stderr.length > 0 && !stderr.includes('screen size is bogus')) {
                reject(new Error(stderr));
                return;
            }

            resolve(stdout);
        });
    });
}
