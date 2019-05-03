// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode"

export async function setup(): Promise<void>
{
    if (process.platform === "win32")
    {
        // use cmd.exe as the integrated terminal for ROS workspace
        vscode.workspace.getConfiguration().update("terminal.integrated.shell.windows", "cmd.exe");
    }
}
