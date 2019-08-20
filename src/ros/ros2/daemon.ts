// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../extension";

/**
 * start the ROS2 daemon.
 */
export async function startDaemon() {
    const command: string = "ros2 daemon start";
    const exec = util.promisify(child_process.exec);
    await exec(command, { env: this.env });
}

/**
 * stop the ROS2 daemon.
 */
export async function stopDaemon() {
    const command: string = "ros2 daemon stop";
    const exec = util.promisify(child_process.exec);
    await exec(command, { env: this.env });
}

/**
 * Shows the ROS core status in the status bar.
 */
export class StatusBarItem {
    private item: vscode.StatusBarItem;
    private timeout: NodeJS.Timeout;

    public constructor() {
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);
        this.item.text = "$(question) ROS2 Daemon";
        this.item.command = extension.Commands.ShowCoreStatus;
    }

    public activate() {
        this.item.show();
        this.timeout = setTimeout(() => this.update(), 200);
    }

    public dispose() {
        clearTimeout(this.timeout);
        this.item.dispose();
    }

    private async update() {
        const command: string = "ros2 daemon status";
        const exec = util.promisify(child_process.exec);
        const { stdout } = await exec(command, { env: extension.env });
        const status: boolean = stdout.includes("The daemon is running");
        this.item.text = (status ? "$(check)" : "$(x)") + " ROS2 Daemon";
        this.timeout = setTimeout(() => this.update(), 200);
        return;
    }
}
