// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as util from "util";
import * as vscode from "vscode";

import * as extension from "../../extension";
import * as ros2_monitor from "./ros2-monitor"

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
    private ros2cli: ros2_monitor.XmlRpcApi;

    public constructor() {
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);
        this.item.text = "$(question) ROS2 Daemon";
        this.item.command = extension.Commands.ShowCoreStatus;
        this.ros2cli = new ros2_monitor.XmlRpcApi();
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
        let status: boolean = false;
        try {
            const result = await this.ros2cli.getNodeNamesAndNamespaces();
            status = true;
        } catch (error) {
            // do nothing.
        } finally {
            this.item.text = (status ? "$(check)" : "$(x)") + " ROS2 Daemon";
            this.timeout = setTimeout(() => this.update(), 200);
        }
    }
}
