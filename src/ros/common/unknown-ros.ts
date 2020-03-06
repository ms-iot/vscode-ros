// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as ros from "../ros";

/**
 * Provides behavior for unknown ROS environment.
 */
export class UnknownROS implements ros.ROSApi {
    public setContext(context: vscode.ExtensionContext, env: any) {
    }

    public getPackageNames(): Promise<string[]> {
        console.error("Unknown ROS distro.");
        return;
    }

    public getPackages(): Promise<{ [name: string]: () => Promise<string> }> {
        console.error("Unknown ROS distro.");
        return;
    }

    public getIncludeDirs(): Promise<string[]> {
        console.error("Unknown ROS distro.");
        return;
    }

    public getWorkspaceIncludeDirs(workspaceDir: string): Promise<string[]> {
        console.error("Unknown ROS distro.");
        return;
    }

    public findPackageExecutables(packageName: string): Promise<string[]> {
        console.error("Unknown ROS distro.");
        return;
    }

    public findPackageLaunchFiles(packageName: string): Promise<string[]> {
        console.error("Unknown ROS distro.");
        return;
    }

    public startCore() {
        console.error("Unknown ROS distro.");
        return;
    }

    public stopCore() {
        console.error("Unknown ROS distro.");
        return;
    }

    public rosdep() {
      console.error("Unknown ROS distro.");
      return;
    }

    public activateCoreMonitor(): vscode.Disposable {
        console.error("Unknown ROS distro.");
        return;
    }

    public showCoreMonitor() {
        console.error("Unknown ROS distro.");
        return;
    }

    public activateRosrun(packageName: string, executableName:string, argument: string): vscode.Terminal {
        console.error("Unknown ROS distro.");
        return;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
        console.error("Unknown ROS distro.");
        return;
    }
}
