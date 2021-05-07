// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as unknownROS from "./common/unknown-ros";
import * as ros1 from "./ros1/ros1";
import * as ros2 from "./ros2/ros2";

export interface ROSApi {
    /**
     * Set additional context.
     */
    setContext: (context: vscode.ExtensionContext, env: any) => void;

    /**
     * Get a list of packages.
     */
    getPackageNames: () => Promise<string[]>;

    /**
     * Gets a map of package names to paths.
     */
    getPackages: () => Promise<{ [name: string]: () => Promise<string> }>;

    /**
     * Gets a list of CMake include paths.
     */
    getIncludeDirs: () => Promise<string[]>;

    /**
     * Gets a list of include paths under current workspace.
     * @param workspaceDir
     */
    getWorkspaceIncludeDirs: (workspaceDir: string) => Promise<string[]>;

    /**
     * list full paths to all executables inside a package
     * @param packageName 
     */
    findPackageExecutables: (packageName: string) => Promise<string[]>;

    /**
     * list all .launch files inside a package
     * @param packageName 
     */
    findPackageLaunchFiles: (packageName: string) => Promise<string[]>;

    /**
     * list all .test files inside a package
     * @param packageName 
     */
    findPackageTestFiles: (packageName: string) => Promise<string[]>;

    /**
     * Start ROS Core
     */
    startCore: () => void;

    /**
     * Stop ROS Core
     */
    stopCore: () => void;

    /**
     * Get ROS Core status
     */
    getCoreStatus: () => Promise<boolean>;

    /**
     * Run ROSDep
     */
    rosdep: () => void;

    /**
     * Activate ROS Core monitor.
     */
    activateCoreMonitor: () => vscode.Disposable;

    /**
     * Bring up ROS Core monitor GUI.
     */
    showCoreMonitor: () => void;

    /**
     * Activate a terminal for rosrun.
     */
    activateRosrun: (packageName: string, executableName:string, argument: string) => vscode.Terminal;

    /**
     * Activate a terminal for roslaunch.
     */
    activateRoslaunch: (launchFilepath: string, argument: string) => vscode.Terminal;

    /**
     * Activate a terminal for rostest.
     */
    activateRostest: (launchFilepath: string, argument: string) => vscode.Terminal;
}

const ros1Api: ROSApi = new ros1.ROS1();
const ros2Api: ROSApi = new ros2.ROS2();
const unknownRosApi: ROSApi = new unknownROS.UnknownROS();

export let rosApi: ROSApi = unknownRosApi;

export function selectROSApi(version: string) {
    rosApi = unknownRosApi;
    switch(version.trim()) {
        case "1": {
            rosApi = ros1Api;
            break;
        }
        case "2": {
            rosApi = ros2Api;
            break;
        }
    }
}
