// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import { rosApi } from "../../../ros/ros";

// interact with the user to create a roslaunch or rosrun configuration
export class RosDebugConfigurationProvider implements vscode.DebugConfigurationProvider {
    public async provideDebugConfigurations(
        folder: vscode.WorkspaceFolder | undefined,
        token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {
        const type = await vscode.window.showQuickPick(
            ["ROS: Launch", "ROS: Debug Launch File", "ROS: Attach"], { placeHolder: "Choose a request type" });
        if (!type) {
            return [];
        }

        switch (type) {
            case "ROS: Debug Launch File":
            case "ROS: Launch": {
                const packageName = await vscode.window.showQuickPick(rosApi.getPackageNames(), {
                    placeHolder: "Choose a package",
                });
                if (!packageName) {
                    return [];
                }
                const launchFiles = (await rosApi.findPackageLaunchFiles(packageName)).concat(await rosApi.findPackageTestFiles(packageName));
                const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
                const target = await vscode.window.showQuickPick(
                    launchFileBasenames, { placeHolder: "Choose a launch file" });
                const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
                if (!launchFilePath) {
                    return [];
                }

                if (type === "ROS: Debug Launch File") {
                    return [{
                        name: type,
                        request: "debug_launch",
                        target: `${launchFilePath}`,
                        type: "ros",
                    }];
                } else {
                    return [{
                        name: type,
                        request: "launch",
                        target: `${launchFilePath}`,
                        launch: ["rviz", "gz", "gzclient", "gzserver"],
                        type: "ros",
                    }];
                }
            }
            case "ROS: Attach": {
                return [{
                    name: "ROS: Attach",
                    request: "attach",
                    type: "ros",
                }];
            }
        }

        return [];
    }
}
