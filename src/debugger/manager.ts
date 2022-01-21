// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as ros_provider from "./configuration/providers/ros";
import * as attach_resolver from "./configuration/resolvers/attach";
import * as ros1_launch_resolver from "./configuration/resolvers/ros1/launch";
import * as ros2_launch_resolver from "./configuration/resolvers/ros2/launch";
import * as requests from "./requests";
import * as extension from "../extension";

class RosDebugManager implements vscode.DebugConfigurationProvider {
    private configProvider: ros_provider.RosDebugConfigurationProvider;
    private attachResolver: attach_resolver.AttachResolver;
    private ros1LaunchResolver: ros1_launch_resolver.LaunchResolver;
    private ros2LaunchResolver: ros2_launch_resolver.LaunchResolver;

    constructor() {
        this.configProvider = new ros_provider.RosDebugConfigurationProvider();
        this.attachResolver = new attach_resolver.AttachResolver();
        this.ros1LaunchResolver = new ros1_launch_resolver.LaunchResolver();
        this.ros2LaunchResolver = new ros2_launch_resolver.LaunchResolver();
    }

    public async provideDebugConfigurations(folder: vscode.WorkspaceFolder | undefined, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {
        return this.configProvider.provideDebugConfigurations(folder, token);
    }

    public async resolveDebugConfigurationWithSubstitutedVariables(folder: vscode.WorkspaceFolder | undefined, config: vscode.DebugConfiguration, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration> {
        if (config.request === "attach") {
            return this.attachResolver.resolveDebugConfigurationWithSubstitutedVariables(folder, config as requests.IAttachRequest, token);
        } else if (config.request === "launch") {
            if ((typeof extension.env.ROS_VERSION === "undefined") || (extension.env.ROS_VERSION.trim() == "1")) {
                return this.ros1LaunchResolver.resolveDebugConfigurationWithSubstitutedVariables(folder, config as requests.ILaunchRequest, token);
            } else {
                return this.ros2LaunchResolver.resolveDebugConfigurationWithSubstitutedVariables(folder, config as requests.ILaunchRequest, token);
            }
        }
    }
}

export function registerRosDebugManager(context: vscode.ExtensionContext): void {
    var rosProvider = new RosDebugManager();
    context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", rosProvider, vscode.DebugConfigurationProviderTriggerKind.Initial));
    context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", rosProvider, vscode.DebugConfigurationProviderTriggerKind.Dynamic));
}
