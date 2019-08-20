// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as ros_provider from "./configuration/providers/ros";
import * as attach_resolver from "./configuration/resolvers/attach";
import * as launch_resolver from "./configuration/resolvers/launch";
import * as requests from "./requests";

class RosDebugManager implements vscode.DebugConfigurationProvider {
    private configProvider: ros_provider.RosDebugConfigurationProvider;
    private attachResolver: attach_resolver.AttachResolver;
    private launchResolver: launch_resolver.LaunchResolver;

    constructor() {
        this.configProvider = new ros_provider.RosDebugConfigurationProvider();
        this.attachResolver = new attach_resolver.AttachResolver();
        this.launchResolver = new launch_resolver.LaunchResolver();
    }

    public async provideDebugConfigurations(folder: vscode.WorkspaceFolder | undefined, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {
        return this.configProvider.provideDebugConfigurations(folder, token);
    }

    public async resolveDebugConfiguration(folder: vscode.WorkspaceFolder | undefined, config: vscode.DebugConfiguration, token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration> {
        if (config.request === "attach") {
            return this.attachResolver.resolveDebugConfiguration(folder, config as requests.IAttachRequest, token);
        } else if (config.request === "launch") {
            return this.launchResolver.resolveDebugConfiguration(folder, config as requests.ILaunchRequest, token);
        }
    }
}

export function registerRosDebugManager(context: vscode.ExtensionContext): void {
    context.subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", new RosDebugManager()));
}
