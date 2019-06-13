// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as ros_provider from "./providers/ros";

export function getRosDebugConfigurationProvider(): vscode.DebugConfigurationProvider {
    return new ros_provider.RosDebugConfigurationProvider();
}
