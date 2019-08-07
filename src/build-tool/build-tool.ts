// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import * as pfs from "../promise-fs";
import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import * as catkin_make from "./catkin-make";
import * as catkin_tools from "./catkin-tools";
import * as colcon from "./colcon";

export abstract class BuildTool {
    static current: BuildTool;
    static registerTaskProvider(): vscode.Disposable {
        return this.current._registerTaskProvider();
    }

    static async createPackage(context: vscode.ExtensionContext) {
        const reporter = telemetry.getReporter(context);
        reporter.sendTelemetryCommand(extension.Commands.CreateCatkinPackage);
        return this.current._createPackage();
    }

    protected abstract _registerTaskProvider(): vscode.Disposable;
    protected abstract async _createPackage(): Promise<void>;
}

class NotImplementedBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return null;
    }

    protected async _createPackage(): Promise<void> {
        return;
    }
}

class CatkinCmakeBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("catkin_cmake", new catkin_make.CatkinMakeProvider());
    }

    protected async _createPackage(): Promise<void> {
        return catkin_make.createPackage();
    }

    static async isApplicable(dir: string): Promise<boolean> {
        return (await pfs.exists(`${dir}/.catkin_workspace`));
    }
}

class CatkinToolsBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("catkin_tools", new catkin_tools.CatkinToolsProvider());
    }

    protected async _createPackage(): Promise<void> {
        return catkin_tools.createPackage();
    }

    static async isApplicable(dir: string): Promise<boolean> {
        return (await pfs.exists(`${dir}/.catkin_tools`));
    }
}

class ColconBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("colcon", new colcon.ColconProvider());
    }

    protected async _createPackage(): Promise<void> {
        // Do nothing.
        return;
    }

    static async isApplicable(dir: string): Promise<boolean> {
        return (await colcon.isApplicable(dir));
    }
}

BuildTool.current = new NotImplementedBuildTool();

/**
 * Determines build system and workspace path in use by checking for unique
 * auto-generated files.
 */
export async function determineBuildTool(dir: string): Promise<boolean> {
    while (dir && path.dirname(dir) !== dir) {
        if (await CatkinCmakeBuildTool.isApplicable(dir)) {
            extension.setBaseDir(dir);
            BuildTool.current = new CatkinCmakeBuildTool();
            return true;
        } else if (await CatkinToolsBuildTool.isApplicable(dir)) {
            extension.setBaseDir(dir);
            BuildTool.current = new CatkinToolsBuildTool();
            return true;
        } else if (await ColconBuildTool.isApplicable(dir)) {
            extension.setBaseDir(dir);
            BuildTool.current = new ColconBuildTool();
            return true;
        }

        dir = path.dirname(dir);
    }
    return false;
}
