// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as pfs from "../promise-fs";
import * as telemetry from "../telemetry-helper";
import * as catkin_make from "./catkin-make";
import * as catkin_tools from "./catkin-tools";
import * as colcon from "./colcon";

export abstract class BuildTool {
    public static current: BuildTool;
    public static registerTaskProvider(): vscode.Disposable {
        return this.current._registerTaskProvider();
    }

    public static async createPackage(context: vscode.ExtensionContext) {
        const reporter = telemetry.getReporter(context);
        reporter.sendTelemetryCommand(extension.Commands.CreateCatkinPackage);
        return this.current._createPackage();
    }

    protected abstract _registerTaskProvider(): vscode.Disposable;
    protected abstract async _createPackage(): Promise<void>;
}

// tslint:disable-next-line: max-classes-per-file
class NotImplementedBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return null;
    }

    protected async _createPackage(): Promise<void> {
        return;
    }
}

// tslint:disable-next-line: max-classes-per-file
class CatkinCmakeBuildTool extends BuildTool {
    public static async isApplicable(dir: string): Promise<boolean> {
        return pfs.exists(`${dir}/.catkin_workspace`);
    }

    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("catkin_cmake", new catkin_make.CatkinMakeProvider());
    }

    protected async _createPackage(): Promise<void> {
        return catkin_make.createPackage();
    }
}

// tslint:disable-next-line: max-classes-per-file
class CatkinToolsBuildTool extends BuildTool {
    public static async isApplicable(dir: string): Promise<boolean> {
        return pfs.exists(`${dir}/.catkin_tools`);
    }

    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("catkin_tools", new catkin_tools.CatkinToolsProvider());
    }

    protected async _createPackage(): Promise<void> {
        return catkin_tools.createPackage();
    }
}

// tslint:disable-next-line: max-classes-per-file
class ColconBuildTool extends BuildTool {
    public static async isApplicable(dir: string): Promise<boolean> {
        return colcon.isApplicable(dir);
    }

    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("colcon", new colcon.ColconProvider());
    }

    protected async _createPackage(): Promise<void> {
        // Do nothing.
        return;
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
