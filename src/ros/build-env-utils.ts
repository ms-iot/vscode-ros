// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as pfs from "../promise-fs";
import * as telemetry from "../telemetry-helper";
import { rosApi } from "./ros";

const PYTHON_AUTOCOMPLETE_PATHS = "python.autoComplete.extraPaths";
const PYTHON_ANALYSIS_PATHS = "python.analysis.extraPaths";

/**
 * Creates config files which don't exist.
 */
export async function createConfigFiles() {
    const config = vscode.workspace.getConfiguration();

    // Update the Python autocomplete paths if required.
    if (config.get(PYTHON_AUTOCOMPLETE_PATHS, []).length === 0) {
        updatePythonAutoCompletePathInternal();
    }

    // Update the Python analysis paths if required.
    if (config.get(PYTHON_ANALYSIS_PATHS, []).length === 0) {
        updatePythonAnalysisPathInternal();
    }

    const dir = path.join(vscode.workspace.rootPath, ".vscode");

    // Update the C++ path.
    pfs.exists(path.join(dir, "c_cpp_properties.json")).then(exists => {
        if (!exists) {
            updateCppPropertiesInternal();
        }
    });
}

export async function updateCppProperties(context: vscode.ExtensionContext): Promise<void> {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.UpdateCppProperties);

    updateCppPropertiesInternal();
}

/**
 * Updates the `c_cpp_properties.json` file with ROS include paths.
 */
async function updateCppPropertiesInternal(): Promise<void> {
    let includes = await rosApi.getIncludeDirs();
    const workspaceIncludes = await rosApi.getWorkspaceIncludeDirs(vscode.workspace.rootPath);
    includes = includes.concat(workspaceIncludes);

    if (process.platform === "linux") {
        includes.push(path.join("/", "usr", "include"));
    }

    // append ** so the IntelliSense engine will do a recursive search for hearder files starting from that directory
    includes = includes.map((include: string) => {
        return path.join(include, "**");
    });

    // https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/c_cpp_properties.json.md
    const cppProperties: any = {
        configurations: [
            {
                browse: {
                    databaseFilename: "${default}",
                    limitSymbolsToIncludedHeaders: false,
                },
                includePath: includes,
                name: "ROS",
            },
        ],
        version: 4,
    };

    if (process.platform === "linux") {
        cppProperties.configurations[0].intelliSenseMode = "gcc-" + process.arch
        cppProperties.configurations[0].compilerPath = "/usr/bin/gcc"
        cppProperties.configurations[0].cStandard = "gnu11"
        cppProperties.configurations[0].cppStandard = "c++14"
    }

    // Ensure the ".vscode" directory exists then update the C++ path.
    const dir = path.join(vscode.workspace.rootPath, ".vscode");

    if (!await pfs.exists(dir)) {
        await pfs.mkdir(dir);
    }

    const filename = path.join(vscode.workspace.rootPath, ".vscode", "c_cpp_properties.json");
    await pfs.writeFile(filename, JSON.stringify(cppProperties, undefined, 2));
}

export function updatePythonPath(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.UpdatePythonPath);

    updatePythonAutoCompletePathInternal();
    updatePythonAnalysisPathInternal();
}

/**
 * Updates the python autocomplete path to support ROS.
 */
function updatePythonAutoCompletePathInternal() {
    vscode.workspace.getConfiguration().update(PYTHON_AUTOCOMPLETE_PATHS, extension.env.PYTHONPATH.split(path.delimiter));
}

/**
 * Updates the python analysis path to support ROS.
 */
function updatePythonAnalysisPathInternal() {
  vscode.workspace.getConfiguration().update(PYTHON_ANALYSIS_PATHS, extension.env.PYTHONPATH.split(path.delimiter));
}
