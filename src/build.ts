// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as _ from "underscore";
import * as vscode from "vscode";

import * as extension from "./extension";
import * as pfs from "./promise-fs";
import * as telemetry from "./telemetry-helper";
import * as utils from "./utils";

const PYTHON_AUTOCOMPLETE_PATHS = "python.autoComplete.extraPaths";

/**
 * Creates config files which don't exist.
 */
export async function createConfigFiles() {
    const config = vscode.workspace.getConfiguration();

    // Update the Python path if required.
    if (config.get(PYTHON_AUTOCOMPLETE_PATHS, []).length === 0) {
        updatePythonPathInternal();
    }

    // Ensure the ".vscode" directory exists then update the C++ path.
    const dir = path.join(vscode.workspace.rootPath, ".vscode");

    if (!await pfs.exists(dir)) {
        await pfs.mkdir(dir);
    }

    pfs.exists(path.join(dir, "c_cpp_properties.json")).then(exists => {
        if (!exists) {
            updateCppPropertiesInternal();
        }
    });
}

export async function updateCppProperties(context: vscode.ExtensionContext): Promise<void> {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.UpdateCppProperties);

    updateCppPropertiesInternal();
}

/**
 * Updates the `c_cpp_properties.json` file with ROS include paths.
 */
async function updateCppPropertiesInternal(): Promise<void> {
    const includes = await utils.getIncludeDirs();
    const filename = vscode.workspace.rootPath + "/.vscode/c_cpp_properties.json";

    // Get all packages within the workspace, and check if they have an include
    // directory. If so, add them to the list.
    const packages = await utils.getPackages().then(
        pkgs => _.values(pkgs).filter(pkg => pkg.startsWith(extension.baseDir))
    );

    await Promise.all(packages.map(pkg => {
        const include = path.join(pkg, "include");

        return pfs.exists(include).then(exists => {
            if (exists) {
                includes.push(include);
            }
        });
    }));

    await pfs.writeFile(filename, JSON.stringify({
        configurations: [
            {
                browse: { databaseFilename: "", limitSymbolsToIncludedHeaders: true },
                includePath: [...includes, "/usr/include"],
                name: "Linux",
            },
        ],
    }, undefined, 2));
}

export function updatePythonPath(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.UpdatePythonPath);

    updatePythonPathInternal();
}

/**
 * Updates the python autocomplete path to support ROS.
 */
function updatePythonPathInternal() {
    vscode.workspace.getConfiguration().update(PYTHON_AUTOCOMPLETE_PATHS, extension.env.PYTHONPATH.split(path.delimiter));
}
