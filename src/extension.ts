// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as cpp_formatter from "./cpp-formatter";
import * as debug_provider from "./debugger/configuration/provider";
import * as debug_utils from "./debugger/utils";
import * as pfs from "./promise-fs";
import * as telemetry from "./telemetry-helper";
import * as vscode_utils from "./vscode-utils";

import * as catkin from "./catkin/catkin";

import * as ros_build_utils from "./ros/build-env-utils";
import * as ros_cli from "./ros/cli";
import * as ros_core from "./ros/core-helper";
import * as ros_utils from "./ros/utils";
import URDFPreviewManager from "./urdfPreview/previewManager"

/**
 * The catkin workspace base dir.
 */
export let baseDir: string;

export enum BuildSystem { None, CatkinMake, CatkinTools };

/**
 * The build system in use.
 */
export let buildSystem: BuildSystem;

/**
 * The sourced ROS environment.
 */
export let env: any;

let onEnvChanged = new vscode.EventEmitter<void>();

/**
 * Triggered when the env is soured.
 */
export let onDidChangeEnv = onEnvChanged.event;

/**
 * Subscriptions to dispose when the environment is changed.
 */
let subscriptions = <vscode.Disposable[]>[];

export enum Commands {
    CreateCatkinPackage = "ros.createCatkinPackage",
    CreateTerminal = "ros.createTerminal",
    GetDebugSettings = "ros.getDebugSettings",
    Rosrun = "ros.rosrun",
    Roslaunch = "ros.roslaunch",
    ShowCoreStatus = "ros.showCoreStatus",
    StartRosCore = "ros.startCore",
    TerminateRosCore = "ros.stopCore",
    UpdateCppProperties = "ros.updateCppProperties",
    UpdatePythonPath = "ros.updatePythonPath",
    PreviewURDF = "ros.previewUrdf",
}

export async function activate(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);

    // Activate if we're in a catkin workspace.
    await determineBuildSystem(vscode.workspace.rootPath);

    if (buildSystem == BuildSystem.None) {
        return;
    }

    // Activate components when the ROS env is changed.
    context.subscriptions.push(onDidChangeEnv(activateEnvironment.bind(null, context)));

    // Activate components which don't require the ROS env.
    context.subscriptions.push(vscode.languages.registerDocumentFormattingEditProvider(
        "cpp", new cpp_formatter.CppFormatter()
    ));

    URDFPreviewManager.INSTANCE.setContext(context);

    // Source the environment, and re-source on config change.
    let config = vscode_utils.getExtensionConfiguration();

    context.subscriptions.push(vscode.workspace.onDidChangeConfiguration(() => {
        const updatedConfig = vscode_utils.getExtensionConfiguration();
        const fields = Object.keys(config).filter(k => !(config[k] instanceof Function));
        const changed = fields.some(key => updatedConfig[key] !== config[key]);

        if (changed) {
            sourceRosAndWorkspace();
        }

        config = updatedConfig;
    }));

    sourceRosAndWorkspace().then(() =>
    {
        vscode.window.registerWebviewPanelSerializer('urdfPreview', URDFPreviewManager.INSTANCE);
    });

    reporter.sendTelemetryActivate();

    return {
        getBaseDir: () => baseDir,
        getEnv: () => env,
        onDidChangeEnv: (listener: () => any, thisArg: any) => onDidChangeEnv(listener, thisArg),
    };
}

export function deactivate() {
    subscriptions.forEach(disposable => disposable.dispose());
}

/**
 * Determines build system and workspace path in use by checking for unique
 * auto-generated files.
 */
async function determineBuildSystem(dir: string): Promise<void> {
    while (dir && path.dirname(dir) !== dir) {
        if (await pfs.exists(`${dir}/.catkin_workspace`)) {
            baseDir = dir;
            buildSystem = BuildSystem.CatkinMake;
            return;
        } else if (await pfs.exists(`${dir}/.catkin_tools`)) {
            baseDir = dir;
            buildSystem = BuildSystem.CatkinTools;
            return;
        }

        dir = path.dirname(dir);
    }

    buildSystem = BuildSystem.None;
}

/**
 * Activates components which require a ROS env.
 */
function activateEnvironment(context: vscode.ExtensionContext) {
    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop().dispose();
    }

    if (typeof env.ROS_ROOT === "undefined") {
        return;
    }

    // Set up the master.
    const roscoreApi = new ros_core.XmlRpcApi(env.ROS_MASTER_URI);
    const coreStatusItem = new ros_core.StatusBarItem(roscoreApi);

    coreStatusItem.activate();

    subscriptions.push(coreStatusItem);
    subscriptions.push(vscode.workspace.registerTaskProvider("catkin", catkin.getCatkinTaskProvider()));
    subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", debug_provider.getRosDebugConfigurationProvider()));

    // register plugin commands
    subscriptions.push(
        vscode.commands.registerCommand(Commands.CreateCatkinPackage, () => {
            catkin.createPackage(context);
        }),
        vscode.commands.registerCommand(Commands.CreateTerminal, () => {
            ros_utils.createTerminal(context);
        }),
        vscode.commands.registerCommand(Commands.GetDebugSettings, () => {
            debug_utils.getDebugSettings(context);
        }),
        vscode.commands.registerCommand(Commands.ShowCoreStatus, () => {
            ros_core.launchMonitor(context);
        }),
        vscode.commands.registerCommand(Commands.StartRosCore, () => {
            ros_core.startCore(context);
        }),
        vscode.commands.registerCommand(Commands.TerminateRosCore, () => {
            ros_core.stopCore(context, roscoreApi);
        }),
        vscode.commands.registerCommand(Commands.UpdateCppProperties, () => {
            ros_build_utils.updateCppProperties(context);
        }),
        vscode.commands.registerCommand(Commands.UpdatePythonPath, () => {
            ros_build_utils.updatePythonPath(context);
        }),
        vscode.commands.registerCommand(Commands.Rosrun, () => {
            ros_cli.rosrun(context);
        }),
        vscode.commands.registerCommand(Commands.Roslaunch, () => {
            ros_cli.roslaunch(context);
        }),

        vscode.commands.registerCommand(Commands.PreviewURDF, () => {
            URDFPreviewManager.INSTANCE.preview(vscode.window.activeTextEditor.document.uri);
        }),
    );

    // Generate config files if they don't already exist.
    ros_build_utils.createConfigFiles();
}

/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
async function sourceRosAndWorkspace(): Promise<void> {
    env = undefined;

    const config = vscode_utils.getExtensionConfiguration();
    const distro = config.get("distro", "");
    let setupScriptExt: string;
    if (process.platform === "win32") {
        setupScriptExt = ".bat";
    }
    else {
        setupScriptExt = ".bash";
    }

    if (distro) {
        try {
            let globalInstallPath: string;
            if (process.platform === "win32") {
                globalInstallPath = path.join("C:", "opt", "ros", `${distro}`, "x64");
            }
            else {
                globalInstallPath = path.join("/", "opt", "ros", `${distro}`);
            }
            let setupScript: string = path.format({
                dir: globalInstallPath,
                name: "setup",
                ext: setupScriptExt,
            });
            env = await ros_utils.sourceSetupFile(setupScript, {});
        } catch (err) {
            vscode.window.showErrorMessage(`Could not source the setup file for ROS distro "${distro}".`);
        }
    } else if (typeof process.env.ROS_ROOT !== "undefined") {
        env = process.env;
    } else {
        const message = "The ROS distro is not configured.";
        const configure = "Configure";

        if (await vscode.window.showErrorMessage(message, configure) === configure) {
            config.update("distro", await vscode.window.showQuickPick(ros_utils.getDistros()));
        }
    }

    // Source the workspace setup over the top.
    let workspaceDevelPath: string;
    workspaceDevelPath = path.join(`${baseDir}`, "devel_isolated");
    if (!await pfs.exists(workspaceDevelPath)) {
        workspaceDevelPath = path.join(`${baseDir}`, "devel");
    }
    let wsSetupScript: string = path.format({
        dir: workspaceDevelPath,
        name: "setup",
        ext: setupScriptExt,
    });

    if (env && typeof env.ROS_ROOT !== "undefined" && await pfs.exists(wsSetupScript)) {
        try {
            env = await ros_utils.sourceSetupFile(wsSetupScript, env);
        } catch (_err) {
            vscode.window.showErrorMessage("Failed to source the workspace setup file.");
        }
    }

    // Notify listeners the environment has changed.
    onEnvChanged.fire();
}
