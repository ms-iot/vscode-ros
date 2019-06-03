// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as build from "./build";
import * as catkin from "./catkin";
import CatkinTaskProvider from "./catkin-task-provider";
import CppFormatter from "./cpp-formatter";
import * as debug from "./debug";
import * as master from "./master";
import * as pfs from "./promise-fs";
import * as utils from "./utils";

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
}

export async function activate(context: vscode.ExtensionContext) {
    // Activate if we're in a catkin workspace.
    await determineBuildSystem(vscode.workspace.rootPath);

    if (buildSystem == BuildSystem.None) {
        return;
    }

    console.log(`Activating ROS extension in "${baseDir}"`);

    // Activate components when the ROS env is changed.
    context.subscriptions.push(onDidChangeEnv(activateEnvironment.bind(null, context)));

    // Activate components which don't require the ROS env.
    context.subscriptions.push(vscode.languages.registerDocumentFormattingEditProvider(
        "cpp", new CppFormatter()
    ));

    // Source the environment, and re-source on config change.
    let config = utils.getConfig();

    context.subscriptions.push(vscode.workspace.onDidChangeConfiguration(() => {
        const updatedConfig = utils.getConfig();
        const fields = Object.keys(config).filter(k => !(config[k] instanceof Function));
        const changed = fields.some(key => updatedConfig[key] !== config[key]);

        if (changed) {
            sourceRosAndWorkspace();
        }

        config = updatedConfig;
    }));

    sourceRosAndWorkspace();

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
    const masterApi = new master.XmlRpcApi(env.ROS_MASTER_URI);
    const coreStatusItem = new master.StatusBarItem(masterApi);

    coreStatusItem.activate();

    subscriptions.push(coreStatusItem);
    subscriptions.push(vscode.workspace.registerTaskProvider("catkin", new CatkinTaskProvider()));
    subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", new debug.RosDebugConfigProvider()));

    // register plugin commands
    subscriptions.push(
        vscode.commands.registerCommand(Commands.CreateCatkinPackage, catkin.createPackage),
        vscode.commands.registerCommand(Commands.CreateTerminal, utils.createTerminal),
        vscode.commands.registerCommand(Commands.GetDebugSettings, debug.getDebugSettings),
        vscode.commands.registerCommand(Commands.ShowCoreStatus, () => { master.launchMonitor(context) }),
        vscode.commands.registerCommand(Commands.StartRosCore, master.startCore),
        vscode.commands.registerCommand(Commands.TerminateRosCore, () => { master.stopCore(masterApi) }),
        vscode.commands.registerCommand(Commands.UpdateCppProperties, build.updateCppProperties),
        vscode.commands.registerCommand(Commands.UpdatePythonPath, build.updatePythonPath),
        vscode.commands.registerCommand(Commands.Rosrun, rosrundelegate),
        vscode.commands.registerCommand(Commands.Roslaunch, roslaunchdelegate),
    );

    // Generate config files if they don't already exist.
    build.createConfigFiles();
}

async function rosrundelegate() {
    let terminal = await preparerosrun();
    terminal.show();
}

async function preparerosrun(): Promise<vscode.Terminal> {
    const packages = utils.getPackages();
    const packageName = await vscode.window.showQuickPick(packages.then(Object.keys), { placeHolder: "Choose a package" });
    if (packageName !== undefined) {
        let basenames = (files: string[]) => files.map(file => path.basename(file));

        const executables = utils.findPackageExecutables(packageName).then(basenames);
        let target = await vscode.window.showQuickPick(executables, { placeHolder: "Choose an executable" });
        let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        let terminal = vscode.window.createTerminal({
            name: "rosrun",
            env: env
        });
        terminal.sendText(`rosrun ${packageName} ${target} ${argument}`);
        return terminal;
    } else {
        // none of the packages selected, error!
    }
}

async function roslaunchdelegate() {
    let terminal = await prepareroslaunch();
    terminal.show();
}

async function prepareroslaunch(): Promise<vscode.Terminal> {
    const packages = utils.getPackages();
    const packageName = await vscode.window.showQuickPick(packages.then(Object.keys), { placeHolder: "Choose a package" });
    if (packageName !== undefined) {
        const launchFiles = await utils.findPackageLaunchFiles(packageName);
        const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
        let target = await vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
        let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        let terminal = vscode.window.createTerminal({
            name: "roslaunch",
            env: env
        });
        terminal.sendText(`roslaunch ${launchFiles[launchFileBasenames.indexOf(target)]} ${argument}`);
        return terminal;
    } else {
        // none of the packages selected, error!
    }
}

/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
async function sourceRosAndWorkspace(): Promise<void> {
    env = undefined;

    const config = utils.getConfig();
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
            env = await utils.sourceSetupFile(setupScript, {});
        } catch (err) {
            vscode.window.showErrorMessage(`Could not source the setup file for ROS distro "${distro}".`);
        }
    } else if (typeof process.env.ROS_ROOT !== "undefined") {
        env = process.env;
    } else {
        const message = "The ROS distro is not configured.";
        const configure = "Configure";

        if (await vscode.window.showErrorMessage(message, configure) === configure) {
            config.update("distro", await vscode.window.showQuickPick(utils.getDistros()));
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
            env = await utils.sourceSetupFile(wsSetupScript, env);
        } catch (_err) {
            vscode.window.showErrorMessage("Failed to source the workspace setup file.");
        }
    }

    // Notify listeners the environment has changed.
    onEnvChanged.fire();
}
