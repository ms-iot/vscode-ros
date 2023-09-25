// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as cpp_formatter from "./cpp-formatter";
import * as pfs from "./promise-fs";
import * as telemetry from "./telemetry-helper";
import * as vscode_utils from "./vscode-utils";

import * as buildtool from "./build-tool/build-tool";

import * as ros_build_utils from "./ros/build-env-utils";
import * as ros_cli from "./ros/cli";
import * as ros_utils from "./ros/utils";
import { rosApi, selectROSApi } from "./ros/ros";
import URDFPreviewManager from "./urdfPreview/previewManager"

import * as debug_manager from "./debugger/manager";
import * as debug_utils from "./debugger/utils";
import { registerRosShellTaskProvider } from "./build-tool/ros-shell";

/**
 * The catkin workspace base dir.
 */
export let baseDir: string;

export function setBaseDir(dir: string) {
    baseDir = dir;
}

/**
 * The sourced ROS environment.
 */
export let env: any;
export let processingWorkspace = false;

export let extPath: string;
export let outputChannel: vscode.OutputChannel;

let onEnvChanged = new vscode.EventEmitter<void>();

/**
 * Triggered when the env is soured.
 */
export let onDidChangeEnv = onEnvChanged.event;

export async function resolvedEnv() {
    if (env === undefined) { // Env reload in progress
        await debug_utils.oneTimePromiseFromEvent(onDidChangeEnv, () => env !== undefined);
    }
    return env
}

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
    Rostest = "ros.rostest",
    Rosdep = "ros.rosdep",
    ShowCoreStatus = "ros.showCoreStatus",
    StartRosCore = "ros.startCore",
    TerminateRosCore = "ros.stopCore",
    UpdateCppProperties = "ros.updateCppProperties",
    UpdatePythonPath = "ros.updatePythonPath",
    PreviewURDF = "ros.previewUrdf",
}

export async function activate(context: vscode.ExtensionContext) {
    let init = vscode.window.withProgress({
        location: vscode.ProgressLocation.Notification,
        title: "ROS Extension Initializing...",
        cancellable: false
    }, async (progress, token) => {
        const reporter = telemetry.getReporter();
        extPath = context.extensionPath;
        outputChannel = vscode_utils.createOutputChannel();
        context.subscriptions.push(outputChannel);

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

        vscode.window.registerWebviewPanelSerializer('urdfPreview', URDFPreviewManager.INSTANCE);

        vscode.commands.registerCommand(Commands.CreateTerminal, () => {
            ensureErrorMessageOnException(() => {
                ros_utils.createTerminal(context);
            });
        });

        vscode.commands.registerCommand(Commands.GetDebugSettings, () => {
            ensureErrorMessageOnException(() => {
                return debug_utils.getDebugSettings(context);
            });
        });

        vscode.commands.registerCommand(Commands.ShowCoreStatus, () => {
            ensureErrorMessageOnException(() => {
                rosApi.showCoreMonitor();
            });
        });

        vscode.commands.registerCommand(Commands.StartRosCore, () => {
            ensureErrorMessageOnException(() => {
                rosApi.startCore();
            });
        });

        vscode.commands.registerCommand(Commands.TerminateRosCore, () => {
            ensureErrorMessageOnException(() => {
                rosApi.stopCore();
            });
        });

        vscode.commands.registerCommand(Commands.UpdateCppProperties, () => {
            ensureErrorMessageOnException(() => {
                return ros_build_utils.updateCppProperties(context);
            });
        });

        vscode.commands.registerCommand(Commands.UpdatePythonPath, () => {
            ensureErrorMessageOnException(() => {
                ros_build_utils.updatePythonPath(context);
            });
        });

        vscode.commands.registerCommand(Commands.Rosrun, () => {
            ensureErrorMessageOnException(() => {
                return ros_cli.rosrun(context);
            });
        });

        vscode.commands.registerCommand(Commands.Roslaunch, () => {
            ensureErrorMessageOnException(() => {
                return ros_cli.roslaunch(context);
            });
        });

        vscode.commands.registerCommand(Commands.Rostest, () => {
            ensureErrorMessageOnException(() => {
                return ros_cli.rostest(context);
            });
        });

        vscode.commands.registerCommand(Commands.PreviewURDF, () => {
            ensureErrorMessageOnException(() => {
                URDFPreviewManager.INSTANCE.preview(vscode.window.activeTextEditor.document.uri);
            });
        });

        vscode.commands.registerCommand(Commands.Rosdep, () => {
            ensureErrorMessageOnException(() => {
                rosApi.rosdep();
            });
        });


        reporter.sendTelemetryActivate();


        // Activate the workspace environment if possible.
        await activateEnvironment(context);

        return {
            getBaseDir: () => baseDir,
            getEnv: () => env,
            onDidChangeEnv: (listener: () => any, thisArg: any) => onDidChangeEnv(listener, thisArg),
        };
    });

    return await init;
}

export async function deactivate() {
    subscriptions.forEach(disposable => disposable.dispose());
    await telemetry.clearReporter();
}

async function ensureErrorMessageOnException(callback: (...args: any[]) => any) {
    try {
        await callback();
    } catch (err) {
        vscode.window.showErrorMessage(err.message);
    }
}

/**
 * Activates components which require a ROS env.
 */
async function activateEnvironment(context: vscode.ExtensionContext) {

    if (processingWorkspace) {
        return;
    }

    processingWorkspace = true;

    // Clear existing disposables.
    while (subscriptions.length > 0) {
        subscriptions.pop().dispose();
    }

    await sourceRosAndWorkspace();

    if (typeof env.ROS_DISTRO === "undefined") {
        processingWorkspace = false;
        return;
    }

    if (typeof env.ROS_VERSION === "undefined") {
        processingWorkspace = false;
        return;
    }

    // Determine if we're in a catkin workspace.
    let buildToolDetected = await buildtool.determineBuildTool(vscode.workspace.rootPath);

    // http://www.ros.org/reps/rep-0149.html#environment-variables
    // Learn more about ROS_VERSION definition.
    selectROSApi(env.ROS_VERSION);
    
    // Do this again, after the build tool has been determined.
    await sourceRosAndWorkspace();

    rosApi.setContext(context, env);

    subscriptions.push(rosApi.activateCoreMonitor());
    if (buildToolDetected) {
        subscriptions.push(...buildtool.BuildTool.registerTaskProvider());
    }
    subscriptions.push(...registerRosShellTaskProvider());

    debug_manager.registerRosDebugManager(context);

    // Register commands dependent on a workspace
    if (buildToolDetected) {
        subscriptions.push(
            vscode.commands.registerCommand(Commands.CreateCatkinPackage, () => {
                ensureErrorMessageOnException(() => {
                    return buildtool.BuildTool.createPackage(context);
                });
            }),
            vscode.tasks.onDidEndTask((event: vscode.TaskEndEvent) => {
                if (buildtool.isROSBuildTask(event.execution.task)) {
                    sourceRosAndWorkspace();
                }
            }),
        );
    }
    else {
        subscriptions.push(
            vscode.commands.registerCommand(Commands.CreateCatkinPackage, () => {
                vscode.window.showErrorMessage(`${Commands.CreateCatkinPackage} requires a ROS workspace to be opened`);
            }),
        );
    }

    // Generate config files if they don't already exist, but only for catkin workspaces
    if (buildToolDetected) {
        ros_build_utils.createConfigFiles();
    }

    processingWorkspace = false;
}

/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
async function sourceRosAndWorkspace(): Promise<void> {

    // Processing a new environment can take time which introduces a race condition. 
    // Wait to atomicly switch by composing a new environment block then switching at the end.
    let newEnv = undefined;

    const kWorkspaceConfigTimeout = 30000; // ms

    let setupScriptExt: string;
    if (process.platform === "win32") {
        setupScriptExt = ".bat";
    } else {
        setupScriptExt = ".bash";
    }

    const config = vscode_utils.getExtensionConfiguration();
    let isolateEnvironment = config.get("isolateEnvironment", "");
    if (!isolateEnvironment) {
        // Capture the host environment unless specifically isolated
        newEnv = process.env;
    }


    let rosSetupScript = config.get("rosSetupScript", "");

    // If the workspace setup script is not set, try to find the ROS setup script in the environment
    let attemptWorkspaceDiscovery = true;

    if (rosSetupScript) {
        // Try to support cases where the setup script doesn't make sense on different environments, such as host vs container.
        if (await pfs.exists(rosSetupScript)){
            try {
                newEnv = await ros_utils.sourceSetupFile(rosSetupScript, newEnv);

                attemptWorkspaceDiscovery = false;
            } catch (err) {
                vscode.window.showErrorMessage(`A ROS setup script was provided, but could not source "${rosSetupScript}". Attempting standard discovery.`);
            }
        }
    }
    
    if (attemptWorkspaceDiscovery) {
        let distro = config.get("distro", "");

        // Is there a distro defined either by setting or environment?
        if (!distro && !process.env.ROS_DISTRO)
        {
            // No? Try to find one.
            const installedDistros = await ros_utils.getDistros();
            if (!installedDistros.length) {
                throw new Error("ROS has not been found on this system.");
            } else if (installedDistros.length === 1) {
                
                // if there is only one distro installed, directly choose it
                config.update("distro", installedDistros[0]);
            } else {
                const message = "Unable to determine ROS distribution, please configure this workspace by adding \"ros.distro\": \"<ROS Distro>\" in settings.json";
                await vscode.window.setStatusBarMessage(message, kWorkspaceConfigTimeout);
            }
        }

        if (distro) {
            let setupScript: string;
            try {
                let globalInstallPath: string;
                if (process.platform === "win32") {
                    globalInstallPath = path.join("C:", "opt", "ros", `${distro}`, "x64");
                } else {
                    globalInstallPath = path.join("/", "opt", "ros", `${distro}`);
                }
                setupScript = path.format({
                    dir: globalInstallPath,
                    name: "setup",
                    ext: setupScriptExt,
                });
                newEnv = await ros_utils.sourceSetupFile(setupScript, newEnv);
            } catch (err) {
                vscode.window.showErrorMessage(`Could not source ROS setup script at "${setupScript}".`);
            }
        } else if (process.env.ROS_DISTRO) {
            newEnv = process.env;
        }
    }

    let workspaceOverlayPath: string = "";
    if (baseDir !== undefined) {
        // Source the workspace setup over the top.
        // TODO: we should test what's the build tool (catkin vs colcon).
        workspaceOverlayPath = path.join(`${baseDir}`, "devel_isolated");
        if (!await pfs.exists(workspaceOverlayPath)) {
            workspaceOverlayPath = path.join(`${baseDir}`, "devel");
        }
        if (!await pfs.exists(workspaceOverlayPath)) {
            workspaceOverlayPath = path.join(`${baseDir}`, "install");
        }
        let wsSetupScript: string = path.format({
            dir: workspaceOverlayPath,
            name: "setup",
            ext: setupScriptExt,
        });

        if (newEnv && typeof newEnv.ROS_DISTRO !== "undefined" && await pfs.exists(wsSetupScript)) {
            try {
                newEnv = await ros_utils.sourceSetupFile(wsSetupScript, newEnv);
            } catch (_err) {
                vscode.window.showErrorMessage("Failed to source the workspace setup file.");
            }
        }

    }

    env = newEnv;

    // Notify listeners the environment has changed.
    onEnvChanged.fire();
}
