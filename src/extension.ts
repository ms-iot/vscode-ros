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
import * as platformHelper from "./platformHelper"

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

export enum RosCommands {
    CreateCatkinPackage = "ros.createCatkinPackage",
    CreateTerminal = "ros.createTerminal",
    GetDebugSettings = "ros.getDebugSettings",
    Rosrun = "ros.rosrun",
    Roslaunch = "ros.roslaunch",
    ShowMasterStatus = "ros.showMasterStatus",
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

    await platformHelper.setup();

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
  const masterStatusItem = new master.StatusBarItem(masterApi);
  const masterStatusProvider = new master.StatusDocumentProvider(context, masterApi);

  masterStatusItem.activate();

  subscriptions.push(masterStatusItem);
  subscriptions.push(vscode.workspace.registerTextDocumentContentProvider("ros-master", masterStatusProvider));
  subscriptions.push(vscode.workspace.registerTaskProvider("catkin", new CatkinTaskProvider()));
  subscriptions.push(vscode.debug.registerDebugConfigurationProvider("ros", new debug.RosDebugConfigProvider()));

    // register plugin commands
  subscriptions.push(
        vscode.commands.registerCommand(RosCommands.CreateCatkinPackage, catkin.createPackage),
        vscode.commands.registerCommand(RosCommands.CreateTerminal, utils.createTerminal),
        vscode.commands.registerCommand(RosCommands.GetDebugSettings, debug.getDebugSettings),
        vscode.commands.registerCommand(RosCommands.ShowMasterStatus, () => { master.showMasterStatus() }),
        vscode.commands.registerCommand(RosCommands.StartRosCore, master.launchCore),
        vscode.commands.registerCommand(RosCommands.TerminateRosCore, () => { master.terminateCore(masterApi) }),
        vscode.commands.registerCommand(RosCommands.UpdateCppProperties, build.updateCppProperties),
        vscode.commands.registerCommand(RosCommands.UpdatePythonPath, build.updatePythonPath)
  );

  // Generate config files if they don't already exist.
  build.createConfigFiles();
}

/**
 * Loads the ROS environment, and prompts the user to select a distro if required.
 */
async function sourceRosAndWorkspace(): Promise<void> {
  env = undefined;

  const config = utils.getConfig();
  const distro = config.get("distro", "");

  if (distro) {
    try {
            let setupScript: string;
            if (process.platform === "win32") {
                setupScript = `C:\\opt\\ros\\${distro}\\x64\\setup.bat`;
            }
            else {
                setupScript = `/opt/ros/${distro}/setup.bash`;
            }
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
    let wsSetupScript: string;
    if (process.platform === "win32") {
        wsSetupScript = `${baseDir}\\devel_isolated\\setup.bat`;
    }
    else {
        wsSetupScript = `${baseDir}/devel/setup.bash`;
    }

    if (env && typeof env.ROS_ROOT !== "undefined" && await pfs.exists(wsSetupScript)) {
    try {
            env = await utils.sourceSetupFile(wsSetupScript, env);
    } catch (err) {
      vscode.window.showWarningMessage("Could not source the workspace setup file.");
    }
  }

  // Notify listeners the environment has changed.
  onEnvChanged.fire();
}
