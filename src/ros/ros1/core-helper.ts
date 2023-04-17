// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as path from "path";
import * as vscode from "vscode";
import * as xmlrpc from "xmlrpc";

import * as extension from "../../extension";
import * as telemetry from "../../telemetry-helper";

export function startCore(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.StartRosCore);

    let launchCoreCommand: string = "roscore";
    let processOptions: child_process.SpawnOptions = {
        cwd: extension.baseDir,
        env: extension.env,
    };

    const roscoreProcess = child_process.spawn(launchCoreCommand, processOptions);
    roscoreProcess.on('error', (_err) => {
        vscode.window.showErrorMessage("Failed to launch ROS core.");
    });
}

export function stopCore(context: vscode.ExtensionContext, api: XmlRpcApi) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.TerminateRosCore);

    if (process.platform === "win32") {
        api.getPid().then(pid => child_process.exec(`taskkill /pid ${pid} /f`));
    }
    else {
        api.getPid().then(pid => child_process.exec(`kill $(ps -o ppid= -p '${pid}')`));
    }
}

export function launchMonitor(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.ShowCoreStatus);

    const panel = vscode.window.createWebviewPanel(
        "rosCoreStatus",
        "ROS Core Status",
        vscode.ViewColumn.Two,
        {
            enableScripts: true,
        }
    );

    let stylesheet = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "assets", "ros", "core-monitor", "style.css")));
    let script = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "dist", "ros1_webview_main.js")));

    panel.webview.html = getCoreStatusWebviewContent(stylesheet, script);

    const pollingStatus = setInterval(() => {
        const masterApi = new XmlRpcApi(extension.env.ROS_MASTER_URI);
        masterApi.check().then((status: boolean) => {
            if (status) {
                let getParameters = masterApi.getParam("/");
                let getSystemState = masterApi.getSystemState();

                Promise.all([getParameters, getSystemState]).then(([parameters, systemState]) => {
                    let parametersJSON = JSON.stringify(parameters);
                    let systemStateJSON = JSON.stringify(systemState);

                    panel.webview.postMessage({
                        status: status,
                        parameters: parametersJSON,
                        systemState: systemStateJSON,
                    });
                });
            }
            else {
                panel.webview.postMessage({
                    status: status,
                });
            }
        });
    }, 100);
    panel.onDidDispose(() => {
        clearInterval(pollingStatus);
    });
}

function getCoreStatusWebviewContent(stylesheet: vscode.Uri, script: vscode.Uri): string {
    return `
<!DOCTYPE html>
<html lang="en">

<head>
    <link rel="stylesheet" href="${stylesheet.toString()}" />

    <script src="${script.toString()}"></script>
</head>

<body>
    <h1>ROS Core Status</h1>
    <h2 id="ros-status">-</h2>

    <div id="parameters"></div>
    <div id="topics"></div>
    <div id="services"></div>
</body>

</html>
`;
}

interface ISystemState {
    publishers: { [topic: string]: string[] };
    subscribers: { [topic: string]: string[] };
    services: { [service: string]: string[] };
}

const CALLER_ID = "vscode-ros";

/**
 * Exposes the ROS master XML-RPC api.
 */
export class XmlRpcApi {
    private client: xmlrpc.Client;

    public constructor(uri: string) {
        this.client = xmlrpc.createClient(uri);
    }

    /**
     * Returns true if a master process is running.
     */
    public check(): Promise<boolean> {
        return this.getPid().then(() => true, () => false);
    }

    public getPid(): Promise<number> {
        return this.methodCall("getPid");
    }

    public getSystemState(): Promise<ISystemState> {
        const responseReducer = (acc: object, cur: any[]) => {
            const k: string = cur[0] as string;
            const v: string[] = cur[1] as string[];
            acc[k] = v;
            return acc;
        };
        return this.methodCall("getSystemState").then((res) => {
            const systemState: ISystemState = {
                publishers: res[0].reduce(responseReducer, {}),
                services: res[2].reduce(responseReducer, {}),
                subscribers: res[1].reduce(responseReducer, {}),
            };
            return systemState;
        });
    }

    public getParamNames(): Promise<string[]> {
        return this.methodCall("getParamNames");
    }

    public getParam(name: string): Promise<any> {
        return this.methodCall("getParam", name);
    }

    private methodCall(method: string, ...args: any[]): Promise<any> {
        return new Promise((resolve, reject) => {
            this.client.methodCall(method, [CALLER_ID, ...args], (err, val) => {
                if (err) {
                    reject(err);
                } else if (val[0] !== 1) {
                    reject(val);
                } else {
                    resolve(val[2]);
                }
            });
        });
    }
}

/**
 * Shows the ROS core status in the status bar.
 */
// tslint:disable-next-line: max-classes-per-file
export class StatusBarItem {
    private item: vscode.StatusBarItem;
    private timer: NodeJS.Timer;
    private status: boolean;

    public constructor(private api: XmlRpcApi) {
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);

        const waitIcon = "$(clock)";
        const ros = "ROS";
        this.item.text = `${waitIcon} ${ros}`;
        this.item.command = extension.Commands.ShowCoreStatus;
    }

    public activate() {
        this.item.show();
        this.timer = setInterval(() => this.update(), 200);
    }

    public dispose() {
        this.item.dispose();
    }

    private async update() {
        const status = await this.api.check();

        if (status === this.status) {
            return;
        }

        const statusIcon = status ? "$(check)" : "$(x)";
        let ros = "ROS";

        // these environment variables are set by the ros_environment package
        // https://github.com/ros/ros_environment
        const rosVersionChecker = "ROS_VERSION";
        const rosDistroChecker = "ROS_DISTRO";
        if (rosVersionChecker in extension.env && rosDistroChecker in extension.env) {
            const rosVersion: string = extension.env[rosVersionChecker];
            const rosDistro: string = extension.env[rosDistroChecker];
            ros += `${rosVersion}.${rosDistro}`;
        } else {
            // for older ROS1 installations with outdated ros_environment
            // "rosversion --distro" might be needed
            // ignoring such case for now
        }
        this.item.text = `${statusIcon} ${ros}`;
        this.status = status;
    }
}
