// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as _ from "underscore";
import * as vscode from "vscode";
import * as xmlrpc from "xmlrpc";
import * as path from "path";

import * as extension from "./extension";

export function startCore() {
    let newProcessOptions = {
        cwd: extension.baseDir,
        env: extension.env,
        shell: "cmd",
        windowsHide: false
    };

    const masterProcess = child_process.spawn("roscore", [], newProcessOptions);

    masterProcess.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });
    masterProcess.stderr.on('data', (data) => {
        console.log(`stderr: ${data}`);
    });
    masterProcess.on('close', (code) => {
        console.log(`child process exited with code ${code}`);
    });
}

export function stopCore(api: XmlRpcApi) {
    if (process.platform === "win32") {
        api.getPid().then(pid => child_process.exec(`taskkill /pid ${pid} /f`));
    }
    else {
        api.getPid().then(pid => child_process.exec(`kill $(ps -o ppid= -p '${pid}')`));
    }
}

export function launchMonitor(context: vscode.ExtensionContext) {
    const panel = vscode.window.createWebviewPanel(
        "rosMasterStatus",
        "ROS Master Status",
        vscode.ViewColumn.Two,
        {
            enableScripts: true,
        }
    );

    let stylesheet = vscode.Uri.file(path.join(context.extensionPath, "assets", "masterMonitorStyle.css")).with({
        scheme: "vscode-resource"
    });
    let script = vscode.Uri.file(path.join(context.extensionPath, "assets", "masterMonitor.js")).with({
        scheme: "vscode-resource"
    });

    panel.webview.html = getMasterStatusWebviewContent(stylesheet, script);

    setInterval(() => {
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
}

function getMasterStatusWebviewContent(stylesheet: vscode.Uri, script: vscode.Uri): string {
    return `
<!DOCTYPE html>
<html lang="en">

<head>
    <link rel="stylesheet" href="${stylesheet.toString()}" />

    <script src="${script.toString()}"></script>
</head>

<body>
    <h1>ROS Master Status</h1>
    <h2 id="master-status">-</h2>

    <div id="parameters"></div>
    <div id="topics"></div>
    <div id="services"></div>
</body>

</html>
`;
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
        return this.methodCall("getSystemState").then(res => <ISystemState>{
            publishers: _.object(res[0]),
            services: _.object(res[2]),
            subscribers: _.object(res[1]),
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

interface ISystemState {
    publishers: { [topic: string]: string[] };
    subscribers: { [topic: string]: string[] };
    services: { [service: string]: string[] };
}

/**
 * Shows the ROS master status in the status bar.
 */
export class StatusBarItem {
    private item: vscode.StatusBarItem;
    private timer: NodeJS.Timer;
    private status: boolean;

    public constructor(private api: XmlRpcApi) {
        this.item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 200);
        this.item.text = "$(question) ROS master";
        this.item.command = extension.Commands.ShowMasterStatus;
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

        this.item.text = (status ? "$(check)" : "$(x)") + " ROS master";
        this.status = status;
    }
}
