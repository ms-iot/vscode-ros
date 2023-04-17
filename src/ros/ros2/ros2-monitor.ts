// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import * as xmlrpc from "xmlrpc";

import * as extension from "../../extension";
import * as telemetry from "../../telemetry-helper";

function getDaemonPort() {
    let basePort: number = 11511;
    basePort += Number(process.env.ROS_DOMAIN_ID) || 0
    return basePort;
}

function getDaemonUri() {
    return `http://localhost:${getDaemonPort()}/ros2cli/`;
}

export function launchMonitor(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.ShowCoreStatus);

    const panel = vscode.window.createWebviewPanel(
        "ros2Status",
        "ROS2 Status",
        vscode.ViewColumn.Two,
        {
            enableScripts: true,
        }
    );

    const stylesheet = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "assets", "ros", "core-monitor", "style.css")));

    const script = panel.webview.asWebviewUri(vscode.Uri.file(path.join(context.extensionPath, "dist", "ros2_webview_main.js")));

    panel.webview.html = getCoreStatusWebviewContent(stylesheet, script);

    const ros2cliApi = new XmlRpcApi();
    const pollingHandle = setInterval(async () => {
        try {
            const result: any[] = await Promise.all([
                ros2cliApi.getNodeNamesAndNamespaces(), 
                ros2cliApi.getTopicNamesAndTypes(), 
                ros2cliApi.getServiceNamesAndTypes()]);
            const nodesJSON = JSON.stringify(result[0]);
            const topicsJSON = JSON.stringify(result[1]);
            const servicesJSON = JSON.stringify(result[2]);
            panel.webview.postMessage({
                ready: true,
                nodes: nodesJSON,
                topics: topicsJSON,
                services: servicesJSON,
            });
        } catch (e) {
            panel.webview.postMessage({
                ready: false,
            });
        }
    }, 200);

    panel.onDidDispose(() => {
        clearInterval(pollingHandle);
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
    <h1>ROS2 System Status</h1>
    <h2 id="ros-status">-</h2>

    <div id="parameters"></div>
    <div id="topics"></div>
    <div id="services"></div>
</body>

</html>
`;
}

/**
 * ros2cli xmlrpc interfaces.
 */
export class XmlRpcApi {
    private client: xmlrpc.Client;

    public constructor() {
        this.client = xmlrpc.createClient(getDaemonUri());
    }

    public check() : Promise<boolean> {
        // the ROS2 CLI doesn't have an API which returns detailed status, 
        // so we're just using another endpoint to verify it is running
        return this.methodCall("get_node_names_and_namespaces").then(() => true, () => false);
    }

    public getNodeNamesAndNamespaces() : Promise<any> {
        return this.methodCall("get_node_names_and_namespaces");
    }

    public getServiceNamesAndTypes() : Promise<any> {
        return this.methodCall("get_service_names_and_types");
    }

    public getTopicNamesAndTypes() : Promise<any> {
        return this.methodCall("get_topic_names_and_types");
    }

    private methodCall(method: string, ...args: any[]): Promise<any> {
        return new Promise((resolve, reject) => {
            this.client.methodCall(method, [...args], (err, val) => {
                if (err) {
                    reject(err);
                } else {
                    resolve(val);
                }
            });
        });
    }
}
