// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as adapter from "vscode-debugadapter";
import * as protocol from "vscode-debugprotocol";

interface ILaunchRequestArguments extends protocol.DebugProtocol.LaunchRequestArguments {
    command: "roslaunch" | "rosrun";
    package: string;
    target: string;
    args: string[];
    debugSettings: string;
}

export class RosDebugSession extends adapter.DebugSession {
    private process: child_process.ChildProcess;

    public shutdown() {
        if (this.process) {
            this.process.kill();
        }

        super.shutdown();
    }

    protected launchRequest(response: protocol.DebugProtocol.LaunchResponse, request: ILaunchRequestArguments) {
        if (request.command !== "roslaunch" && request.command !== "rosrun") {
            this.sendErrorResponse(response, 0, "Invalid command");
            return;
        }

        // Merge the ROS env with the current env so we aren't running in headless mode.
        const settings = JSON.parse(request.debugSettings);
        const env = Object.assign(process.env, settings.env || process.env);
        const args = [request.package, request.target].concat(request.args || []);

        this.process = child_process.spawn(request.command, args, { env });

        this.process.stdout.on("data", chunk =>
            this.sendEvent(new adapter.OutputEvent(chunk.toString(), "stdout"))
        );
        this.process.stderr.on("data", chunk =>
            this.sendEvent(new adapter.OutputEvent(chunk.toString(), "stderr"))
        );

        this.process.on("error", (err: Error) => {
            this.sendEvent(new adapter.OutputEvent(err.message, "stderr"));
            this.sendEvent(new adapter.TerminatedEvent());
        });
        this.process.on("exit", () => this.sendEvent(new adapter.TerminatedEvent()));

        this.sendResponse(response);
    }
}
