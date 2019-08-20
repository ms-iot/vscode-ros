// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as adapter from "vscode-debugadapter";
import * as protocol from "vscode-debugprotocol";

import * as requests from "./requests";

export class RosDebugSession extends adapter.DebugSession {
    public shutdown() {
        super.shutdown();
    }

    protected launchRequest(response: protocol.DebugProtocol.LaunchResponse, request: requests.ILaunchRequest): void {
        // launch requests are propagated to runtime-specific debugger extensions,
        // this is only a proxy session, self-terminate immediately
        this.shutdown();
    }

    protected attachRequest(response: protocol.DebugProtocol.AttachResponse, request: requests.IAttachRequest): void {
        // attach requests are propagated to runtime-specific debugger extensions,
        // this is only a proxy session, self-terminate immediately
        this.shutdown();
    }
}
