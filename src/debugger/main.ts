// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as adapter from "vscode-debugadapter";

import * as session from "./debug-session";

adapter.DebugSession.run(session.RosDebugSession);
