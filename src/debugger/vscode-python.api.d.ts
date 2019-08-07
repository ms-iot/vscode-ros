// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-azurefunctions

interface IPythonExtensionApi {
	debug: {
		/**
		 * Generate an array of strings for commands to pass to the Python executable to launch the debugger for remote debugging.
		 * Users can append another array of strings of what they want to execute along with relevant arguments to Python.
		 * E.g `['/Users/..../pythonVSCode/pythonFiles/experimental/ptvsd_launcher.py', '--host', 'localhost', '--port', '57039', '--wait']`
		*/
		getRemoteLauncherCommand(host: string, port: number, waitUntilDebuggerAttaches?: boolean): Promise<string[]>
	}
}
