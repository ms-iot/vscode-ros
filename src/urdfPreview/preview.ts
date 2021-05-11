// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as path from 'path';
import { rosApi } from '../ros/ros';
import { xacro } from '../ros/utils'; 
import { Disposable, window } from 'vscode';

export default class URDFPreview 
{
    private _resource: vscode.Uri;
    private _processing: boolean;
    private  _context: vscode.ExtensionContext;
    private _disposable: Disposable;
    _urdfEditor: vscode.TextEditor;
    _webview: vscode.WebviewPanel;

    public get state() {
        return {
            resource: this.resource.toString()
        };
    }

    public static create(
        context: vscode.ExtensionContext,
        resource: vscode.Uri
        ) : URDFPreview
    {
        // Create and show a new webview
        var editor = vscode.window.createWebviewPanel(
            'urdfPreview', // Identifies the type of the webview. Used internally
            'URDF Preview', // Title of the panel displayed to the user
            vscode.ViewColumn.Two, // Editor column to show the new webview panel in.
            { 
                enableScripts: true,
                retainContextWhenHidden: true
            }
        );

        return new URDFPreview(editor, context, resource);
    } 

    private constructor(
        webview: vscode.WebviewPanel,
        context: vscode.ExtensionContext,
        resource: vscode.Uri
    )
    {
        this._webview = webview;
        this._context = context;
        this._resource = resource;
        this._processing = false;

        let subscriptions: Disposable[] = [];

        const templateFilename = this._context.asAbsolutePath("templates/preview.html");
        vscode.workspace.openTextDocument(templateFilename).then(doc => {
            var previewText = doc.getText();
            this._webview.webview.html = previewText;

            setTimeout(() => this.refresh(), 1000);
        });

        this._webview.onDidChangeViewState(e => {
            if (e.webviewPanel.active) {
                setTimeout(() => this.refresh(), 1000);
            }
            this._onDidChangeViewStateEmitter.fire(e);
        }, this, subscriptions);

        vscode.workspace.onDidSaveTextDocument(event => {

            if (event && this.isPreviewOf(event.uri)) {
                this.refresh();
            }
        }, this, subscriptions);

        this._webview.onDidDispose(() => {
            this.dispose();
        }, null, subscriptions);        

        this._disposable = Disposable.from(...subscriptions);
    }

    public get resource(): vscode.Uri {
        return this._resource;
    }

    public async refresh() {
        if (this._processing == false) {
            this.loadResource();
        }
    }

    private async loadResource() {
        this._processing = true;

        var urdfText;
        let ext = path.extname(this._resource.fsPath);
        if (ext == ".xacro") {
            try {
                urdfText = await xacro(this._resource.fsPath);
            } catch (err) {
                vscode.window.showErrorMessage(err.message);
            }
        } else {
            // at this point, the text document could have changed
            var doc = await vscode.workspace.openTextDocument(this._resource.fsPath);
            urdfText = doc.getText();
        }

        var packageMap = await rosApi.getPackages();

        // replace package://(x) with fully resolved paths
        var pattern =  /package:\/\/(.*?)\//g;
        var match;
        while (match = pattern.exec(urdfText)) {
            var packagePath = await packageMap[match[1]]();
            if (packagePath.charAt(0)  === '/') {
                // inside of mesh re \source, the loader attempts to concatinate the base uri with the new path. It first checks to see if the
                // base path has a /, if not it adds it.
                // We are attempting to use a protocol handler as the base path - which causes this to fail.
                // basepath - vscode-webview-resource:
                // full path - /home/test/ros
                // vscode-webview-resource://home/test/ros.
                // It should be vscode-webview-resource:/home/test/ros.
                // So remove the first char.

                packagePath = packagePath.substr(1);
            }
            let normPath = path.normalize(packagePath);
            let vsPath = vscode.Uri.file(normPath);
            let newUri = this._webview.webview.asWebviewUri(vsPath);
            let hackThePath = newUri.toString().replace('https:', '');

            // HACKHACK - the RosWebTools will alwayse prefix the paths with a '/' if we don't pass a prefix.
            // to workaround this without changing RWT, we are stripping off the known protocol, and passing the
            // resulting path into RWT with that known prefix as an option. Internally it will see that there is a prefix
            // and combine them. 
            urdfText = urdfText.replace('package://' + match[1], hackThePath);
        }

        var previewFile = this._resource.toString();

        console.log("URDF previewing: " + previewFile);

        this._webview.webview.postMessage({ command: 'previewFile', previewFile: previewFile});
        this._webview.webview.postMessage({ command: 'urdf', urdf: urdfText });

        this._processing = false;
    }

    public static async revive(
        webview: vscode.WebviewPanel,
        context: vscode.ExtensionContext,
        state: any,
    ): Promise<URDFPreview> {
        const resource = vscode.Uri.parse(state.previewFile);

        const preview = new URDFPreview(
            webview,
            context,
            resource);

        return preview;
    }    

    public matchesResource(
        otherResource: vscode.Uri
    ): boolean {
        return this.isPreviewOf(otherResource);
    }

    public reveal() {
        this._webview.reveal(vscode.ViewColumn.Two);
    }    

    private isPreviewOf(resource: vscode.Uri): boolean {
        return this._resource.fsPath === resource.fsPath;
    }

    private readonly _onDisposeEmitter = new vscode.EventEmitter<void>();
    public readonly onDispose = this._onDisposeEmitter.event;    
    
    private readonly _onDidChangeViewStateEmitter = new vscode.EventEmitter<vscode.WebviewPanelOnDidChangeViewStateEvent>();
    public readonly onDidChangeViewState = this._onDidChangeViewStateEmitter.event;

    public update(resource: vscode.Uri) {
        const editor = vscode.window.activeTextEditor;

        // If we have changed resources, cancel any pending updates
        const isResourceChange = resource.fsPath !== this._resource.fsPath;
        this._resource = resource;
        // Schedule update if none is pending
        this.refresh();
    }
    
    public dispose() {
        this._disposable.dispose();
        this._onDisposeEmitter.fire();
        this._onDisposeEmitter.dispose();

        this._onDidChangeViewStateEmitter.dispose();
        this._webview.dispose();    
    }
}
