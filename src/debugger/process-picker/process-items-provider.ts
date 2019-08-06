// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as os from "os";

import * as process_item from "./process-entry";
import * as ps_provider from "./process-items-provider-impl-ps";
import * as wmic_provider from "./process-items-provider-impl-wmic";

export interface IProcessQuickPickItemsProvider {
    getItems(): Promise<process_item.IProcessQuickPickItem[]>;
}

export class LocalProcessItemsProviderFactory {
    static Get(): IProcessQuickPickItemsProvider {
        if (os.platform() === 'win32') {
            return new wmic_provider.WmicProvider();
        } else {
            return new ps_provider.PsProvider();
        }
    }
}
