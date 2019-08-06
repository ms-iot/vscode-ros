// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// copied from https://github.com/microsoft/vscode-cpptools

/* tslint:disable */

import * as process_item from "./process-entry";
import * as attach_items_provider from "./process-items-provider";

export abstract class LocalProcessQuickPickItemsProvider implements attach_items_provider.IProcessQuickPickItemsProvider {
    public getItems(): Promise<process_item.IProcessQuickPickItem[]> {
        return this.getInternalProcessEntries().then((processEntries) => {
            // localeCompare is significantly slower than < and > (2000 ms vs 80 ms for 10,000 elements)
            // We can change to localeCompare if this becomes an issue
            processEntries.sort((a, b) => {
                if (a.name === undefined) {
                    if (b.name === undefined) {
                        return 0;
                    }
                    return 1;
                }
                if (b.name === undefined) {
                    return -1;
                }
                const aLower: string = a.name.toLowerCase();
                const bLower: string = b.name.toLowerCase();
                if (aLower === bLower) {
                    return 0;
                }
                return aLower < bLower ? -1 : 1;
            });
            return processEntries.map((p) => process_item.createProcessQuickPickItem(p));
        });
    }

    protected abstract getInternalProcessEntries(): Promise<process_item.ProcessEntry[]>;
}
