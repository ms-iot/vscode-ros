/* tslint:disable */

import * as os from "os";

import * as process_item from "./process-entry";
import * as ps_provider from "./process-quick-pick-items-provider-ps";
import * as wmic_provider from "./process-quick-pick-items-provider-wmic";

export interface IProcessQuickPickItemsProvider {
    getItems(): Promise<process_item.IProcessQuickPickItem[]>;
}

export class NativeAttachItemsProviderFactory {
    static Get(): IProcessQuickPickItemsProvider {
        if (os.platform() === 'win32') {
            return new wmic_provider.WmicProvider();
        } else {
            return new ps_provider.PsProvider();
        }
    }
}
