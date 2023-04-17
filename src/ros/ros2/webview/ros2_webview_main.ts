// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

namespace ros2monitor {
    function removeAllChildElements(e) {
        while (e.firstChild) {
            e.removeChild(e.firstChild);
        }
    };

    function generateColumnTable(dataArray: any, headers: string[], callback: (data: any, i: number) => string) {
        let t = document.createElement("table");
        let th = document.createElement("thead");
        let headerRow = document.createElement("tr");
        headers.forEach((name, _i) => {
            let h = document.createElement("th");
            h.appendChild(document.createTextNode(name));
            headerRow.appendChild(h);
        });

        th.appendChild(headerRow);
        t.appendChild(th);

        let tb = document.createElement("tbody");
        for (const i in dataArray) {
            const data = dataArray[i];
            const r = document.createElement("tr");
            headers.forEach((name, _i) => {
                let cell = document.createElement("td");
                cell.appendChild(document.createTextNode(callback(data, _i)));
                r.appendChild(cell);
            });
            tb.appendChild(r);
        }
        t.appendChild(tb);

        return t;
    }

    export function initializeRos2Monitor() {
        // handle message passed from extension to webview
        window.addEventListener("message", (event) => {
            const message = event.data;

            const coreStatus = document.getElementById("ros-status");
            const topicsElement = document.getElementById("topics");
            const servicesElement = document.getElementById("services");

            removeAllChildElements(topicsElement);
            removeAllChildElements(servicesElement);

            if (message.ready) {
                coreStatus.textContent = "online";

                const nodes = JSON.parse(message.nodes);
                const topics = JSON.parse(message.topics);
                const services = JSON.parse(message.services);

                const nodesHeader = document.createElement("h2");
                nodesHeader.appendChild(document.createTextNode("Nodes"));
                topicsElement.appendChild(nodesHeader);
                topicsElement.appendChild(generateColumnTable(nodes, ["Name"], (data, i) => {
                    return `${data[1]}${data[0]}`;
                }));

                const topicsHeader = document.createElement("h2");
                topicsHeader.appendChild(document.createTextNode("Topics"));
                topicsElement.appendChild(topicsHeader);
                topicsElement.appendChild(generateColumnTable(topics, ["Name", "Type"], (data, i) => {
                    return data[i];
                }));

                const servicesHeader = document.createElement("h2");
                servicesHeader.appendChild(document.createTextNode("Services"));
                servicesElement.appendChild(servicesHeader);
                servicesElement.appendChild(generateColumnTable(services, ["Name", "Type"], (data, i) => {
                    return data[i];
                }));
            }
            else {
                coreStatus.textContent = "offline";
            }
        });
    };
}

window.onload = () => ros2monitor.initializeRos2Monitor();
