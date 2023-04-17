// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
function removeAllChildElements(e) {
    while (e.firstChild) {
        e.removeChild(e.firstChild);
    }
};

function generateParemetersUnorderedList(parameters) {
    let ul = document.createElement("ul");
    for (let p in parameters) {
        if (!parameters.hasOwnProperty(p)) {
            continue;
        }

        let li = document.createElement("li");
        let name = document.createElement("span");
        name.setAttribute("class", "name");
        name.appendChild(document.createTextNode(p.trim().toString() + ": "));
        li.appendChild(name);
        if (parameters[p] instanceof Object) {
            li.appendChild(generateParemetersUnorderedList(parameters[p]));
        }
        else {
            li.appendChild(document.createTextNode(parameters[p].toString().trim()));
        }
        ul.appendChild(li);
    }
    return ul;
}

function generateTopicsTable(publishers, subscribers) {
    let t = document.createElement("table");
    let th = document.createElement("thead");
    let headerRow = document.createElement("tr");
    let headers = [
        "Name",
        "Publishers",
        "Subscribers",
    ];
    headers.forEach((name, _i) => {
        let h = document.createElement("th");
        h.appendChild(document.createTextNode(name));
        headerRow.appendChild(h);
    });

    th.appendChild(headerRow);
    t.appendChild(th);

    let tb = document.createElement("tbody");
    let topics = Object.keys({
        ...publishers,
        ...subscribers,
    });
    console.debug(topics);
    for (let i in topics) {
        let topic = topics[i];
        let r = document.createElement("tr");
        let name = document.createElement("td");
        name.appendChild(document.createTextNode(topic));

        let publisher = document.createElement("td");
        publisher.appendChild(document.createTextNode(publishers.hasOwnProperty(topic) ? publishers[topic] : ""));

        let subscriber = document.createElement("td");
        subscriber.appendChild(document.createTextNode(subscribers.hasOwnProperty(topic) ? subscribers[topic] : ""));

        r.appendChild(name);
        r.appendChild(publisher);
        r.appendChild(subscriber);
        tb.appendChild(r);
    }
    t.appendChild(tb);

    return t;
}

function generateServicesTable(services) {
    let t = document.createElement("table");

    let th = document.createElement("thead");
    let headerRow = document.createElement("tr");
    let headers = [
        "Name",
        "Providers",
    ];
    headers.forEach((name, _i) => {
        let h = document.createElement("th");
        h.appendChild(document.createTextNode(name));
        headerRow.appendChild(h);
    });
    th.appendChild(headerRow);
    t.appendChild(th);

    let tb = document.createElement("tbody");
    for (let s in services) {
        if (!services.hasOwnProperty(s)) {
            continue;
        }

        let r = document.createElement("tr");
        let name = document.createElement("td");
        name.appendChild(document.createTextNode(s));

        let providers = document.createElement("td");
        providers.appendChild(document.createTextNode(services[s].join(", ")));

        r.appendChild(name);
        r.appendChild(providers);
        tb.appendChild(r);
    }
    t.appendChild(tb);

    return t;
}

function initializeCoreMonitor() {
    // handle message passed from extension to webview
    window.addEventListener("message", (event) => {
        const message = event.data;
        console.debug(message);

        const coreStatus = document.getElementById("ros-status");
        const parametersElement = document.getElementById("parameters");
        const topicsElement = document.getElementById("topics");
        const servicesElement = document.getElementById("services");

        removeAllChildElements(parametersElement);
        removeAllChildElements(topicsElement);
        removeAllChildElements(servicesElement);

        if (message.status) {
            coreStatus.textContent = "online";

            let parameters = JSON.parse(message.parameters);
            let systemState = JSON.parse(message.systemState);

            let parametersHeader = document.createElement("h2");
            parametersHeader.appendChild(document.createTextNode("Parameters"));
            parametersElement.appendChild(parametersHeader);
            parametersElement.appendChild(generateParemetersUnorderedList(parameters));

            let topicsHeader = document.createElement("h2");
            topicsHeader.appendChild(document.createTextNode("Topics"));
            topicsElement.appendChild(topicsHeader);
            topicsElement.appendChild(generateTopicsTable(systemState.publishers, systemState.subscribers));
            let servicesHeader = document.createElement("h2");
            servicesHeader.appendChild(document.createTextNode("Services"));
            servicesElement.appendChild(servicesHeader);
            servicesElement.appendChild(generateServicesTable(systemState.services));
        }
        else {
            coreStatus.textContent = "offline";
        }
    });
};

window.onload = () => initializeCoreMonitor();
