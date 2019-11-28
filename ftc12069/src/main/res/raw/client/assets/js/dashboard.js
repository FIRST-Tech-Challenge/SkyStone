// This asynchronously performs a GET request
function get(url) {
    return new Promise((resolve, reject) => {
        let xmlHTTPRequest = new XMLHttpRequest();
        xmlHTTPRequest.onreadystatechange = () => {
            if (xmlHTTPRequest.readyState === 4 && xmlHTTPRequest.status === 200) resolve(xmlHTTPRequest.responseText);
            else reject("Failed to request");
        }
        xmlHTTPRequest.open("GET", url, true);
        xmlHTTPRequest.send(null);
    });
}

let lastRequest = 0;
while (true) {
    if (Date.now() - 100 > lastRequest) {
        let stats = await get("/api/stats").catch(() => null);
        if (stats != null) {
            let feedElement = document.getElementById("cameraFeed");
            feedElement.src = stats.stream;
            lastRequest = Date.now();
        }
    }
}