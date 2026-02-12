function sendCheckboxData(sensor, state) {
    let url = "/" + sensor + "?state=" + (state ? "1" : "0");
    fetch(url)
        .then(response => console.log(sensor + " state changed: " + (state ? "ON" : "OFF")))
        .catch(error => console.error("Error sending request:", error));
}

function sendTextData() {
    let value = document.getElementById("value").value;
    if (value.trim() !== "") {  // Avoid sending empty text
        fetch("/insert?value=" + encodeURIComponent(value))
            .then(response => console.log("Inserted text: " + value))
            .catch(error => console.error("Error sending request:", error));
    } else {
        alert("Please enter some text before sending.");
    }
}

let isRecording = false;
function toggleRecording() {
    isRecording = !isRecording;
    let button = document.getElementById("recordButton");
    button.innerText = isRecording ? "Recording" : "Record";
    button.className = isRecording ? "button button-recording" : "button button-record";
    
    fetch("/record?state=" + (isRecording ? "1" : "0"))
        .then(response => console.log("Recording state: " + (isRecording ? "ON" : "OFF")))
        .catch(error => console.error("Error sending request:", error));
}
