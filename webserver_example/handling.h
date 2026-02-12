#include <ESPAsyncWebServer.h>
#include <sos.h>
#include <MMC.h>
#include <sensors.h>
#include <internet.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

AsyncWebServer server(80);
// Create AsyncWebServer object on port 80
bool isRecording = false;
const char *dataPath = "/data.txt";


void data_ws(){
  if (ws.count()>0) {Serial.println("Sending: " + data);ws.textAll(data);}
//  else {Serial.println("No WebSocket clients connected.");}
}

// Endpoint to handle POST data
void handlePostData(AsyncWebServerRequest *request) {
String imu1State = request->arg("imu1");  // Get IMU 1 state (checked/unchecked)
String imu2State = request->arg("imu2");  // Get IMU 2 state (checked/unchecked)
String textValue = request->arg("value"); // Get the text input

// Print the received values to the Serial monitor
Serial.println("Received Data:");
Serial.println("IMU 1: " + imu1State);
Serial.println("IMU 2: " + imu2State);
Serial.println("Text Input: " + textValue);

// Prepare a data string to append to the SD card
String data = "IMU 1: " + imu1State + ", IMU 2: " + imu2State + ", Text Value: " + textValue;

// Append the data to the SD card
appendFile(SD, "/data.txt", data.c_str());

// Respond to the client
request->send(200, "text/plain", "Data received and saved.");
}
void handling()
{
  // Handle the root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(SD, "/index.html", "text/html");
  });
  
  server.on("/sos", HTTP_GET, [](AsyncWebServerRequest *request){
  sendSOS();
  request->send(200, "text/plain", "SOS Triggered!");
  });

  // Handle mode change
  server.on("/setmode", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (request->hasParam("mode", true)) {
          mode = request->getParam("mode", true)->value();
      }
      if (mode == "STA" && request->hasParam("ssid", true) && request->hasParam("password", true)) {
          ssid = request->getParam("ssid", true)->value();
          password = request->getParam("password", true)->value();
      }
      saveConfigWIFI();
      request->send(200, "text/plain", "Saved! Rebooting...");
      delay(1000);
      ESP.restart();
  });

  server.on("/record", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (request->hasParam("state")) {
          String state = request->getParam("state")->value();
          if (state == "1") {
              isRecording = true;
              Serial.println("Recording Started!");
              // Add your recording logic here (e.g., start capturing data)
          } else {
              isRecording = false;
              Serial.println("Recording Stopped!");
              // Add your stopping logic here
          }
          request->send(200, "text/plain", "Recording state: " + state);
      } else {
          request->send(400, "text/plain", "Missing state parameter");
      }
  });

  // Handle checkbox updates
  server.on("/imu1", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("state")) {
        String state = request->getParam("state")->value();
        Serial.print("IMU1 state: ");
        Serial.println(state);
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/imu2", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("state")) {
        String state = request->getParam("state")->value();
        Serial.print("IMU2 state: ");
        Serial.println(state);
    }
    request->send(200, "text/plain", "OK");
  });

  // Handle checkbox updates
  server.on("/imu3", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("state")) {
        String state = request->getParam("state")->value();
        Serial.print("IMU3 state: ");
        Serial.println(state);
    }
    request->send(200, "text/plain", "OK");
  });


  // Handle text input updates
  server.on("/insert", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
        String value = request->getParam("value")->value();
        Serial.print("Inserted value: ");
        Serial.println(value);
    }
    request->send(200, "text/plain", "OK");
  });

  // Handle the download button
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/data.txt", String(), true);
  });

  // Handle the View Data button
  server.on("/view-data", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/data.txt", "text/plain", false);
  });

  // Handle the delete button
  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request) {
    deleteFile(SD, dataPath);
    request->send(200, "text/plain", "data.txt was deleted.");
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request){
    calibration=true;
    request->send(200, "text/html", "<html><body><h2>Datalog calibration</h2><pre id='log'></pre><script>let ws=new WebSocket('ws://'+location.host+'/ws'); ws.onmessage=e=>document.getElementById('log').innerText=e.data;</script></body></html>");
    });

  server.on("/display", HTTP_GET, [](AsyncWebServerRequest *request){ 
    request->send(200, "text/html", 
        "<html>"
        "<body>"
        "<h2>Datalog Display</h2>"
        "<table border='1' id='dataTable'>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "</table>"
        "<script>"
        "let ws = new WebSocket('ws://' + location.host + '/ws');"
        "ws.onmessage = function(e) {"
        "  let data = e.data.split(',');"
        "  let table = document.getElementById('dataTable');"
        "  table.innerHTML = '<tr><th>Parameter</th><th>Value</th></tr>';" // Clear the table and add headers
        "  let parameters = ['Time', 'Latitude', 'Longitude', 'Altitude', 'Satellites', 'GPS Date', 'GPS Time', 'Accel X', 'Accel Y', 'Accel Z', 'Gyro X', 'Gyro Y', 'Gyro Z', 'Mag X', 'Mag Y', 'Mag Z', 'Yaw', 'Pitch', 'Roll', 'Heading', 'Temperature', 'Pressure', 'Altitude BMP', 'Temperature BMP'];"
        "  for (let i = 0; i < data.length; i++) {"
        "    let row = table.insertRow(-1);"
        "    let cell1 = row.insertCell(0);"
        "    let cell2 = row.insertCell(1);"
        "    cell1.innerHTML = parameters[i];"
        "    cell2.innerHTML = data[i];"
        "  }"
        "};"
        "</script>"
        "</body>"
        "</html>"
    );
});
}
