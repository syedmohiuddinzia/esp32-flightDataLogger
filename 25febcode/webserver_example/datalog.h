#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MPU9250.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "eeprom_utils.h"
#include <SPIFFS.h>
#include "LittleFS.h"
#include <TinyGPS++.h>
TinyGPSPlus gps;

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
    
// Replace with your network credentials
const char *ssid = "ESP32";         // Enter SSID here
const char *password = "12345678";  //Enter Password here
String messages="", data="";
int count = 0;
bool isRecording = false;

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

Adafruit_BMP280 bmp280;
MPU9250 mpu;

bool calibration = false;  
float pressure, altitude, tempBMP;
float ax, ay, az, gx, gy, gz, mx, my, mz, la_x, la_y, la_z;
float h, qx, qy, qz, qw, ex, ey, ez, yaw, pitch, roll, t;
float latitude, longitude, alt, sat;
//float declinationAngle = 0.02234021, heading; // Magnetic Declination in Karachi is 0.02234021 radians, 1.28 degrees.
float declinationAngle = 0.071558, heading; // Magnetic Declination in Benghazi is 0.071558 radians, 4.1 degrees.
String gpsDate, gpsTime;
int headingDegrees;

// Timer variables
unsigned long lastTime = 0, timerDelay = 500;
// Variables to hold sensor readings
float temp, hum, pres;
String dataMessage;
String calibrationStatus = "Waiting for calibration...";
String calibrationStatusData = "Calibration Started";

const char *dataPath = "/data.txt";

// Init microSD card
void initSDCard() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// Write to the SD card
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card
void appendFile(fs::FS &fs, const char *path, const char *message) {
//  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
//    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// Delete file
void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

// Function that initializes wi-fi
void initWiFi() {
    Serial.println("Setting AP (Access Point)…");
    // You can remove the password parameter if you want the AP to be open.
    // a valid password must have more than 7 characters
    if (!WiFi.softAP(ssid, password)) {
      log_e("Soft AP creation failed.");
      while (1);
    }
    WiFi.softAPConfig(local_ip, gateway, subnet);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
}

void BMP280(){
    // Read BMP280 barometer values
    pressure = bmp280.readPressure();         // Pressure in Pa
    altitude = bmp280.readAltitude(1013.25);  // Altitude in meters (default sea level pressure)
    tempBMP = bmp280.readTemperature();       // Temperature in °
}

// Function to initialize BMP280 barometer
void initializeBMP280() {
  if (!bmp280.begin(0x76)) {  // Default I2C address of BMP280 is 0x76
    Serial.println("Error: BMP280 not detected. Check connections.");
    while (1);  // Halt if BMP280 initialization fails
  }
  Serial.println("BMP280 initialized successfully.");

  // Set BMP280 configuration (oversampling and filter settings)
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,      // Temperature oversampling
                     Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                     Adafruit_BMP280::FILTER_X16,       // Filtering
                     Adafruit_BMP280::STANDBY_MS_500);  // Standby time
}

void gpsData()
{
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        gps.encode(c);  // Feed data to TinyGPS++

        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();longitude=gps.location.lng();alt=gps.altitude.meters();sat=gps.satellites.value();
            Serial.print("Latitude: ");Serial.print(", Longitude: ");Serial.print(", Altitude: ");Serial.print(" m, Satellites: ");
        }
        else{/*Serial.print("bad location");*/}
        if (gps.time.isUpdated() && gps.date.isUpdated()) {
            // Format date & time as a string "YYYY-MM-DD HH:MM:SS"
            gpsDate = String(gps.date.year())+"-"+String(gps.date.month())+"-"+String(gps.date.day());
            gpsTime = String(gps.time.hour())+":"+String(gps.time.minute())+":"+String(gps.time.second());
//            Serial.print("⏳ GPS Date & Time (UTC): ");Serial.print(gpsDate + " ");Serial.println(gpsTime);
        }
        }
}


void accelerometer() {
  ax = mpu.getAccX();
  ay = mpu.getAccY();
  az = mpu.getAccZ();
  // Serial.println("AccX: " + String(ax) + " AccY: " + String(ay) + " AccZ: " + String(az));
}

void gyrometer() {
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();
  // Serial.println("GyroX: " + String(gx) + " GyroY: " + String(gy) + " GyroZ: " + String(gz));
}

void magnetometer() {
  mx = mpu.getMagX();
  my = mpu.getMagY();
  mz = mpu.getMagZ();
//  Serial.println("MagX: " + String(mx) + " MagY: " + String(my) + " MagZ: " + String(mz));
}

void linear_acc() {
  la_x = mpu.getLinearAccX();
  la_y = mpu.getLinearAccY();
  la_z = mpu.getLinearAccZ();
  // Serial.println("LinAccX: " + String(la_x) + " LinAccY: " + String(la_y) + " LinAccZ: " + String(la_z));
}

void quaternion() {
  qx = mpu.getQuaternionX();
  qy = mpu.getQuaternionY();
  qz = mpu.getQuaternionZ();
  qw = mpu.getQuaternionW();
  // Serial.println("QuatX: " + String(qx) + " QuatY: " + String(qy) + " QuatZ: " + String(qz) + " QuatW: " + String(qw));
}

void euler() {
  ex = mpu.getEulerX();
  ey = mpu.getEulerX();
  ez = mpu.getEulerZ();
  // Serial.println("EulerX: " + String(ex) + " EulerY: " + String(ey) + " EulerZ: " + String(ez));
}

void roll_pitch_yaw() {
  yaw = mpu.getYaw()+46;
  pitch = mpu.getPitch();
  roll = mpu.getRoll();
//  Serial.println("Yaw: " + String(yaw) + " Pitch: " + String(pitch) + " Roll: " + String(roll));
}

void temperature(){
  t= mpu.getTemperature();
//  Serial.println("Temperature: " + String(t));
}

void mag_heading(){
  headingDegrees = atan2(mpu.getMagY(), mpu.getMagX())/0.0174532925;
  if(headingDegrees<=0)
  {headingDegrees+=360;}
  Serial.println("Heading: " + String(headingDegrees));
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

// Function to initialize MPU9250 barometer
void initializeMPU9250() {
  delay(2000);
  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
              }
  }
#if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.begin(0x80);
#endif
    loadCalibration();
    print_calibration();
    roll_pitch_yaw();
}

void calibrateMPU9250(){
    // Sends the updated messages string to all connected WebSocket clients
    ws.textAll(calibrationStatus);
    delay(2000);
    calibrationStatus += calibrationStatusData + "\n\n";
    // calibrate anytime you want to
    calibrationStatusData = "Accel Gyro calibration will start in 5sec. Please leave the device still on the flat plane.";
    calibrationStatus += calibrationStatusData + "\n\n";
    ws.textAll(calibrationStatus);
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();
    calibrationStatusData = "Mag calibration will start in 5sec. Please Wave device in a figure eight until done..";
    calibrationStatus += calibrationStatusData + "\n\n";
    ws.textAll(calibrationStatus);
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();
    print_calibration();
    mpu.verbose(false);
    // save to eeprom
    saveCalibration();
    calibrationStatusData = "Calibration Saved";
    calibrationStatus += calibrationStatusData + "\n\n";
    Serial.println("Calibration Saved");
    ws.textAll(calibrationStatus);
    // load from eeprom
    loadCalibration();
    calibrationStatusData = "Calibration loaded";
    calibrationStatus += calibrationStatusData + "\n\n";
    Serial.println("Calibration loaded");
    ws.textAll(calibrationStatus);
    calibration = false;
    calibrationStatus = "";
    calibrationStatusData = "";
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        client->text(messages);
    } else if (type == WS_EVT_DISCONNECT) {
//        Serial.println("WebSocket client disconnected");
    }
}
  
void handling()
{
     // Handle the root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/index.html", "text/html");
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

void dataString(){
// Create a data string
data = String(millis()) + "," + String(latitude,5) + "," + String(longitude,5) + "," + String(alt,3) + "," + String(sat) + "," + String(gpsDate) + "," + String(gpsTime) + "," + 
String(ax,3) + "," + String(ay,3) + "," + String(az,3) + "," + String(gx,3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
String(mx,3) + "," + String(my,3) + "," + String(mz,3) + "," + String(yaw,3) + "," + String(pitch,3) + "," + String(roll,3) + "," + 
String(headingDegrees) + "," + String(t, 3) + "," + String(pressure, 3) + "," + String(altitude, 3) + "," + String(tempBMP, 3) + "\r\n";
}
void data_ws(){
  if (ws.count()>0) {Serial.println("Sending: " + data);ws.textAll(data);}
//  else {Serial.println("No WebSocket clients connected.");}
}
