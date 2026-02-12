#include <datalog.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Serial port begin");
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  initSDCard();
  loadConfigWIFI();
  if (mode == "STA" && !ssid.isEmpty() && !password.isEmpty()) {
      if (!startSTAMode()) {mode = "AP";startAPMode();}} else {startAPMode();}
//  initWiFi();

delay(1000);
File root = SD.open("/");
while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    Serial.println(entry.name()); // Print all files found
    entry.close();
}

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  handling();
  server.begin();
  initializeBMP280();
  initializeMPU9250();

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.csv");
  if (!file) {Serial.println("File doesn't exist");Serial.println("Creating file...");
    writeFileSD(SD, "/data.csv", "Time, Lat, Lng, Alt, Sat, gpsDate, gpsTime, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ, Yaw, Pitch, Roll, Heading, TempIMU, Pressure, Altitude, TempBMP \r\n");
  } else {Serial.println("File already exists");}
  file.close();

  // Uncomment the following line if you need to serve more static files like CSS and javascript or favicon
  //server.serveStatic("/", SD, "/");
}

void loop() {
  if (calibration){calibrateMPU9250();}
  if (mpu.update()) {
  if ((millis() - lastTime) > timerDelay) {
    gpsData();
    accelerometer();gyrometer();magnetometer();
    roll_pitch_yaw();temperature();mag_heading();
    BMP280();
//    Serial.print(data);
    dataString();
    data_ws();
    ws.cleanupClients();
    lastTime = millis();
    if (isRecording){appendFileSD(SD, "/data.txt", data.c_str());}
    }
  }
  if (ESP.getFreeHeap()<10000){ESP.restart();}
}
