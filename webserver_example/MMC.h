#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <sensors.h>

String messages="", data="";
int count = 0;

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Timer variables
unsigned long lastTime = 0, timerDelay = 250;

String dataMessage;

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
  if (fs.remove(path)) {Serial.println("- file deleted");} 
  else {Serial.println("- delete failed");}
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        client->text(messages);
    } else if (type == WS_EVT_DISCONNECT) {
//        Serial.println("WebSocket client disconnected");
    }
}

void dataString(){
// Create a data string
data = String(millis()) + "," + String(latitude,5) + "," + String(longitude,5) + "," + String(alt,3) + "," + String(sat) + "," + String(gpsDate) + "," + String(gpsTime) + "," + 
String(ax,3) + "," + String(ay,3) + "," + String(az,3) + "," + String(gx,3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
String(mx,3) + "," + String(my,3) + "," + String(mz,3) + "," + String(yaw,3) + "," + String(pitch,3) + "," + String(roll,3) + "," + 
String(headingDegrees) + "," + String(t, 3) + "," + String(pressure, 3) + "," + String(altitude, 3) + "," + String(tempBMP, 3) + "\r\n";
}
    
