#include <WiFi.h>
//#include <WiFiUdp.h>

const char* AP_SSID = "ESP32_Config";
const char* AP_PASSWORD = "12345678";
// Replace with your network credentials
String mode = "AP";
String ssid = "";
String password = "";
const char* configFile = "/config.txt";
//const char *ssid = "ESP32";         // Enter SSID here
//const char *password = "12345678";  //Enter Password here

// Function to load config from SD card
void loadConfigWIFI() {
    if (!SD.exists(configFile)) {
        Serial.println("Config file not found.");
        return;
    }

    File file = SD.open(configFile, FILE_READ);
    if (file) {
        mode = file.readStringUntil('\n');
        mode.trim();
        if (mode == "STA") {
            ssid = file.readStringUntil('\n');
            password = file.readStringUntil('\n');
            ssid.trim();
            password.trim();
        }
        file.close();
        Serial.println("Loaded from SD:");
        Serial.println("Mode: " + mode);
        Serial.println("SSID: " + ssid);
        Serial.println("Password: " + password);
    } else {
        Serial.println("Failed to open config file.");
    }
}

// Function to save config to SD card
void saveConfigWIFI() {
    File file = SD.open(configFile, FILE_WRITE);
    if (file) {
        file.println(mode);
        if (mode == "STA") {
            file.println(ssid);
            file.println(password);
        }
        file.close();
        Serial.println("Config saved to SD.");
    } else {
        Serial.println("Failed to write to SD card.");
    }
}

// Start Access Point Mode
void startAPMode() {
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.println("Started in AP Mode");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
}

// Start Station Mode
bool startSTAMode() {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi...");
    unsigned long startAttemptTime = millis();
    
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(1000);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nFailed to connect. Switching to AP Mode.");
        return false;
    }
}
