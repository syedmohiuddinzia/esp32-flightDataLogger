String apiKey = "";
String mobileNumber = "";

// Function to load config from SD card
void loadConfigSOS() {
    if (!SD.exists(configFile)) {Serial.println("Config file not found.");return;}

    File file = SD.open(configFile, "r");
    if (file) {
        mode = file.readStringUntil('\n');mode.trim();
        apiKey = file.readStringUntil('\n');apiKey.trim();
        mobileNumber = file.readStringUntil('\n');mobileNumber.trim();
        file.close();
        Serial.println("Loaded from SD: API Key: " + apiKey + " Mobile: " + mobileNumber);
    } else {Serial.println("Failed to open config file.");}
}

// Function to save config to SD card
void saveConfigSOS(String newApiKey, String newMobile) {
    File file = SD.open(configFile, "w");  // Overwrite existing file
    if (file) {
        file.println(newApiKey);file.println(newMobile);file.close();
        Serial.println("Config saved to SD.");
    } else {Serial.println("Failed to write to SD card.");}
}

// Handle incoming request to save config
void handleSaveConfigSOS(AsyncWebServerRequest *request) {
    if (request->hasParam("apikey") && request->hasParam("mobile")) {
        String newApiKey = request->getParam("apikey")->value();
        String newMobile = request->getParam("mobile")->value();

        saveConfigSOS(newApiKey, newMobile);  // Save to SD card

        request->send(200, "text/plain", "Configuration saved to SD card.");
    } else {
        request->send(400, "text/plain", "Missing parameters.");
    }
}

void sendSOS() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected!");
        return;
    }

    String message = "DANGER ALERT !!!";
    String url = "https://api.callmebot.com/whatsapp.php?phone=" + String(mobileNumber) +
                 "&text=" + message + "&apikey=" + String(apiKey);
    
    WiFiClientSecure client;
    client.setInsecure();  // Use an insecure connection to avoid SSL verification

    HTTPClient http;
    http.begin(client, url);
    http.setTimeout(5000);  // Prevent watchdog resets

    int httpCode = http.GET();  // Send the request

    if (httpCode > 0) {
        Serial.println("SOS message sent successfully!");
    } else {
        Serial.println("Error sending SOS message: " + String(http.errorToString(httpCode).c_str()));
    }
    http.end();
}
