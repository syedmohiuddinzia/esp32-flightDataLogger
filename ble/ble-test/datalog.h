#include <Arduino.h>
#include "SD.h"

#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MPU9250.h"

#include "eeprom_utils.h"
#include <SPIFFS.h>
#include "LittleFS.h"
#include <TinyGPS++.h>
TinyGPSPlus gps;

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

String messages="", data="";
int count = 0;
bool isRecording = false;

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
String apiKey = "";
String mobileNumber = "";
String mobileMessage = "";
const char* configFileSOS = "/configSOS.txt";

// Function to load config from SD card
void loadConfigSOS() {
    if (!SD.exists(configFileSOS)) {Serial.println("Config file SOS not found.");return;}

    File file = SD.open(configFileSOS, "r");
    if (file) {
//      mode = file.readStringUntil('\n');mode.trim();
        apiKey = file.readStringUntil('\n');apiKey.trim();
        mobileNumber = file.readStringUntil('\n');mobileNumber.trim();
        mobileMessage = file.readStringUntil('\n');mobileMessage.trim();
        file.close();
        Serial.println("Loaded from SD: API Key: " + apiKey + " Mobile: " + mobileNumber + " Message: " + mobileMessage);
    } else {Serial.println("Failed to open config file.");}
}

// Function to save config to SD card
void saveConfigSOS(String newApiKey, String newMobile, String message) {
    File file = SD.open(configFileSOS, "w");  // Overwrite existing file
    if (file) {
        file.println(newApiKey);file.println(newMobile);file.println(message);file.close();
        Serial.println("Config SOS saved to SD.");
    } else {Serial.println("Failed to write SOS to SD card.");}
}

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
void writeFileSD(fs::FS &fs, const char *path, const char *message) {
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
void appendFileSD(fs::FS &fs, const char *path, const char *message) {
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
void deleteFileSD(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
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
//  Serial.println("Heading: " + String(headingDegrees));
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
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();
    print_calibration();
    mpu.verbose(false);
    // save to eeprom
    saveCalibration();
    // load from eeprom
    loadCalibration();
    calibration = false;
}

void dataString(){
// Create a data string
data = String(millis()) + "," + String(latitude,5) + "," + String(longitude,5) + "," + String(alt,3) + "," + String(sat) + "," + String(gpsDate) + "," + String(gpsTime) + "," + 
String(ax,3) + "," + String(ay,3) + "," + String(az,3) + "," + String(gx,3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
String(mx,3) + "," + String(my,3) + "," + String(mz,3) + "," + String(yaw,3) + "," + String(pitch,3) + "," + String(roll,3) + "," + 
String(headingDegrees) + "," + String(t, 3) + "," + String(pressure, 3) + "," + String(altitude, 3) + "," + String(tempBMP, 3) + "\r\n";
}
