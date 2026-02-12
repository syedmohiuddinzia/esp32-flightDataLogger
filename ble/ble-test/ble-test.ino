#include <datalog.h>
#include <ble_init.h>

void setup() {
  Serial.begin(115200); pinMode(ledPin, OUTPUT);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  initSDCard();
  delay(1000);
  File root = SD.open("/");
  while (true) {File entry = root.openNextFile();if (!entry) break;
  Serial.println(entry.name());entry.close();} // Print all files found

  initializeBMP280(); initializeMPU9250();
  initBLE_CHARACTERISTICS();
}

void loop() {
  if (calibration){calibrateMPU9250();}
  if (mpu.update()) {if ((millis() - lastTime) > timerDelay) {
    gpsData();accelerometer();gyrometer();magnetometer();
    roll_pitch_yaw();temperature();mag_heading();BMP280();
    dataString();lastTime = millis();
    if (isRecording){appendFileSD(SD, "/data.txt", data.c_str());}
    }
  }
  if (deviceConnected) {
//    String msg = "hello 123 : " + String(t);pSensorCharacteristic->setValue(msg.c_str());pSensorCharacteristic->notify();value++;
//    Serial.print("New value notified: ");Serial.println(value);delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    p_ax_CHARACTERISTIC->setValue(String(ax).c_str());p_ax_CHARACTERISTIC->notify();
    Serial.print("Ax notified: ");Serial.println(ax);delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {Serial.println("Device disconnected.");delay(100); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();Serial.println("Start advertising");oldDeviceConnected = deviceConnected; // restart advertising
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {oldDeviceConnected = deviceConnected;Serial.println("Device Connected");// do stuff here on connecting
  }
}
