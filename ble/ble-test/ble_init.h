#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
BLECharacteristic* p_ax_CHARACTERISTIC = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
const int ledPin = 2; // Use the appropriate GPIO pin for your setup

// See the following for generating UUIDs: // https://www.uuidgenerator.net/
#define SERVICE_UUID       "19b10000-e8f2-537e-4f6c-d104768a1214"
#define ax_CHARACTERISTIC  "c0cebf60-6921-4d89-95cc-49e554042543"
#define ay_CHARACTERISTIC  "fe50f581-bb2e-44cd-a291-61d610c7d371"
#define az_CHARACTERISTIC  "0fff8801-6eaf-4169-935a-a5c8fafba5bf"
#define gx_CHARACTERISTIC  "a297f306-9bdc-432a-84d9-0e52c0e0518c"
#define gy_CHARACTERISTIC  "74cbbd0a-20aa-41d8-a066-b8b9c8de3ab9"
#define gz_CHARACTERISTIC  "900660dd-fcf0-4e4f-9395-d3defbf5a60f"
#define mx_CHARACTERISTIC  "9995c794-f866-47d7-bbc6-5e5689af9d42"
#define my_CHARACTERISTIC  "618efe1d-f42e-40aa-ba5e-b8b238635db5"
#define mz_CHARACTERISTIC  "080695a7-43ef-4a2e-aa2c-a34e73aa983a"
#define la_x_CHARACTERISTIC "8eb46be3-e9f5-458e-b301-04677cf58de8"
#define la_y_CHARACTERISTIC "4288eaf5-2db4-4187-8bae-db5b8651187d"
#define la_z_CHARACTERISTIC "521e5bdf-adcc-4778-bc4c-33d216d96631"
#define h_CHARACTERISTIC "08abc1b6-c2e2-4369-81f0-5a503d7ce510"
#define qx_CHARACTERISTIC "c9abbc0c-706d-4e51-b64e-f11ee77c56d3"
#define qy_CHARACTERISTIC "22ebb5cf-f9bf-473a-b0ea-34b6ae317c83"
#define qz_CHARACTERISTIC "0c7dd35e-36fc-4427-a3d9-be9941636709"
#define qw_CHARACTERISTIC "591a8986-75be-41ce-8f6e-f6497b7c5379"
#define ex_CHARACTERISTIC "56011175-a314-4255-9710-ca1b61a02e75"
#define ey_CHARACTERISTIC "b90071f8-1564-4b9f-96e4-7f98390257c2"
#define ez_CHARACTERISTIC "8adbaeb9-b9b2-4c38-9bfb-1b52ed3baf7c"
#define yaw_CHARACTERISTIC "b3f0c6eb-fa34-4d29-8648-291fedd3027a"
#define pitch_CHARACTERISTIC "cbd471ac-a01f-413e-b115-2f8808264f8a"
#define roll_CHARACTERISTIC "065c644d-a53c-4aaf-9484-9a9610857953"
#define t_CHARACTERISTIC "45a1920f-a5bc-4415-8dbb-9c92f262c8df"
#define headingDegrees_CHARACTERISTIC "9e3a2d36-27f5-44f0-b175-d39d8a9cdac0"
#define temp_CHARACTERISTIC "bf7c7ebe-b5bc-4aeb-9d0e-23b2ca92ffa9"
#define hum_CHARACTERISTIC "b936b792-e878-4520-8f05-47dc91b7d633"
#define pres_CHARACTERISTIC "02db706a-40af-4b16-b3d8-b02819198fc2"

#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {void onConnect(BLEServer* pServer) {deviceConnected = true;};

void onDisconnect(BLEServer* pServer) {deviceConnected = false;}};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    String value = pLedCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Characteristic event, written: ");Serial.println(static_cast<int>(value[0])); // Print the integer value
      int receivedValue = static_cast<int>(value[0]);if (receivedValue == 1) {digitalWrite(ledPin, HIGH);} else {digitalWrite(ledPin, LOW);}
    }}};

void initBLE_CHARACTERISTICS()
{
 BLEDevice::init("FLIGHT LOGGER"); // Create the BLE Device

  // Create the BLE Server
  pServer = BLEDevice::createServer(); pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID); // Create the BLE Service

  // Create a BLE Characteristic
  p_ax_CHARACTERISTIC = pService->createCharacteristic(ax_CHARACTERISTIC,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_NOTIFY|BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(LED_CHARACTERISTIC_UUID,BLECharacteristic::PROPERTY_WRITE);
  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  p_ax_CHARACTERISTIC->addDescriptor(new BLE2902()); pLedCharacteristic->addDescriptor(new BLE2902());
  pService->start();// Start the service

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();Serial.println("Waiting a client connection to notify...");
}
