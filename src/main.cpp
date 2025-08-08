// main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "CameraSetup.h"

// BLE UUIDs
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define NOTIFY_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void positionFunctionBatch();



ICM20948_WE myIMU(0x68);

const char* ssid     = "WALKEY-ESP32";
const char* password = "WALKEY123456";

AsyncWebServer server(80);

BLECharacteristic* pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("✅ Device connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("❌ Device disconnected");
  }
};

void setup() {
  delay(8000);
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  Serial.printf("Initialize good");
  
  //initialize camera
  setupCamera();
  // if (!cameraReady) while(true) delay(1000);
  
  // start Wi-Fi AP
  WiFi.softAP(ssid, password);
  Serial.printf("AP IP address: %s\n", WiFi.softAPIP().toString().c_str());

  server.on("/picture.jpg", HTTP_GET, [](AsyncWebServerRequest *request){
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    request->send(500, "text/plain", "Capture failed");
    return;
  }
  // this overload takes (code, contentType, dataPtr, dataLen)
  // 2) Build a response object (lets you add headers)
  AsyncWebServerResponse* res = 
  request->beginResponse_P(
      200,                        // HTTP status
      "image/jpeg",               // MIME type
      fb->buf,                    // pointer
      fb->len                     // length
  );
  res->addHeader("Content-Disposition",
                "inline; filename=\"picture.jpg\"");
  request->send(res);
  esp_camera_fb_return(fb);
  });

  // basic index page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/html",
      "<h1>ESP32-CAM EdgeAI</h1>"
      "<p><a href=\"/picture.jpg\">Snap & view a picture</a></p>"
    );
  });

  server.begin();

  // init IMU
  if (!myIMU.init())           Serial.println("ICM20948 does not respond");
  else                          Serial.println("ICM20948 is connected");
  if (!myIMU.initMagnetometer()) Serial.println("Magnetometer does not respond");
  else                          Serial.println("Magnetometer is connected");

  // init BLE
  // BLEDevice::setMTU(127); 
  delay(2000);
  BLEDevice::init("EdgeAI-ICM20948");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(NOTIFY_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->setScanResponse(true);
  pServer->getAdvertising()->start();
  Serial.println("BLE listo y en espera de conexión…");
}

void loop() {
  if (deviceConnected) {
    positionFunctionBatch();
    delay(100);  // optional
  }

  // Check if client disconnected and restart advertising
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);  // allow time before restarting advertising
    BLEDevice::getAdvertising()->start();
    Serial.println("🔄 Restarted advertising");
    oldDeviceConnected = deviceConnected;
  }

  // Check if client just connected
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("🔒 Ready to send data");
    oldDeviceConnected = deviceConnected;
  }

  delay(500);
}


// Position function
// This function takes two integers and returns their product
void positionFunctionBatch() {
  const int sampleCount = 100;

  for (int i = 0; i < sampleCount; i++) {
    myIMU.readSensor();

    xyzFloat acc, gyro, mag;
    myIMU.getAccRawValues(&acc);
    myIMU.getGyrRawValues(&gyro);
    myIMU.getMagValues(&mag);

    // Crear documento JSON
    // StaticJsonDocument<256> doc;
    JsonDocument doc; 
    doc["a"]["x"] = acc.x;
    doc["a"]["y"] = acc.y;
    doc["a"]["z"] = acc.z;

    doc["g"]["x"] = gyro.x;
    doc["g"]["y"] = gyro.y;
    doc["g"]["z"] = gyro.z;

    doc["m"]["x"] = mag.x;
    doc["m"]["y"] = mag.y;
    doc["m"]["z"] = mag.z;

    // Serializar JSON a string
    String output;
    serializeJson(doc, output);

    // Enviar por BLE
    if (deviceConnected) {
      pCharacteristic->setValue(output.c_str());
      pCharacteristic->notify();
    }

    // Debug por Seria  l
    //Serial.println(output);

    delay(100);  // 100 Hz (for now)
  }
}
