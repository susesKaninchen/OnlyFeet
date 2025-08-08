#include <Arduino.h>
#include <Ticker.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// Timer
Ticker photoTicker;
Ticker imuLidarTicker;

// Mutex für SD-Zugriffe
SemaphoreHandle_t sdMutex;

// Task Handles
TaskHandle_t micTaskHandle;

// Setup SD
bool initSD() {
  if (!SD.begin()) {
    Serial.println("SD Karte nicht gefunden");
    return false;
  }
  Serial.println("SD Karte initialisiert");
  return true;
}

// Kamera: 1 Bild pro Sekunde
void capturePhoto() {
  String filename = "/photo_" + String(millis() / 1000) + ".jpg";
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100))) {
    savePhoto(SD, filename.c_str());
    xSemaphoreGive(sdMutex);
  }
}

// IMU & Lidar @100Hz
void imuLidarUpdate() {
  static uint32_t lastTimestamp = millis();
  uint32_t now = millis();

  imu_data_t imu = readIMU();
  lidar_data_t lidar = readLidar();

  String filename = "/data_" + String(now / 1000) + ".csv";
  String dataLine = String(now) + "," +
                    String(imu.accelX) + "," +
                    String(imu.gyroX) + "," +
                    String(imu.magX) + "," +
                    String(lidar.distance) + "\n";

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10))) {
    File file = SD.open(filename, FILE_APPEND);
    if (file) {
      file.print(dataLine);
      file.close();
    }
    xSemaphoreGive(sdMutex);
  }
}

// Mikrofon Task (1s Audio)
void micTask(void *parameter) {
  while (true) {
    String filename = "/audio_" + String(millis() / 1000) + ".raw";
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500))) {
      recordAudio(SD, filename.c_str(), 1000); // 1s Aufnahme
      xSemaphoreGive(sdMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Sekunde warten
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  if (!initSD()) {
    while (true);
  }

  sdMutex = xSemaphoreCreateMutex();

  // Starte Tasks und Timer
  photoTicker.attach(1.0, capturePhoto);           // 1Hz Foto
  imuLidarTicker.attach_ms(10, imuLidarUpdate);    // 100Hz IMU & Lidar

  xTaskCreatePinnedToCore(
    micTask,
    "Mic Task",
    4096,
    NULL,
    1,
    &micTaskHandle,
    1
  );
}

void loop() {
  // Hauptloop leer, alles über Timer & Tasks
  vTaskDelay(pdMS_TO_TICKS(5000)); // 1 Sekunde warten, um CPU zu entlasten
}
