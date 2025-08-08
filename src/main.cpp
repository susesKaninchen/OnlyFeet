// main.cpp – sensor packet + 1‑s audio + photo + JSON logger to SD‑card for Seeed XIAO ESP32S3 Sense
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>

#include "esp_camera.h"
#include "camera_pins.h"   // liefert XIAO‑Sense‑Pins, wenn CAMERA_MODEL_XIAO_ESP32S3 definiert ist

#include "ESP_I2S.h"        // PDM‑MIC
#include "FS.h"
#include "SD.h"

/********* Audio / SD configuration *********/
static constexpr int MIC_DATA_PIN = 42;   // PDM‑DAT  (GPIO42)
static constexpr int MIC_CLK_PIN  = 41;   // PDM‑CLK  (GPIO41)
static constexpr int SD_CS_PIN    = 21;   // µSD‑CS   (GPIO21)
static constexpr uint16_t AUDIO_FS   = 16'000;   // 16 kHz mono 16‑bit
static constexpr uint8_t  AUDIO_SEC  = 1;        // duration per recording (s)

/********* Camera configuration (unchanged) *********/
static camera_config_t cfg = {
  /* --- Pins --- */
  .pin_pwdn     = PWDN_GPIO_NUM,
  .pin_reset    = RESET_GPIO_NUM,
  .pin_xclk     = XCLK_GPIO_NUM,
  .pin_sccb_sda = SIOD_GPIO_NUM,
  .pin_sccb_scl = SIOC_GPIO_NUM,
  .pin_d7       = Y9_GPIO_NUM,
  .pin_d6       = Y8_GPIO_NUM,
  .pin_d5       = Y7_GPIO_NUM,
  .pin_d4       = Y6_GPIO_NUM,
  .pin_d3       = Y5_GPIO_NUM,
  .pin_d2       = Y4_GPIO_NUM,
  .pin_d1       = Y3_GPIO_NUM,
  .pin_d0       = Y2_GPIO_NUM,
  .pin_vsync    = VSYNC_GPIO_NUM,
  .pin_href     = HREF_GPIO_NUM,
  .pin_pclk     = PCLK_GPIO_NUM,

  /* --- Clock & PWM --- */
  .xclk_freq_hz = 24'000'000,
  .ledc_timer   = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  /* --- **ab hier exakt die Reihenfolge aus esp_camera.h!** --- */
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size   = FRAMESIZE_QVGA,   // 320×240 → ≈90 KB
  .jpeg_quality = 12,               // 0‑63 (lower → better quality)
  .fb_count     = 2,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_WHEN_EMPTY
};

/********* IMU & ToF settings (unchanged) *********/
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU(ICM20948_ADDR);
VL53L1X      tof;

constexpr uint32_t PACKET_INTERVAL_MS = 1000;   // once per second
constexpr size_t   MAX_FIFO_SETS      = 120;    // safety margin
unsigned long lastPacketMillis = 0;

bool hasMag = false;
bool hasTOF = false;

struct IMUSample { float ax, ay, az, gx, gy, gz; };
IMUSample imuBuf[MAX_FIFO_SETS];
size_t    imuCount = 0;

/********* Globals *********/
I2SClass  i2s;
uint32_t  mediaCounter = 0;

/********* Forward declarations *********/
void setupIMU();
void setupTOF();
void buildAndStorePacket(uint32_t idx, uint32_t ts);
void captureAndStoreMedia(uint32_t idx, uint32_t ts);

/********* Arduino setup *********/
void setup() {
  delay(8000);                    // allow USB‑serial to connect
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);          // I2C Fast‑Mode (400 kHz)
  Serial.println("Booting …");

  /* ---- Sensors ---- */
  setupIMU();
  setupTOF();

  /* ---- Camera ---- */
  if (esp_camera_init(&cfg) == ESP_OK) {
    Serial.println("✅ Kamera bereit");
  } else {
    Serial.println("❌ Kamera‑Init fehlgeschlagen");
    for (;;) {}
  }

  /* ---- SD card ---- */
  Serial.println("Mounting SD…");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("❌ SD‑Karte nicht gefunden / mount fehlgeschlagen");
    for (;;) {}
  }
  Serial.println("✅ SD‑Karte bereit");

  /* ---- I2S microphone ---- */
  Serial.println("Initialising microphone …");
  i2s.setPinsPdmRx(MIC_DATA_PIN, MIC_CLK_PIN);
  if (!i2s.begin(I2S_MODE_PDM_RX, AUDIO_FS, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("❌ I2S‑Init fehlgeschlagen");
    for (;;) {}
  }
  Serial.println("✅ Mikrofon bereit");
}

/********* Arduino loop *********/
void loop() {
  if (millis() - lastPacketMillis >= PACKET_INTERVAL_MS) {
    lastPacketMillis += PACKET_INTERVAL_MS;    // maintain exact cadence

    uint32_t ts = millis();                    // timestamp in ms since boot
    buildAndStorePacket(mediaCounter, ts);     // JSON → Serial + SD
    captureAndStoreMedia(mediaCounter, ts);    // audio + photo → SD
    mediaCounter++;
  }

  // … other background tasks here …
}

/********* IMU init *********/
void setupIMU() {
  if (!myIMU.init()) {
    Serial.println("❌ ICM20948 does not respond");
    return;
  }
  Serial.println("✅ ICM20948 connected");

  if (myIMU.initMagnetometer()) {
    Serial.println("✅ Magnetometer ready (100 Hz)");
    hasMag = true;
  } else {
    Serial.println("⚠️  Magnetometer init failed – continuing without");
  }

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);

  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setGyrSampleRateDivider(10);

  myIMU.setFifoMode(ICM20948_CONTINUOUS);
  myIMU.enableFifo(true);
  delay(100);
  myIMU.startFifo(ICM20948_FIFO_ACC_GYR);
  Serial.println("FIFO started (ACC+GYR @ ≈100 Hz)");
}

/********* ToF init *********/
void setupTOF() {
  tof.setBus(&Wire);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(50000); // 50 ms
    tof.startContinuous(0);
    hasTOF = true;
    Serial.println("✅ VL53L1X ready (continuous)");
  } else {
    Serial.println("⚠️  VL53L1X init failed – continuing without");
  }
}

/********* JSON build, serial out & SD save *********/
void buildAndStorePacket(uint32_t idx, uint32_t ts) {
  myIMU.stopFifo();
  myIMU.findFifoBegin();
  imuCount = myIMU.getNumberOfFifoDataSets();
  if (imuCount > MAX_FIFO_SETS) imuCount = MAX_FIFO_SETS;
  for (size_t i = 0; i < imuCount; ++i) {
    xyzFloat acc, gyr;
    myIMU.getGValuesFromFifo(&acc);
    myIMU.getGyrValuesFromFifo(&gyr);
    imuBuf[i] = {acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z};
  }
  myIMU.resetFifo();
  myIMU.startFifo(ICM20948_FIFO_ACC_GYR);

  xyzFloat mag = {0, 0, 0};
  if (hasMag) {
    myIMU.readSensor();
    myIMU.getMagValues(&mag);
  }

  uint16_t tofRange = 0;
  uint8_t  tofStatus = 255;
  if (hasTOF) {
    if (tof.dataReady()) tof.read(false);
    tofRange  = tof.ranging_data.range_mm;
    tofStatus = static_cast<uint8_t>(tof.ranging_data.range_status);
  }

  DynamicJsonDocument doc(16384);
  doc["ts"] = ts;                // timestamp in ms
  JsonObject magObj = doc.createNestedObject("mag");
  magObj["x"] = mag.x;
  magObj["y"] = mag.y;
  magObj["z"] = mag.z;
  JsonObject tofObj = doc.createNestedObject("tof");
  tofObj["r"] = tofRange;
  tofObj["s"] = tofStatus;
  JsonArray imuArr = doc.createNestedArray("IMU");
  for (size_t i = 0; i < imuCount; ++i) {
    JsonObject e = imuArr.createNestedObject();
    e["i"] = static_cast<uint16_t>(i);
    JsonObject a = e.createNestedObject("a");
    a["x"] = imuBuf[i].ax;
    a["y"] = imuBuf[i].ay;
    a["z"] = imuBuf[i].az;
    JsonObject g = e.createNestedObject("g");
    g["x"] = imuBuf[i].gx;
    g["y"] = imuBuf[i].gy;
    g["z"] = imuBuf[i].gz;
  }

  /* --- serial out --- */
  serializeJson(doc, Serial);
  Serial.println();

  /* --- save to SD --- */
  char path[32];
  snprintf(path, sizeof(path), "/pkt_%lu.json", idx);
  File pktFile = SD.open(path, FILE_WRITE);
  if (pktFile) {
    serializeJson(doc, pktFile);
    pktFile.close();
    Serial.printf("📝  saved %s\n", path);
  } else {
    Serial.printf("❌  failed to open %s\n", path);
  }
}

/********* Audio + photo capture *********/
void captureAndStoreMedia(uint32_t idx, uint32_t ts) {
  char path[48];

  /* --- 1) Take photo --- */
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    snprintf(path, sizeof(path), "/img_%lu_%lu.jpg", idx, ts);
    File imgFile = SD.open(path, FILE_WRITE);
    if (imgFile) {
      imgFile.write(fb->buf, fb->len);
      imgFile.close();
      Serial.printf("📷  saved %s (%u bytes)\n", path, fb->len);
    } else {
      Serial.printf("❌  failed to open %s\n", path);
    }
    esp_camera_fb_return(fb);
  }

  /* --- 2) Record %u s audio --- */
  size_t   wavSize;
  uint8_t *wavBuf = i2s.recordWAV(AUDIO_SEC, &wavSize);   // blocking ≈1 s
  if (wavBuf && wavSize) {
    snprintf(path, sizeof(path), "/rec_%lu_%lu.wav", idx, ts);
    File wavFile = SD.open(path, FILE_WRITE);
    if (wavFile) {
      wavFile.write(wavBuf, wavSize);
      wavFile.close();
      Serial.printf("🎤  saved %s (%u bytes)\n", path, (unsigned)wavSize);
    } else {
      Serial.printf("❌  failed to open %s\n", path);
    }
    free(wavBuf);
  }
}
