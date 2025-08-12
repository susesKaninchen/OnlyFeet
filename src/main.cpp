// main.cpp — FreeRTOS-scheduled sensor logger (ESP32S3 Sense)
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>

#include "esp_camera.h"
#include "camera_pins.h"   // XIAO Sense cam pins (CAMERA_MODEL_XIAO_ESP32S3)

#include "ESP_I2S.h"       // PDM MIC
#include "FS.h"
#include "SD.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/********* Audio / SD configuration *********/
static constexpr int MIC_DATA_PIN = 42;   // PDM DAT  (GPIO42)
static constexpr int MIC_CLK_PIN  = 41;   // PDM CLK  (GPIO41)
static constexpr int SD_CS_PIN    = 21;   // SD CS    (GPIO21)
static constexpr uint16_t AUDIO_FS   = 16'000;   // 16 kHz mono 16-bit
static constexpr uint8_t  AUDIO_SEC  = 1;        // duration per recording (s)

/********* Camera configuration *********/
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

  /* --- follow esp_camera.h field order --- */
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size   = FRAMESIZE_QVGA,   // 320x240 ~ 40–90 KB
  .jpeg_quality = 12,               // 0–63 (lower = better quality)
  .fb_count     = 2,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_WHEN_EMPTY
};

/********* IMU & ToF settings *********/
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU(ICM20948_ADDR);
VL53L1X      tof;

constexpr uint32_t PACKET_INTERVAL_MS = 1000;   // once per second
constexpr size_t   MAX_FIFO_SETS      = 120;    // safety margin

bool hasMag = false;
bool hasTOF = false;

struct IMUSample { float ax, ay, az, gx, gy, gz; };

/********* Globals *********/
I2SClass  i2s;
uint32_t  mediaCounter = 0;
QueueHandle_t gPacketQueue = nullptr;

struct Packet {
  uint32_t idx;
  uint32_t ts_ms;      // window start timestamp (ms since boot)
  size_t   imuCount;
  IMUSample imu[MAX_FIFO_SETS];
  xyzFloat mag;        // magnetometer at end-of-window (if available)
  uint16_t tofRange;   // mm
  uint8_t  tofStatus;
  uint8_t *wavBuf;     // malloc'ed WAV buffer (ownership passed to writer)
  size_t   wavSize;
};

/********* Forward declarations *********/
void setupIMU();
void setupTOF();
void samplerTask(void* arg);
void writerTask(void* arg);

/********* Arduino setup *********/
void setup() {
  delay(8000);                    // allow USB-serial to connect
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);          // I2C Fast-Mode (400 kHz)
  Serial.println("Booting …");

  /* ---- Sensors ---- */
  setupIMU();
  setupTOF();

  /* ---- Camera ---- */
  if (esp_camera_init(&cfg) == ESP_OK) {
    Serial.println("OK. Camera ready");
  } else {
    Serial.println("ERR Camera init failed");
    for (;;) {}
  }

  /* ---- SD card ---- */
  Serial.println("Mounting SD …");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERR SD mount failed");
    for (;;) {}
  }
  Serial.println("OK. SD ready");

  /* ---- I2S microphone ---- */
  Serial.println("Initialising microphone …");
  i2s.setPinsPdmRx(MIC_DATA_PIN, MIC_CLK_PIN);
  if (!i2s.begin(I2S_MODE_PDM_RX, AUDIO_FS, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("ERR I2S init failed");
    for (;;) {}
  }
  Serial.println("OK. Microphone ready");

  // Create queue and tasks
  gPacketQueue = xQueueCreate(2, sizeof(Packet));
  if (!gPacketQueue) {
    Serial.println("ERR queue create failed");
    for(;;) {}
  }

  BaseType_t ok1 = xTaskCreatePinnedToCore(samplerTask, "sampler", 8192, nullptr, 3, nullptr, 0);
  BaseType_t ok2 = xTaskCreatePinnedToCore(writerTask,  "writer",  8192, nullptr, 2, nullptr, 1);
  if (ok1 != pdPASS || ok2 != pdPASS) {
    Serial.println("ERR task create failed");
    for(;;) {}
  }
}

/********* Arduino loop *********/
void loop() {
  // All work is handled in FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/********* IMU init *********/
void setupIMU() {
  if (!myIMU.init()) {
    Serial.println("ERR ICM20948 does not respond");
    return;
  }
  Serial.println("OK. ICM20948 connected");

  if (myIMU.initMagnetometer()) {
    Serial.println("OK. Magnetometer ready (100 Hz)");
    hasMag = true;
  } else {
    Serial.println("WARN Magnetometer init failed — continuing without");
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
  Serial.println("FIFO started (ACC+GYR @ ~100 Hz)");
}

/********* ToF init *********/
void setupTOF() {
  tof.setBus(&Wire);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(50000); // 50 ms
    tof.startContinuous(0);
    hasTOF = true;
    Serial.println("OK. VL53L1X ready (continuous)");
  } else {
    Serial.println("WARN VL53L1X init failed — continuing without");
  }
}

/********* Sampler Task (precise 1 s windows) *********/
void samplerTask(void* arg) {
  const TickType_t periodTicks = pdMS_TO_TICKS(PACKET_INTERVAL_MS);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // Window start aligned to FreeRTOS tick
    uint32_t ts = millis();

    // Prepare IMU FIFO for the coming window
    myIMU.resetFifo();
    myIMU.startFifo(ICM20948_FIFO_ACC_GYR);

    // Record audio for exactly AUDIO_SEC seconds (blocking in this task)
    size_t wavSize = 0;
    uint8_t* wavBuf = i2s.recordWAV(AUDIO_SEC, &wavSize);

    // End of window: stop IMU FIFO and read out samples collected in this second
    myIMU.stopFifo();
    myIMU.findFifoBegin();
    size_t count = myIMU.getNumberOfFifoDataSets();
    if (count > MAX_FIFO_SETS) count = MAX_FIFO_SETS;

    Packet pkt{};
    pkt.idx = mediaCounter;
    pkt.ts_ms = ts;
    pkt.imuCount = count;
    for (size_t i = 0; i < count; ++i) {
      xyzFloat acc, gyr;
      myIMU.getGValuesFromFifo(&acc);
      myIMU.getGyrValuesFromFifo(&gyr);
      pkt.imu[i] = {acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z};
    }

    // Optional sensors at end-of-window
    pkt.mag = {0,0,0};
    if (hasMag) {
      myIMU.readSensor();
      myIMU.getMagValues(&pkt.mag);
    }

    pkt.tofRange = 0;
    pkt.tofStatus = 255;
    if (hasTOF) {
      if (tof.dataReady()) tof.read(false);
      pkt.tofRange  = tof.ranging_data.range_mm;
      pkt.tofStatus = static_cast<uint8_t>(tof.ranging_data.range_status);
    }

    pkt.wavBuf  = wavBuf;
    pkt.wavSize = wavSize;

    // Enqueue for writer; writer will save JSON, audio, and capture a photo
    if (xQueueSend(gPacketQueue, &pkt, pdMS_TO_TICKS(50)) != pdPASS) {
      // Drop on queue full; free audio buffer to avoid leak
      if (wavBuf) free(wavBuf);
      Serial.println("WARN queue full — dropping packet");
    } else {
      mediaCounter++;
    }

    // Maintain strict cadence
    vTaskDelayUntil(&lastWake, periodTicks);
  }
}

/********* Writer Task (SD + Serial + Photo) *********/
void writerTask(void* arg) {
  for (;;) {
    Packet pkt;
    if (xQueueReceive(gPacketQueue, &pkt, portMAX_DELAY) != pdPASS) {
      continue;
    }

    // 1) Build JSON from packet
    DynamicJsonDocument doc(16384);
    doc["ts"] = pkt.ts_ms;
    JsonObject magObj = doc.createNestedObject("mag");
    magObj["x"] = pkt.mag.x;
    magObj["y"] = pkt.mag.y;
    magObj["z"] = pkt.mag.z;
    JsonObject tofObj = doc.createNestedObject("tof");
    tofObj["r"] = pkt.tofRange;
    tofObj["s"] = pkt.tofStatus;
    JsonArray imuArr = doc.createNestedArray("IMU");
    for (size_t i = 0; i < pkt.imuCount; ++i) {
      JsonObject e = imuArr.createNestedObject();
      e["i"] = static_cast<uint16_t>(i);
      JsonObject a = e.createNestedObject("a");
      a["x"] = pkt.imu[i].ax;
      a["y"] = pkt.imu[i].ay;
      a["z"] = pkt.imu[i].az;
      JsonObject g = e.createNestedObject("g");
      g["x"] = pkt.imu[i].gx;
      g["y"] = pkt.imu[i].gy;
      g["z"] = pkt.imu[i].gz;
    }

    // Serial out
    serializeJson(doc, Serial);
    Serial.println();

    // 2) Save JSON
    char path[48];
    snprintf(path, sizeof(path), "/pkt_%lu.json", pkt.idx);
    if (File f = SD.open(path, FILE_WRITE)) {
      serializeJson(doc, f);
      f.close();
      Serial.printf("OK  saved %s\n", path);
    } else {
      Serial.printf("ERR failed to open %s\n", path);
    }

    // 3) Save audio WAV
    if (pkt.wavBuf && pkt.wavSize) {
      snprintf(path, sizeof(path), "/rec_%lu_%lu.wav", pkt.idx, pkt.ts_ms);
      if (File wf = SD.open(path, FILE_WRITE)) {
        wf.write(pkt.wavBuf, pkt.wavSize);
        wf.close();
        Serial.printf("OK  saved %s (%u bytes)\n", path, (unsigned)pkt.wavSize);
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.wavBuf);
    }

    // 4) Capture and save photo (non-critical timing)
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      snprintf(path, sizeof(path), "/img_%lu_%lu.jpg", pkt.idx, pkt.ts_ms);
      if (File img = SD.open(path, FILE_WRITE)) {
        img.write(fb->buf, fb->len);
        img.close();
        Serial.printf("OK  saved %s (%u bytes)\n", path, fb->len);
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      esp_camera_fb_return(fb);
    }
  }
}

