// main_legacy.cpp — Legacy-Hardware-Variante: ICM20948 via I2C, VL53L1X Einzelpunkt-ToF
// Gleiche FreeRTOS-Architektur wie main.cpp, aber mit älterer Sensor-Bestückung.
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>

#include "esp_camera.h"
#include "camera_pins.h"   // XIAO Sense cam pins (CAMERA_MODEL_XIAO_ESP32S3)

#include "ESP_I2S.h"

I2SClass I2S;
#include "FS.h"
#include "SD.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define LOG_VERBOSE 0
#if LOG_VERBOSE
  #define LOGV(...)    Serial.printf(__VA_ARGS__)
  #define LOGV_LN(s)   Serial.println(s)
#else
  #define LOGV(...)    ((void)0)
  #define LOGV_LN(s)   ((void)0)
#endif

/********* Audio / SD configuration *********/
static constexpr int MIC_DATA_PIN = 42;
static constexpr int MIC_CLK_PIN  = 41;
static constexpr int SD_CS_PIN    = 21;
static constexpr uint16_t AUDIO_FS   = 16000;
static constexpr uint8_t  AUDIO_BITS = 16;
static constexpr uint8_t  AUDIO_CHAN = 1;
static constexpr uint32_t AUDIO_PCM_SIZE = AUDIO_FS * (AUDIO_BITS / 8) * AUDIO_CHAN;
static constexpr uint32_t WAV_HEADER_SIZE = 44;
static constexpr float AUDIO_HPF_ALPHA = 0.999f;
static constexpr float AUDIO_GAIN = 4.0f;

/********* Camera configuration *********/
static camera_config_t cfg = {
  .pin_pwdn     = PWDN_GPIO_NUM, .pin_reset    = RESET_GPIO_NUM,
  .pin_xclk     = XCLK_GPIO_NUM, .pin_sccb_sda = SIOD_GPIO_NUM,
  .pin_sccb_scl = SIOC_GPIO_NUM, .pin_d7       = Y9_GPIO_NUM,
  .pin_d6       = Y8_GPIO_NUM,   .pin_d5       = Y7_GPIO_NUM,
  .pin_d4       = Y6_GPIO_NUM,   .pin_d3       = Y5_GPIO_NUM,
  .pin_d2       = Y4_GPIO_NUM,   .pin_d1       = Y3_GPIO_NUM,
  .pin_d0       = Y2_GPIO_NUM,   .pin_vsync    = VSYNC_GPIO_NUM,
  .pin_href     = HREF_GPIO_NUM, .pin_pclk     = PCLK_GPIO_NUM,
  .xclk_freq_hz = 24000000,      .ledc_timer   = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG, .frame_size   = FRAMESIZE_QVGA,
  .jpeg_quality = 30, .fb_count     = 2,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_LATEST
};

/********* IMU & ToF settings *********/
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU(ICM20948_ADDR);
VL53L1X     tof;

constexpr uint32_t PACKET_INTERVAL_MS = 1000;
constexpr size_t   MAX_FIFO_SETS      = 120;
constexpr size_t   MAX_TOF_READINGS   = 16;
constexpr uint8_t  TOF_ZONES          = 1;   // VL53L1X: Einzelpunkt
constexpr uint32_t PACKETS_PER_BUCKET = 30;
constexpr size_t   MAX_ERRORS    = 8;
constexpr size_t   MAX_ERROR_LEN = 64;
constexpr uint32_t MID_PHOTO_TARGET_MS = 250;
constexpr uint32_t END_PHOTO_TARGET_MS = 750;

/********* Task & Queue Configuration *********/
static constexpr uint32_t JSON_DOC_SIZE = 16384;

static constexpr size_t PACKET_QUEUE_LEN    = 16;
static constexpr size_t TOF_QUEUE_LEN       = 20;
static constexpr size_t AUDIO_QUEUE_LEN     = 4;
static constexpr size_t MID_PHOTO_QUEUE_LEN = 2;
static constexpr size_t END_PHOTO_QUEUE_LEN = 2;

static constexpr uint32_t SAMPLER_STACK_SIZE   = 8192;
static constexpr uint32_t WRITER_STACK_SIZE    = 8192;
static constexpr uint32_t TOF_STACK_SIZE       = 4096;
static constexpr uint32_t AUDIO_STACK_SIZE     = 6144;
static constexpr uint32_t MID_PHOTO_STACK_SIZE = 6144;
static constexpr uint32_t END_PHOTO_STACK_SIZE = 6144;

static constexpr UBaseType_t AUDIO_PRIORITY      = 5;
static constexpr UBaseType_t SAMPLER_PRIORITY    = 4;
static constexpr UBaseType_t WRITER_PRIORITY     = 3;
static constexpr UBaseType_t TOF_PRIORITY        = 3;
static constexpr UBaseType_t MID_PHOTO_PRIORITY  = 4;
static constexpr UBaseType_t END_PHOTO_PRIORITY  = 4;

bool hasMag = false;
bool hasTOF = false;

static volatile uint32_t gMidPhotoWindowStartMs = 0;
static volatile uint32_t gEndPhotoWindowStartMs = 0;

struct IMUSample { float ax, ay, az, gx, gy, gz; };
struct TOFSample  { uint32_t ts_ms; int16_t distance_mm[TOF_ZONES]; uint8_t target_status[TOF_ZONES]; };
struct TOFReading { uint32_t ts_offset_ms; int16_t distance_mm[TOF_ZONES]; uint8_t target_status[TOF_ZONES]; };
struct MidPhoto { uint8_t* buf; size_t len; char reason[32]; uint32_t captureTs; uint32_t packetIdx; };
struct EndPhoto { uint8_t* buf; size_t len; char reason[32]; uint32_t captureTs; uint32_t packetIdx; };

/********* Globals *********/
uint32_t  mediaCounter = 0;
char      dataDir[16]  = "/data0";
QueueHandle_t gPacketQueue    = nullptr;
QueueHandle_t gTofQueue       = nullptr;
QueueHandle_t gAudioQueue     = nullptr;
QueueHandle_t gMidPhotoQueue  = nullptr;
QueueHandle_t gEndPhotoQueue  = nullptr;
TaskHandle_t gAudioTaskHandle    = nullptr;
TaskHandle_t gTofTaskHandle      = nullptr;
TaskHandle_t gMidPhotoTaskHandle = nullptr;
TaskHandle_t gEndPhotoTaskHandle = nullptr;

struct Packet {
  uint32_t idx;
  uint32_t ts_ms;
  size_t   imuCount;
  IMUSample imu[MAX_FIFO_SETS];
  xyzFloat mag;
  size_t   tofCount;
  TOFReading tof[MAX_TOF_READINGS];
  uint8_t *pcmBuf;
  size_t   pcmSize;
  uint8_t *midImgBuf;
  size_t   midImgLen;
  uint8_t *endImgBuf;
  size_t   endImgLen;
  uint32_t midCaptureTs;
  uint32_t endCaptureTs;
  size_t   errorCount;
  char     errorMsgs[MAX_ERRORS][MAX_ERROR_LEN];
};

/********* Forward declarations *********/
void setupIMU();
void setupTOF();
void setupAudio();
void audioTask(void* arg);
void tofTask(void* arg);
void samplerTask(void* arg);
void writerTask(void* arg);
void buildJson(const Packet& pkt, JsonDocument& doc);
void midPhotoTask(void* arg);
void endPhotoTask(void* arg);

/********* Arduino setup *********/
void setup() {
  delay(8000);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Booting (legacy hardware: I2C IMU + VL53L1X) …");

  setupIMU();
  setupTOF();
  setupAudio();

  if (esp_camera_init(&cfg) == ESP_OK) {
    Serial.println("OK. Camera init done");
  } else {
    Serial.println("ERR Camera init failed");
    while(1);
  }
  for (int i = 0; i < 5; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
  Serial.println("OK. Camera warm-up done");

  Serial.println("Mounting SD …");
  if (!SD.begin(SD_CS_PIN, SPI, 40000000)) {
    Serial.println("ERR SD mount failed");
    while(1);
  }
  Serial.printf("OK. SD ready (SPI @ 40 MHz, card size %llu MB)\n",
                SD.cardSize() / (1024ULL * 1024ULL));

  int n = 0;
  do {
    snprintf(dataDir, sizeof(dataDir), "/data%d", n++);
  } while (SD.exists(dataDir));

  for (int attempt = 0; ; attempt++) {
    if (SD.mkdir(dataDir)) {
      Serial.printf("OK. Created data directory: %s\n", dataDir);
      break;
    }
    Serial.printf("ERR failed to create %s (attempt %d/3)\n", dataDir, attempt + 1);
    if (attempt >= 2) break;
    delay(50);
  }

  gPacketQueue   = xQueueCreate(PACKET_QUEUE_LEN,    sizeof(Packet));
  gTofQueue      = xQueueCreate(TOF_QUEUE_LEN,       sizeof(TOFSample));
  gAudioQueue    = xQueueCreate(AUDIO_QUEUE_LEN,     sizeof(uint8_t*));
  gMidPhotoQueue = xQueueCreate(MID_PHOTO_QUEUE_LEN, sizeof(MidPhoto));
  gEndPhotoQueue = xQueueCreate(END_PHOTO_QUEUE_LEN, sizeof(EndPhoto));
  if (!gPacketQueue || !gTofQueue || !gAudioQueue || !gMidPhotoQueue || !gEndPhotoQueue) {
    Serial.println("ERR queue create failed");
    while(1);
  }

  BaseType_t ok1 = xTaskCreatePinnedToCore(samplerTask,  "sampler",  SAMPLER_STACK_SIZE,   nullptr, SAMPLER_PRIORITY,   nullptr,              0);
  BaseType_t ok2 = xTaskCreatePinnedToCore(writerTask,   "writer",   WRITER_STACK_SIZE,    nullptr, WRITER_PRIORITY,    nullptr,              1);
  BaseType_t ok3 = xTaskCreatePinnedToCore(tofTask,      "tof",      TOF_STACK_SIZE,       nullptr, TOF_PRIORITY,       &gTofTaskHandle,      0);
  BaseType_t ok4 = xTaskCreatePinnedToCore(audioTask,    "audio",    AUDIO_STACK_SIZE,     nullptr, AUDIO_PRIORITY,     &gAudioTaskHandle,    0);
  BaseType_t ok5 = xTaskCreatePinnedToCore(midPhotoTask, "midphoto", MID_PHOTO_STACK_SIZE, nullptr, MID_PHOTO_PRIORITY, &gMidPhotoTaskHandle, 1);
  BaseType_t ok6 = xTaskCreatePinnedToCore(endPhotoTask, "endphoto", END_PHOTO_STACK_SIZE, nullptr, END_PHOTO_PRIORITY, &gEndPhotoTaskHandle, 1);
  if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS || ok6 != pdPASS) {
    Serial.println("ERR task create failed");
    while(1);
  }
}

void loop() {
  vTaskDelete(NULL);
}

/********* Audio Init *********/
void setupAudio() {
  Serial.println("Initialising microphone …");
  I2S.setPinsPdmRx(42, 41);
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("ERR Failed to initialize I2S!");
    while(1);
  }
  Serial.println("OK. I2S driver ready.");
}

/********* Audio Task *********/
void audioTask(void* arg) {
  for (;;) {
    uint8_t* pcm_buf = (uint8_t*)malloc(AUDIO_PCM_SIZE);
    if (!pcm_buf) {
      Serial.println("ERR audioTask malloc failed");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    size_t bytes_read = I2S.readBytes((char*)pcm_buf, AUDIO_PCM_SIZE);
    if (bytes_read != AUDIO_PCM_SIZE) {
      Serial.printf("WARN I2S read only %u of %u bytes\n", bytes_read, (unsigned int)AUDIO_PCM_SIZE);
    }

    {
      static float hpf_x_prev = 0.0f;
      static float hpf_y_prev = 0.0f;
      int16_t* samples = reinterpret_cast<int16_t*>(pcm_buf);
      const size_t n = bytes_read / sizeof(int16_t);
      for (size_t i = 0; i < n; ++i) {
        float x = static_cast<float>(samples[i]);
        float y = x - hpf_x_prev + AUDIO_HPF_ALPHA * hpf_y_prev;
        hpf_x_prev = x;
        hpf_y_prev = y;
        float out = y * AUDIO_GAIN;
        if      (out >  32767.0f) out =  32767.0f;
        else if (out < -32768.0f) out = -32768.0f;
        samples[i] = static_cast<int16_t>(out);
      }
    }

    if (xQueueSend(gAudioQueue, &pcm_buf, 0) != pdPASS) {
      Serial.println("WARN audio queue full, dropping");
      free(pcm_buf);
    }
  }
}

/********* Mid-window Photo Task *********/
void midPhotoTask(void* arg) {
  for (;;) {
    uint32_t notifiedIdx = 0;
    xTaskNotifyWait(0, UINT32_MAX, &notifiedIdx, portMAX_DELAY);
    const uint32_t wStart = gMidPhotoWindowStartMs;

    const uint32_t target = wStart + MID_PHOTO_TARGET_MS;
    const uint32_t now = millis();
    if (now < target) vTaskDelay(pdMS_TO_TICKS(target - now));

    uint8_t* buf = nullptr;
    size_t   len = 0;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      buf = (uint8_t*)malloc(fb->len);
      if (buf) {
        memcpy(buf, fb->buf, fb->len);
        len = fb->len;
      }
      esp_camera_fb_return(fb);
    }

    MidPhoto mp = {};
    mp.buf       = buf;
    mp.len       = len;
    mp.captureTs = (fb && buf) ? millis() : 0;
    mp.packetIdx = notifiedIdx;
    if (!fb)       strncpy(mp.reason, "camera capture failed", sizeof(mp.reason) - 1);
    else if (!buf) strncpy(mp.reason, "malloc failed",         sizeof(mp.reason) - 1);
    if (xQueueSend(gMidPhotoQueue, &mp, pdMS_TO_TICKS(50)) != pdPASS) {
      if (buf) free(buf);
      Serial.println("ERR mid-photo queue full");
    }
  }
}

/********* End-of-window Photo Task *********/
void endPhotoTask(void* arg) {
  for (;;) {
    uint32_t notifiedIdx = 0;
    xTaskNotifyWait(0, UINT32_MAX, &notifiedIdx, portMAX_DELAY);
    const uint32_t wStart = gEndPhotoWindowStartMs;

    const uint32_t target = wStart + END_PHOTO_TARGET_MS;
    const uint32_t now = millis();
    if (now < target) vTaskDelay(pdMS_TO_TICKS(target - now));

    uint8_t* buf = nullptr;
    size_t   len = 0;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      buf = (uint8_t*)malloc(fb->len);
      if (buf) {
        memcpy(buf, fb->buf, fb->len);
        len = fb->len;
      }
      esp_camera_fb_return(fb);
    }

    EndPhoto ep = {};
    ep.buf       = buf;
    ep.len       = len;
    ep.captureTs = (fb && buf) ? millis() : 0;
    ep.packetIdx = notifiedIdx;
    if (!fb)       strncpy(ep.reason, "camera capture failed", sizeof(ep.reason) - 1);
    else if (!buf) strncpy(ep.reason, "malloc failed",         sizeof(ep.reason) - 1);
    if (xQueueSend(gEndPhotoQueue, &ep, pdMS_TO_TICKS(50)) != pdPASS) {
      if (buf) free(buf);
      Serial.println("ERR end-photo queue full");
    }
  }
}

/********* IMU init (I2C) *********/
void setupIMU() {
  if (!myIMU.init()) {
    Serial.println("ERR ICM20948 does not respond");
    return;
  }
  Serial.println("OK. ICM20948 connected via I2C (0x68)");

  if (myIMU.initMagnetometer()) {
    Serial.println("OK. Magnetometer ready");
    hasMag = true;
  } else {
    Serial.println("WARN Magnetometer init failed — continuing without");
  }

  myIMU.setAccRange(ICM20948_ACC_RANGE_8G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);

  myIMU.setGyrRange(ICM20948_GYRO_RANGE_1000);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setGyrSampleRateDivider(10);

  myIMU.setFifoMode(ICM20948_CONTINUOUS);
  myIMU.enableFifo(true);
  delay(100);
  myIMU.startFifo(ICM20948_FIFO_ACC_GYR);
  Serial.println("FIFO started (ACC+GYR @ ~100 Hz)");
}

/********* ToF init (VL53L1X) *********/
void setupTOF() {
  tof.setBus(&Wire);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(50000);
    tof.startContinuous(55); // period_ms must be >= timing budget (50 ms)
    hasTOF = true;
    Serial.println("OK. VL53L1X ready (continuous, long mode)");
  } else {
    Serial.println("WARN VL53L1X init failed — continuing without ToF");
  }
}

/********* ToF Task (VL53L1X polling) *********/
void tofTask(void* arg) {
  if (!hasTOF) vTaskDelete(NULL);

  for (;;) {
    if (tof.dataReady()) {
      tof.read(false);
      TOFSample sample;
      sample.ts_ms          = millis();
      sample.distance_mm[0] = tof.ranging_data.range_mm;
      sample.target_status[0] = static_cast<uint8_t>(tof.ranging_data.range_status);
      xQueueSend(gTofQueue, &sample, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/********* Sampler Task *********/
void samplerTask(void* arg) {
  const TickType_t periodTicks = pdMS_TO_TICKS(PACKET_INTERVAL_MS);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t windowStartMs = millis();

  myIMU.resetFifo();
  myIMU.startFifo(ICM20948_FIFO_ACC_GYR);
  { TOFSample tmp; while (xQueueReceive(gTofQueue, &tmp, 0) == pdPASS) {} }

  // Warmup-Fenster
  gEndPhotoWindowStartMs = windowStartMs;
  gMidPhotoWindowStartMs = windowStartMs;
  if (gEndPhotoTaskHandle) xTaskNotify(gEndPhotoTaskHandle, UINT32_MAX, eSetValueWithOverwrite);
  if (gMidPhotoTaskHandle) xTaskNotify(gMidPhotoTaskHandle, UINT32_MAX, eSetValueWithOverwrite);
  vTaskDelayUntil(&lastWake, periodTicks);

  { uint8_t* buf; while (xQueueReceive(gAudioQueue,    &buf, 0) == pdPASS) { free(buf); } }
  { MidPhoto mp;  while (xQueueReceive(gMidPhotoQueue, &mp,  0) == pdPASS) { if (mp.buf) free(mp.buf); } }
  { EndPhoto ep;  while (xQueueReceive(gEndPhotoQueue, &ep,  0) == pdPASS) { if (ep.buf) free(ep.buf); } }
  { TOFSample t;  while (xQueueReceive(gTofQueue,      &t,   0) == pdPASS) {} }
  myIMU.resetFifo();
  myIMU.startFifo(ICM20948_FIFO_ACC_GYR);
  windowStartMs += PACKET_INTERVAL_MS;

  gMidPhotoWindowStartMs = windowStartMs;
  if (gMidPhotoTaskHandle) xTaskNotify(gMidPhotoTaskHandle, mediaCounter, eSetValueWithOverwrite);

  for (;;) {
    gEndPhotoWindowStartMs = windowStartMs;
    if (gEndPhotoTaskHandle) xTaskNotify(gEndPhotoTaskHandle, mediaCounter, eSetValueWithOverwrite);

    vTaskDelayUntil(&lastWake, periodTicks);

    gMidPhotoWindowStartMs = windowStartMs + PACKET_INTERVAL_MS;
    if (gMidPhotoTaskHandle) xTaskNotify(gMidPhotoTaskHandle, mediaCounter + 1, eSetValueWithOverwrite);
    LOGV("Sampler task: Window end at %lu ms.\n", (unsigned long)millis());

    static Packet pkt;
    pkt = {};
    pkt.idx = mediaCounter;
    pkt.ts_ms = windowStartMs;

    auto addError = [](const char* msg) {
      if (pkt.errorCount < MAX_ERRORS)
        strncpy(pkt.errorMsgs[pkt.errorCount++], msg, MAX_ERROR_LEN - 1);
    };

    uint8_t* pcmBuf = nullptr;
    if (xQueueReceive(gAudioQueue, &pcmBuf, pdMS_TO_TICKS(200)) == pdPASS) {
      pkt.pcmBuf = pcmBuf;
      pkt.pcmSize = AUDIO_PCM_SIZE;
    } else {
      Serial.println("WARN sampler task missing audio buffer.");
      addError("audio: buffer missing (queue timeout)");
      pkt.pcmBuf = nullptr;
      pkt.pcmSize = 0;
    }

    {
      int16_t rawCount = myIMU.getNumberOfFifoDataSets();
      if (rawCount < 0) {
        addError("IMU: FIFO read error (negative count)");
      } else if (rawCount == 0) {
        addError("IMU: FIFO empty at window end");
      }
      size_t count = (rawCount > 0) ? static_cast<size_t>(rawCount) : 0;
      if (count > MAX_FIFO_SETS) count = MAX_FIFO_SETS;
      for (size_t i = 0; i < count; ++i) {
        xyzFloat acc, gyr;
        myIMU.getGValuesFromFifo(&acc);
        myIMU.getGyrValuesFromFifo(&gyr);
        pkt.imu[i] = {acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z};
      }
      pkt.imuCount = count;
    }

    pkt.mag = {0,0,0};
    if (hasMag) {
      myIMU.readSensor();
      myIMU.getMagValues(&pkt.mag);
    }

    pkt.tofCount = 0;
    if (hasTOF) {
      TOFSample sample;
      while (xQueueReceive(gTofQueue, &sample, 0) == pdPASS && pkt.tofCount < MAX_TOF_READINGS) {
        pkt.tof[pkt.tofCount].ts_offset_ms  = sample.ts_ms - pkt.ts_ms;
        pkt.tof[pkt.tofCount].distance_mm[0]    = sample.distance_mm[0];
        pkt.tof[pkt.tofCount].target_status[0]  = sample.target_status[0];
        pkt.tofCount++;
      }
    }

    {
      MidPhoto mp = {};
      bool gotMid = false;
      while (xQueueReceive(gMidPhotoQueue, &mp, 0) == pdPASS) {
        if (mp.packetIdx == pkt.idx) { gotMid = true; break; }
        if (mp.buf) free(mp.buf);
        char msg[MAX_ERROR_LEN];
        snprintf(msg, sizeof(msg), "mid-photo: stale idx=%lu verworfen", (unsigned long)mp.packetIdx);
        addError(msg);
        mp = {};
      }
      if (gotMid) {
        pkt.midImgBuf    = mp.buf;
        pkt.midImgLen    = mp.len;
        pkt.midCaptureTs = mp.captureTs;
        if (!mp.buf) {
          char msg[MAX_ERROR_LEN];
          snprintf(msg, sizeof(msg), "mid-photo: %s", mp.reason[0] ? mp.reason : "null buf");
          addError(msg);
        }
      } else {
        addError("mid-photo: not in queue at window end");
      }
    }
    {
      EndPhoto ep = {};
      bool gotEnd = false;
      while (xQueueReceive(gEndPhotoQueue, &ep, 0) == pdPASS) {
        if (ep.packetIdx == pkt.idx) { gotEnd = true; break; }
        if (ep.buf) free(ep.buf);
        char msg[MAX_ERROR_LEN];
        snprintf(msg, sizeof(msg), "end-photo: stale idx=%lu verworfen", (unsigned long)ep.packetIdx);
        addError(msg);
        ep = {};
      }
      if (gotEnd) {
        pkt.endImgBuf    = ep.buf;
        pkt.endImgLen    = ep.len;
        pkt.endCaptureTs = ep.captureTs;
        if (!ep.buf) {
          char msg[MAX_ERROR_LEN];
          snprintf(msg, sizeof(msg), "end-photo: %s", ep.reason[0] ? ep.reason : "null buf");
          addError(msg);
        }
      } else {
        addError("end-photo: not in queue at window end");
      }
    }

    windowStartMs += PACKET_INTERVAL_MS;

    if (xQueueSend(gPacketQueue, &pkt, pdMS_TO_TICKS(50)) != pdPASS) {
      if (pkt.pcmBuf)    free(pkt.pcmBuf);
      if (pkt.midImgBuf) free(pkt.midImgBuf);
      if (pkt.endImgBuf) free(pkt.endImgBuf);
      Serial.println("WARN packet queue full — dropping packet");
    } else {
      LOGV_LN("Sampler task: Packet sent to writer.");
      mediaCounter++;
    }
  }
}

/********* JSON helper *********/
void buildJson(const Packet& pkt, JsonDocument& doc) {
  doc["ts"] = pkt.ts_ms;
  JsonObject magObj = doc["mag"].to<JsonObject>();
  magObj["x"] = pkt.mag.x;
  magObj["y"] = pkt.mag.y;
  magObj["z"] = pkt.mag.z;

  JsonArray tofArr = doc["tof"].to<JsonArray>();
  for (size_t i = 0; i < pkt.tofCount; ++i) {
    JsonObject e   = tofArr.add<JsonObject>();
    e["t"]         = pkt.tof[i].ts_offset_ms;
    JsonArray dArr = e["d"].to<JsonArray>();
    JsonArray sArr = e["s"].to<JsonArray>();
    dArr.add(pkt.tof[i].distance_mm[0]);
    sArr.add(pkt.tof[i].target_status[0]);
  }

  JsonArray imuArr = doc["IMU"].to<JsonArray>();
  for (size_t i = 0; i < pkt.imuCount; ++i) {
    JsonObject e = imuArr.add<JsonObject>();
    e["i"] = static_cast<uint16_t>(i);
    JsonObject a = e["a"].to<JsonObject>();
    a["x"] = pkt.imu[i].ax;
    a["y"] = pkt.imu[i].ay;
    a["z"] = pkt.imu[i].az;
    JsonObject g = e["g"].to<JsonObject>();
    g["x"] = pkt.imu[i].gx;
    g["y"] = pkt.imu[i].gy;
    g["z"] = pkt.imu[i].gz;
  }

  if (pkt.midCaptureTs > 0)
    doc["midTs"] = (int32_t)(pkt.midCaptureTs - pkt.ts_ms);
  if (pkt.endCaptureTs > 0)
    doc["endTs"] = (int32_t)(pkt.endCaptureTs - pkt.ts_ms);

  if (pkt.errorCount > 0) {
    JsonArray errArr = doc["errors"].to<JsonArray>();
    for (size_t i = 0; i < pkt.errorCount; i++)
      errArr.add(pkt.errorMsgs[i]);
  }
}

/********* Writer Task *********/
void writerTask(void* arg) {
  static Packet pkt;
  char bucketDir[32];
  uint32_t currentBucket = UINT32_MAX;
  for (;;) {
    pkt = {};
    if (xQueueReceive(gPacketQueue, &pkt, portMAX_DELAY) != pdPASS) continue;

    const uint32_t writeStart = millis();

    const uint32_t bucket = pkt.idx / PACKETS_PER_BUCKET;
    if (bucket != currentBucket) {
      snprintf(bucketDir, sizeof(bucketDir), "%s/%04lu", dataDir, (unsigned long)bucket);
      if (!SD.exists(bucketDir)) {
        for (int attempt = 0; ; attempt++) {
          if (SD.mkdir(bucketDir)) break;
          Serial.printf("ERR failed to create bucket %s (attempt %d/3)\n", bucketDir, attempt + 1);
          if (attempt >= 2) break;
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }
      currentBucket = bucket;
    }

    JsonDocument doc;
    buildJson(pkt, doc);

    LOGV("PKT %lu  ts=%lu  imu=%u  tof=%u  audio=%u\n",
         pkt.idx, pkt.ts_ms,
         (unsigned)pkt.imuCount, (unsigned)pkt.tofCount,
         (unsigned)pkt.pcmSize);

    char path[64];
    snprintf(path, sizeof(path), "%s/pkt_%lu.json", bucketDir, pkt.idx);
    if (File f = SD.open(path, FILE_WRITE)) {
      serializeJson(doc, f);
      f.close();
      LOGV("OK  saved %s\n", path);
    } else {
      Serial.printf("ERR failed to open %s\n", path);
    }

    if (pkt.pcmBuf && pkt.pcmSize > 0) {
      snprintf(path, sizeof(path), "%s/rec_%lu_%lu.wav", bucketDir, pkt.idx, pkt.ts_ms);
      if (File wf = SD.open(path, FILE_WRITE)) {
        uint8_t hdr[WAV_HEADER_SIZE];
        const uint32_t data_size   = pkt.pcmSize;
        const uint32_t chunk_size  = WAV_HEADER_SIZE - 8 + data_size;
        const uint32_t byte_rate   = AUDIO_FS * AUDIO_CHAN * (AUDIO_BITS / 8);
        const uint16_t block_align = AUDIO_CHAN * (AUDIO_BITS / 8);
        memcpy(hdr +  0, "RIFF",           4);
        memcpy(hdr +  4, &chunk_size,       4);
        memcpy(hdr +  8, "WAVE",           4);
        memcpy(hdr + 12, "fmt ",           4);
        const uint32_t fmt_size = 16;      memcpy(hdr + 16, &fmt_size,   4);
        const uint16_t pcm_fmt  = 1;       memcpy(hdr + 20, &pcm_fmt,    2);
        const uint16_t channels = AUDIO_CHAN; memcpy(hdr + 22, &channels, 2);
        const uint32_t sample_rate = AUDIO_FS; memcpy(hdr + 24, &sample_rate, 4);
        memcpy(hdr + 28, &byte_rate,        4);
        memcpy(hdr + 32, &block_align,      2);
        const uint16_t bits = AUDIO_BITS;  memcpy(hdr + 34, &bits,       2);
        memcpy(hdr + 36, "data",           4);
        memcpy(hdr + 40, &data_size,        4);
        wf.write(hdr, WAV_HEADER_SIZE);
        wf.write(pkt.pcmBuf, pkt.pcmSize);
        wf.close();
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.pcmBuf);
    }

    if (pkt.endImgBuf && pkt.endImgLen > 0) {
      snprintf(path, sizeof(path), "%s/img_%lu_%lu.jpg", bucketDir, pkt.idx, pkt.ts_ms);
      if (File img = SD.open(path, FILE_WRITE)) {
        img.write(pkt.endImgBuf, pkt.endImgLen);
        img.close();
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.endImgBuf);
    }

    if (pkt.midImgBuf && pkt.midImgLen > 0) {
      snprintf(path, sizeof(path), "%s/mid_%lu_%lu.jpg", bucketDir, pkt.idx, pkt.ts_ms);
      if (File mf = SD.open(path, FILE_WRITE)) {
        mf.write(pkt.midImgBuf, pkt.midImgLen);
        mf.close();
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.midImgBuf);
    }

    const uint32_t writeMs = millis() - writeStart;
    if (writeMs > 950) {
      Serial.printf("WARN writer slow: pkt %lu took %lu ms\n", pkt.idx, writeMs);
    }
  }
}
