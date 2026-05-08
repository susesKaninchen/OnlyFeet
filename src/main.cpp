// main.cpp — FreeRTOS-scheduled sensor logger (ESP32S3 Sense)
// Optionaler WiFi-Web-Server-Modus: beim Boot nach bekanntem Hotspot suchen.
// Batterie-ADC auf A0/GPIO1 (200 kΩ:200 kΩ Spannungsteiler gegen BAT+).
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <ArduinoJson.h>

#include "esp_camera.h"
#include "camera_pins.h"

#include "ESP_I2S.h"
I2SClass I2S;
#include "FS.h"
#include "SD.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WebServer.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Brownout-Detektor
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define LOG_VERBOSE 0
#if LOG_VERBOSE
  #define LOGV(...)    Serial.printf(__VA_ARGS__)
  #define LOGV_LN(s)   Serial.println(s)
#else
  #define LOGV(...)    ((void)0)
  #define LOGV_LN(s)   ((void)0)
#endif

/********* WiFi credentials *********/
#define WIFI_SSID_DEFAULT "MeinHotspot"
#define WIFI_PASS_DEFAULT "MeinPasswort"
static char gWifiSSID[64] = WIFI_SSID_DEFAULT;
static char gWifiPass[64] = WIFI_PASS_DEFAULT;

/********* Audio / SD / Battery *********/
static constexpr int MIC_DATA_PIN    = 42;
static constexpr int MIC_CLK_PIN     = 41;
static constexpr int SD_CS_PIN       = 21;
// A0/GPIO1 mit 200 kΩ:200 kΩ Teiler an BAT+  (IMU-CS auf GPIO43 verlegt)
static constexpr int BATTERY_ADC_PIN = 1;
static constexpr uint16_t AUDIO_FS   = 16000;
static constexpr uint8_t  AUDIO_BITS = 16;
static constexpr uint8_t  AUDIO_CHAN = 1;
static constexpr uint32_t AUDIO_PCM_SIZE  = AUDIO_FS * (AUDIO_BITS / 8) * AUDIO_CHAN;
static constexpr uint32_t WAV_HEADER_SIZE = 44;
static constexpr float AUDIO_HPF_ALPHA = 0.999f;
static constexpr float AUDIO_GAIN      = 4.0f;

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
  .xclk_freq_hz = 20000000,      .ledc_timer   = LEDC_TIMER_0,  // 20 MHz — OV2640 + OV3660 kompatibel
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG, .frame_size   = FRAMESIZE_QVGA,
  .jpeg_quality = 30, .fb_count   = 2,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_LATEST
};

/********* IMU & ToF settings *********/
// CS auf GPIO43 (D6, UART-TX) verlegt → GPIO1 (A0) steht als ADC-Eingang frei
static constexpr int IMU_SPI_CS_PIN   = 43;  // D6 — Hardware-Draht von D0 auf D6 umlegen!
static constexpr int IMU_SPI_MOSI_PIN = 2;   // D1
static constexpr int IMU_SPI_MISO_PIN = 3;   // D2
static constexpr int IMU_SPI_SCK_PIN  = 4;   // D3
static SPIClass imuSPI(HSPI);
ICM20948_WE      myIMU(&imuSPI, IMU_SPI_CS_PIN, IMU_SPI_MOSI_PIN,
                       IMU_SPI_MISO_PIN, IMU_SPI_SCK_PIN, true);
SparkFun_VL53L5CX myImager;

constexpr uint32_t PACKET_INTERVAL_MS = 1000;
constexpr size_t   MAX_FIFO_SETS      = 120;
constexpr size_t   MAX_TOF_READINGS   = 16;
constexpr uint8_t  TOF_ZONES          = 64;
constexpr bool     ENABLE_TOF         = true;
constexpr uint32_t PACKETS_PER_BUCKET = 30;
constexpr size_t   MAX_ERRORS         = 8;
constexpr size_t   MAX_ERROR_LEN      = 64;
constexpr uint32_t MID_PHOTO_TARGET_MS = 250;
constexpr uint32_t END_PHOTO_TARGET_MS = 750;

/********* Task & Queue Configuration *********/
static constexpr size_t PACKET_QUEUE_LEN    = 16;
static constexpr size_t TOF_QUEUE_LEN       = 20;
static constexpr size_t AUDIO_QUEUE_LEN     = 4;
static constexpr size_t MID_PHOTO_QUEUE_LEN = 2;
static constexpr size_t END_PHOTO_QUEUE_LEN = 2;

static constexpr uint32_t SAMPLER_STACK_SIZE   = 8192;
static constexpr uint32_t WRITER_STACK_SIZE    = 8192;
static constexpr uint32_t TOF_STACK_SIZE       = 6144;
static constexpr uint32_t AUDIO_STACK_SIZE     = 6144;
static constexpr uint32_t MID_PHOTO_STACK_SIZE = 6144;
static constexpr uint32_t END_PHOTO_STACK_SIZE = 6144;
static constexpr uint32_t WEB_STACK_SIZE       = 12288;

static constexpr UBaseType_t AUDIO_PRIORITY     = 5;
static constexpr UBaseType_t SAMPLER_PRIORITY   = 4;
static constexpr UBaseType_t MID_PHOTO_PRIORITY = 4;
static constexpr UBaseType_t END_PHOTO_PRIORITY = 4;
static constexpr UBaseType_t WRITER_PRIORITY    = 3;
static constexpr UBaseType_t TOF_PRIORITY       = 3;
static constexpr UBaseType_t WEB_PRIORITY       = 2;

/********* System Log *********/
struct LogEntry { uint32_t ts_ms; char msg[80]; };
static LogEntry gLog[20];
static uint8_t  gLogCount = 0;

void sysLog(const char* msg) {
  Serial.println(msg);
  if (gLogCount < 20) {
    gLog[gLogCount].ts_ms = millis();
    strncpy(gLog[gLogCount].msg, msg, 79);
    gLog[gLogCount].msg[79] = '\0';
    gLogCount++;
  }
}

/********* Init flags *********/
static bool gInitSD     = false;
static bool gInitCamera = false;
static bool gInitAudio  = false;
static bool gInitIMU    = false;
bool hasMag = false;
bool hasTOF = false;

/********* Shared web state *********/
struct WebStatus {
  float ax, ay, az, gx, gy, gz;
  float mag_x, mag_y, mag_z;
  float battery_mv;
  int   battery_pct;
  int16_t tof_center_mm;  // Mittelwert der 4 Zentrums-Zonen (8×8, Zonen 27/28/35/36)
  bool    tof_valid;
  uint32_t uptime_ms;
  uint32_t packet_count;
};
static WebStatus gWebStatus = {};
static volatile bool gRecording = false;
static volatile bool gWifiMode  = false;
static SemaphoreHandle_t gSdMutex = nullptr;

static volatile uint32_t gMidPhotoWindowStartMs = 0;
static volatile uint32_t gEndPhotoWindowStartMs = 0;

struct IMUSample  { float ax, ay, az, gx, gy, gz; };
struct TOFSample  { uint32_t ts_ms; int16_t distance_mm[TOF_ZONES]; uint8_t target_status[TOF_ZONES]; };
struct TOFReading { uint32_t ts_offset_ms; int16_t distance_mm[TOF_ZONES]; uint8_t target_status[TOF_ZONES]; };
struct MidPhoto   { uint8_t* buf; size_t len; char reason[32]; uint32_t captureTs; uint32_t packetIdx; };
struct EndPhoto   { uint8_t* buf; size_t len; char reason[32]; uint32_t captureTs; uint32_t packetIdx; };

/********* Globals *********/
uint32_t  mediaCounter = 0;
char      dataDir[16]  = "/data0";
QueueHandle_t gPacketQueue    = nullptr;
QueueHandle_t gTofQueue       = nullptr;
QueueHandle_t gAudioQueue     = nullptr;
QueueHandle_t gMidPhotoQueue  = nullptr;
QueueHandle_t gEndPhotoQueue  = nullptr;
TaskHandle_t  gAudioTaskHandle    = nullptr;
TaskHandle_t  gTofTaskHandle      = nullptr;
TaskHandle_t  gMidPhotoTaskHandle = nullptr;
TaskHandle_t  gEndPhotoTaskHandle = nullptr;

struct Packet {
  uint32_t idx;
  uint32_t ts_ms;
  size_t   imuCount;
  IMUSample imu[MAX_FIFO_SETS];
  xyzFloat  mag;
  size_t    tofCount;
  TOFReading tof[MAX_TOF_READINGS];
  uint8_t  *pcmBuf;
  size_t    pcmSize;
  uint8_t  *midImgBuf;
  size_t    midImgLen;
  uint8_t  *endImgBuf;
  size_t    endImgLen;
  uint32_t  midCaptureTs;
  uint32_t  endCaptureTs;
  size_t    errorCount;
  char      errorMsgs[MAX_ERRORS][MAX_ERROR_LEN];
};

/********* Forward declarations *********/
void setupIMU();
void setupTOF();
void setupAudio();
void loadWifiConfig();
void audioTask(void* arg);
void tofTask(void* arg);
void samplerTask(void* arg);
void writerTask(void* arg);
void webTask(void* arg);
void buildJson(const Packet& pkt, JsonDocument& doc);
void midPhotoTask(void* arg);
void endPhotoTask(void* arg);

/********* CRC-32 (für ZIP-Download) *********/
static uint32_t crc32_update(uint32_t crc, const uint8_t* buf, size_t len) {
  crc ^= 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
  }
  return crc ^ 0xFFFFFFFFu;
}

/********* Arduino setup *********/
void setup() {
  // Brownout-Detektor komplett deaktivieren — LiPo-Spannungsdips beim Sensor-Peak
  // sollen keinen Reset auslösen. Battery-Management-IC schützt den Akku selbst.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  delay(1000);  // USB-CDC Enumeration
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  sysLog("Booting (main: SPI IMU + VL53L5CX 8x8) ...");

  setupIMU();
  setupTOF();
  setupAudio();

  if (esp_camera_init(&cfg) == ESP_OK) {
    gInitCamera = true;
    sysLog("OK. Camera init done");
  } else {
    sysLog("ERR Camera init failed");
    while(1);
  }
  // 15 Warmup-Frames × 50 ms: OV2640 + OV3660 benötigen mehrere Frames
  // bis Auto-Exposure einschwingt und fb_get() schnell zurückkommt.
  for (int i = 0; i < 15; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(50);
  }
  sysLog("OK. Camera warm-up done");

  sysLog("Mounting SD ...");
  if (!SD.begin(SD_CS_PIN, SPI, 40000000)) {
    sysLog("ERR SD mount failed");
    while(1);
  }
  gInitSD = true;
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "OK. SD ready (%llu MB)", SD.cardSize() / (1024ULL * 1024ULL));
    sysLog(buf);
  }

  loadWifiConfig();

  // Initiale Batteriemessung (vor Tasks, damit /api/status sofort einen Wert zeigt)
  {
    int mv = analogReadMilliVolts(BATTERY_ADC_PIN);
    gWebStatus.battery_mv  = mv * 2.0f;
    gWebStatus.battery_pct = constrain((int)((gWebStatus.battery_mv - 3000.0f) / 12.0f), 0, 100);
  }

  // WiFi-Scan ersetzt delay(8000) — sucht nach konfiguriertem Hotspot
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  sysLog("Scanning for WiFi hotspot ...");
  bool wifiFound = false;
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == gWifiSSID) { wifiFound = true; break; }
  }

  if (wifiFound) {
    char buf[80];
    snprintf(buf, sizeof(buf), "Found %s, connecting...", gWifiSSID);
    sysLog(buf);
    WiFi.begin(gWifiSSID, gWifiPass);
    const uint32_t connStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - connStart < 8000) {
      delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(buf, sizeof(buf), "WiFi: connected %s", WiFi.localIP().toString().c_str());
      sysLog(buf);
      gWifiMode = true;
    } else {
      sysLog("WiFi: connection timeout, going offline");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
  } else {
    sysLog("WiFi: hotspot not found, going offline");
    WiFi.mode(WIFI_OFF);
  }

  // Aufnahmeverzeichnis nur im Offline-Modus vorab anlegen
  // (Web-Modus erstellt es beim Start-Button)
  if (!gWifiMode) {
    int dirN = 0;
    do { snprintf(dataDir, sizeof(dataDir), "/data%d", dirN++); }
    while (SD.exists(dataDir));
    for (int attempt = 0; ; attempt++) {
      if (SD.mkdir(dataDir)) { Serial.printf("OK. Created %s\n", dataDir); break; }
      if (attempt >= 2) break;
      delay(50);
    }
  }

  gSdMutex = xSemaphoreCreateMutex();
  if (!gSdMutex) { sysLog("ERR gSdMutex failed"); while(1); }

  gPacketQueue   = xQueueCreate(PACKET_QUEUE_LEN,    sizeof(Packet));
  gTofQueue      = xQueueCreate(TOF_QUEUE_LEN,       sizeof(TOFSample));
  gAudioQueue    = xQueueCreate(AUDIO_QUEUE_LEN,     sizeof(uint8_t*));
  gMidPhotoQueue = xQueueCreate(MID_PHOTO_QUEUE_LEN, sizeof(MidPhoto));
  gEndPhotoQueue = xQueueCreate(END_PHOTO_QUEUE_LEN, sizeof(EndPhoto));
  if (!gPacketQueue || !gTofQueue || !gAudioQueue || !gMidPhotoQueue || !gEndPhotoQueue) {
    sysLog("ERR queue create failed"); while(1);
  }

  BaseType_t ok1 = xTaskCreatePinnedToCore(samplerTask,  "sampler",  SAMPLER_STACK_SIZE,   nullptr, SAMPLER_PRIORITY,   nullptr,              0);
  BaseType_t ok2 = xTaskCreatePinnedToCore(writerTask,   "writer",   WRITER_STACK_SIZE,    nullptr, WRITER_PRIORITY,    nullptr,              1);
  BaseType_t ok3 = xTaskCreatePinnedToCore(tofTask,      "tof",      TOF_STACK_SIZE,       nullptr, TOF_PRIORITY,       &gTofTaskHandle,      0);
  BaseType_t ok4 = xTaskCreatePinnedToCore(audioTask,    "audio",    AUDIO_STACK_SIZE,     nullptr, AUDIO_PRIORITY,     &gAudioTaskHandle,    0);
  BaseType_t ok5 = xTaskCreatePinnedToCore(midPhotoTask, "midphoto", MID_PHOTO_STACK_SIZE, nullptr, MID_PHOTO_PRIORITY, &gMidPhotoTaskHandle, 1);
  BaseType_t ok6 = xTaskCreatePinnedToCore(endPhotoTask, "endphoto", END_PHOTO_STACK_SIZE, nullptr, END_PHOTO_PRIORITY, &gEndPhotoTaskHandle, 1);
  if (ok1!=pdPASS||ok2!=pdPASS||ok3!=pdPASS||ok4!=pdPASS||ok5!=pdPASS||ok6!=pdPASS) {
    sysLog("ERR task create failed"); while(1);
  }

  if (gWifiMode) {
    BaseType_t okW = xTaskCreatePinnedToCore(webTask, "web", WEB_STACK_SIZE, nullptr, WEB_PRIORITY, nullptr, 1);
    if (okW != pdPASS) { sysLog("ERR webTask create failed"); }
  }
}

void loop() { vTaskDelete(NULL); }

/********* WiFi config from SD (/wifi.cfg) *********/
void loadWifiConfig() {
  if (!SD.exists("/wifi.cfg")) return;
  File f = SD.open("/wifi.cfg", FILE_READ);
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if      (line.startsWith("ssid=")) strncpy(gWifiSSID, line.substring(5).c_str(), 63);
    else if (line.startsWith("pass=")) strncpy(gWifiPass, line.substring(5).c_str(), 63);
  }
  f.close();
  sysLog("WiFi: config loaded from /wifi.cfg");
}

/********* Audio Init *********/
void setupAudio() {
  sysLog("Initialising microphone ...");
  I2S.setPinsPdmRx(42, 41);
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    sysLog("ERR Failed to initialize I2S!");
    while(1);
  }
  gInitAudio = true;
  sysLog("OK. I2S driver ready.");
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
    if (bytes_read != AUDIO_PCM_SIZE)
      Serial.printf("WARN I2S read only %u of %u bytes\n", bytes_read, (unsigned int)AUDIO_PCM_SIZE);
    {
      static float hpf_x_prev = 0.0f, hpf_y_prev = 0.0f;
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
      Serial.println("WARN audio queue full, dropping audio data");
      free(pcm_buf);
    }
  }
}

/********* Mid-window Photo Task *********/
void midPhotoTask(void* arg) {
  for (;;) {
    uint32_t notifiedIdx = 0;
    xTaskNotifyWait(0, UINT32_MAX, &notifiedIdx, portMAX_DELAY);
    const uint32_t target = gMidPhotoWindowStartMs + MID_PHOTO_TARGET_MS;
    const uint32_t now = millis();
    if (now < target) vTaskDelay(pdMS_TO_TICKS(target - now));

    uint8_t* buf = nullptr; size_t len = 0;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      buf = (uint8_t*)malloc(fb->len);
      if (buf) { memcpy(buf, fb->buf, fb->len); len = fb->len; }
      esp_camera_fb_return(fb);
    }
    MidPhoto mp = {};
    mp.buf = buf; mp.len = len;
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
    const uint32_t target = gEndPhotoWindowStartMs + END_PHOTO_TARGET_MS;
    const uint32_t now = millis();
    if (now < target) vTaskDelay(pdMS_TO_TICKS(target - now));

    uint8_t* buf = nullptr; size_t len = 0;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      buf = (uint8_t*)malloc(fb->len);
      if (buf) { memcpy(buf, fb->buf, fb->len); len = fb->len; }
      esp_camera_fb_return(fb);
    }
    EndPhoto ep = {};
    ep.buf = buf; ep.len = len;
    ep.captureTs = (fb && buf) ? millis() : 0;
    ep.packetIdx = notifiedIdx;
    if (!fb)       strncpy(ep.reason, "camera capture failed", sizeof(ep.reason) - 1);
    else if (!buf) strncpy(ep.reason, "malloc failed",         sizeof(ep.reason) - 1);
    if (xQueueSend(gEndPhotoQueue, &ep, pdMS_TO_TICKS(50)) != pdPASS) {
      if (ep.buf) free(ep.buf);
      Serial.println("ERR end-photo queue full");
    }
  }
}

/********* IMU init (SPI) *********/
void setupIMU() {
  myIMU.setSPIClockSpeed(4000000);
  if (!myIMU.init()) {
    sysLog("ERR ICM20948 does not respond");
    return;
  }
  gInitIMU = true;
  {
    char buf[80];
    snprintf(buf, sizeof(buf), "OK. ICM20948 connected via SPI (CS=GPIO%d)", IMU_SPI_CS_PIN);
    sysLog(buf);
  }
  if (myIMU.initMagnetometer()) {
    hasMag = true;
    sysLog("OK. Magnetometer ready (100 Hz)");
  } else {
    sysLog("WARN Magnetometer init failed — continuing without");
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
  sysLog("FIFO started (ACC+GYR @ ~100 Hz)");
}

/********* ToF init (VL53L5CX 8×8) *********/
void setupTOF() {
  if (!ENABLE_TOF) {
    sysLog("INFO VL53L5CX disabled for test");
    hasTOF = false;
    return;
  }
  for (int attempt = 0; attempt < 2; attempt++) {
    if (attempt > 0)
      Serial.printf("INFO Retry VL53L5CX init (%d/2)...\n", attempt + 1);
    if (myImager.begin()) {
      myImager.setResolution(8 * 8);
      myImager.setRangingFrequency(15);
      myImager.startRanging();
      hasTOF = true;
      sysLog("OK. VL53L5CX ready (8x8, 15 Hz, continuous)");
      return;
    }
    Serial.printf("WARN VL53L5CX init failed (attempt %d/2)\n", attempt + 1);
  }
  Serial.println("INFO I2C scan:");
  bool found = false;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf("  0x%02X\n", a); found = true; }
  }
  if (!found) Serial.println("  (keine Geräte gefunden)");
  sysLog("WARN VL53L5CX nicht verfügbar — weiter ohne ToF");
}

/********* ToF Task *********/
void tofTask(void* arg) {
  if (!hasTOF) vTaskDelete(NULL);
  VL53L5CX_ResultsData measurementData;
  for (;;) {
    if (myImager.isDataReady()) {
      if (myImager.getRangingData(&measurementData)) {
        TOFSample sample;
        sample.ts_ms = millis();
        for (int z = 0; z < TOF_ZONES; ++z) {
          sample.distance_mm[z]   = measurementData.distance_mm[z];
          sample.target_status[z] = measurementData.target_status[z];
        }
        xQueueSend(gTofQueue, &sample, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
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

  // Warmup-Fenster: Kamera und Auto-Exposition einpendeln lassen
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
    pkt.idx   = mediaCounter;
    pkt.ts_ms = windowStartMs;

    auto addError = [](const char* msg) {
      if (pkt.errorCount < MAX_ERRORS)
        strncpy(pkt.errorMsgs[pkt.errorCount++], msg, MAX_ERROR_LEN - 1);
    };

    // Audio
    uint8_t* pcmBuf = nullptr;
    if (xQueueReceive(gAudioQueue, &pcmBuf, pdMS_TO_TICKS(200)) == pdPASS) {
      pkt.pcmBuf = pcmBuf; pkt.pcmSize = AUDIO_PCM_SIZE;
    } else {
      Serial.println("WARN sampler task missing audio buffer.");
      addError("audio: buffer missing (queue timeout)");
      pkt.pcmBuf = nullptr; pkt.pcmSize = 0;
    }

    // IMU FIFO
    {
      int16_t rawCount = myIMU.getNumberOfFifoDataSets();
      if      (rawCount < 0) addError("IMU: FIFO read error (negative count)");
      else if (rawCount == 0) addError("IMU: FIFO empty at window end");
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
    if (hasMag) { myIMU.readSensor(); myIMU.getMagValues(&pkt.mag); }

    // ToF (8×8)
    pkt.tofCount = 0;
    if (hasTOF) {
      TOFSample sample;
      while (xQueueReceive(gTofQueue, &sample, 0) == pdPASS && pkt.tofCount < MAX_TOF_READINGS) {
        pkt.tof[pkt.tofCount].ts_offset_ms = sample.ts_ms - pkt.ts_ms;
        for (int z = 0; z < TOF_ZONES; ++z) {
          pkt.tof[pkt.tofCount].distance_mm[z]   = sample.distance_mm[z];
          pkt.tof[pkt.tofCount].target_status[z] = sample.target_status[z];
        }
        pkt.tofCount++;
      }
    }

    // Mid-Photo abholen
    {
      MidPhoto mp = {}; bool gotMid = false;
      while (xQueueReceive(gMidPhotoQueue, &mp, 0) == pdPASS) {
        if (mp.packetIdx == pkt.idx) { gotMid = true; break; }
        if (mp.buf) free(mp.buf);
        char msg[MAX_ERROR_LEN];
        snprintf(msg, sizeof(msg), "mid-photo: stale idx=%lu verworfen", (unsigned long)mp.packetIdx);
        addError(msg); mp = {};
      }
      if (gotMid) {
        pkt.midImgBuf = mp.buf; pkt.midImgLen = mp.len; pkt.midCaptureTs = mp.captureTs;
        if (!mp.buf) { char msg[MAX_ERROR_LEN]; snprintf(msg, sizeof(msg), "mid-photo: %s", mp.reason[0]?mp.reason:"null buf"); addError(msg); }
      } else { addError("mid-photo: not in queue at window end"); }
    }

    // End-Photo abholen
    {
      EndPhoto ep = {}; bool gotEnd = false;
      while (xQueueReceive(gEndPhotoQueue, &ep, 0) == pdPASS) {
        if (ep.packetIdx == pkt.idx) { gotEnd = true; break; }
        if (ep.buf) free(ep.buf);
        char msg[MAX_ERROR_LEN];
        snprintf(msg, sizeof(msg), "end-photo: stale idx=%lu verworfen", (unsigned long)ep.packetIdx);
        addError(msg); ep = {};
      }
      if (gotEnd) {
        pkt.endImgBuf = ep.buf; pkt.endImgLen = ep.len; pkt.endCaptureTs = ep.captureTs;
        if (!ep.buf) { char msg[MAX_ERROR_LEN]; snprintf(msg, sizeof(msg), "end-photo: %s", ep.reason[0]?ep.reason:"null buf"); addError(msg); }
      } else { addError("end-photo: not in queue at window end"); }
    }

    // Web-Status aktualisieren (nur im WiFi-Modus)
    if (gWifiMode) {
      static uint32_t lastBatMs = 0;
      WebStatus ws = {};
      ws.uptime_ms    = millis();
      ws.packet_count = mediaCounter;
      if (pkt.imuCount > 0) {
        const auto& last = pkt.imu[pkt.imuCount - 1];
        ws.ax = last.ax; ws.ay = last.ay; ws.az = last.az;
        ws.gx = last.gx; ws.gy = last.gy; ws.gz = last.gz;
      }
      ws.mag_x = pkt.mag.x; ws.mag_y = pkt.mag.y; ws.mag_z = pkt.mag.z;
      // Mittelwert der 4 Zentrums-Zonen (8×8 → Zeilen 3-4, Spalten 3-4: Indizes 27,28,35,36)
      if (pkt.tofCount > 0) {
        const auto& last = pkt.tof[pkt.tofCount - 1];
        int32_t sum = 0, cnt = 0;
        for (int z : {27, 28, 35, 36}) {
          uint8_t st = last.target_status[z];
          if (st == 5 || st == 9) { sum += last.distance_mm[z]; cnt++; }
        }
        ws.tof_center_mm = cnt > 0 ? (int16_t)(sum / cnt) : -1;
        ws.tof_valid     = cnt > 0;
      }
      // Batterie alle 10 s lesen (ADC1 — WiFi-sicher)
      if (lastBatMs == 0 || millis() - lastBatMs > 10000) {
        int mv = analogReadMilliVolts(BATTERY_ADC_PIN);
        ws.battery_mv  = mv * 2.0f;   // Spannungsteiler 1:1 → ×2
        ws.battery_pct = constrain((int)((ws.battery_mv - 3000.0f) / 12.0f), 0, 100);
        lastBatMs = millis();
      } else {
        ws.battery_mv  = gWebStatus.battery_mv;
        ws.battery_pct = gWebStatus.battery_pct;
      }
      memcpy(&gWebStatus, &ws, sizeof(ws));
    }

    windowStartMs += PACKET_INTERVAL_MS;

    if (gWifiMode && !gRecording) {
      // Im Web-Modus ohne laufende Aufnahme Daten verwerfen
      if (pkt.pcmBuf)    free(pkt.pcmBuf);
      if (pkt.midImgBuf) free(pkt.midImgBuf);
      if (pkt.endImgBuf) free(pkt.endImgBuf);
    } else {
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
}

/********* JSON helper *********/
void buildJson(const Packet& pkt, JsonDocument& doc) {
  doc["ts"] = pkt.ts_ms;
  JsonObject magObj = doc["mag"].to<JsonObject>();
  magObj["x"] = pkt.mag.x; magObj["y"] = pkt.mag.y; magObj["z"] = pkt.mag.z;

  JsonArray tofArr = doc["tof"].to<JsonArray>();
  for (size_t i = 0; i < pkt.tofCount; ++i) {
    JsonObject e   = tofArr.add<JsonObject>();
    e["t"]         = pkt.tof[i].ts_offset_ms;
    JsonArray dArr = e["d"].to<JsonArray>();
    JsonArray sArr = e["s"].to<JsonArray>();
    for (int z = 0; z < TOF_ZONES; ++z) {
      dArr.add(pkt.tof[i].distance_mm[z]);
      sArr.add(pkt.tof[i].target_status[z]);
    }
  }

  JsonArray imuArr = doc["IMU"].to<JsonArray>();
  for (size_t i = 0; i < pkt.imuCount; ++i) {
    JsonObject e = imuArr.add<JsonObject>();
    e["i"] = static_cast<uint16_t>(i);
    JsonObject a = e["a"].to<JsonObject>();
    a["x"] = pkt.imu[i].ax; a["y"] = pkt.imu[i].ay; a["z"] = pkt.imu[i].az;
    JsonObject g = e["g"].to<JsonObject>();
    g["x"] = pkt.imu[i].gx; g["y"] = pkt.imu[i].gy; g["z"] = pkt.imu[i].gz;
  }

  if (pkt.midCaptureTs > 0) doc["midTs"] = (int32_t)(pkt.midCaptureTs - pkt.ts_ms);
  if (pkt.endCaptureTs > 0) doc["endTs"] = (int32_t)(pkt.endCaptureTs - pkt.ts_ms);

  if (pkt.errorCount > 0) {
    JsonArray errArr = doc["errors"].to<JsonArray>();
    for (size_t i = 0; i < pkt.errorCount; i++) errArr.add(pkt.errorMsgs[i]);
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
    const uint32_t bucket     = pkt.idx / PACKETS_PER_BUCKET;

    xSemaphoreTake(gSdMutex, portMAX_DELAY);

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
         pkt.idx, pkt.ts_ms, (unsigned)pkt.imuCount, (unsigned)pkt.tofCount, (unsigned)pkt.pcmSize);

    char path[64];
    snprintf(path, sizeof(path), "%s/pkt_%lu.json", bucketDir, pkt.idx);
    if (File f = SD.open(path, FILE_WRITE)) { serializeJson(doc, f); f.close(); LOGV("OK  saved %s\n", path); }
    else { Serial.printf("ERR failed to open %s\n", path); }

    if (pkt.pcmBuf && pkt.pcmSize > 0) {
      snprintf(path, sizeof(path), "%s/rec_%lu_%lu.wav", bucketDir, pkt.idx, pkt.ts_ms);
      if (File wf = SD.open(path, FILE_WRITE)) {
        uint8_t hdr[WAV_HEADER_SIZE];
        const uint32_t data_size   = pkt.pcmSize;
        const uint32_t chunk_size  = WAV_HEADER_SIZE - 8 + data_size;
        const uint32_t byte_rate   = AUDIO_FS * AUDIO_CHAN * (AUDIO_BITS / 8);
        const uint16_t block_align = AUDIO_CHAN * (AUDIO_BITS / 8);
        memcpy(hdr +  0, "RIFF", 4); memcpy(hdr +  4, &chunk_size,  4);
        memcpy(hdr +  8, "WAVE", 4); memcpy(hdr + 12, "fmt ",       4);
        const uint32_t fmt_size = 16; memcpy(hdr + 16, &fmt_size,   4);
        const uint16_t pcm_fmt  = 1;  memcpy(hdr + 20, &pcm_fmt,    2);
        const uint16_t channels = AUDIO_CHAN;   memcpy(hdr + 22, &channels,    2);
        const uint32_t sample_rate = AUDIO_FS; memcpy(hdr + 24, &sample_rate, 4);
        memcpy(hdr + 28, &byte_rate,   4); memcpy(hdr + 32, &block_align, 2);
        const uint16_t bits = AUDIO_BITS; memcpy(hdr + 34, &bits,    2);
        memcpy(hdr + 36, "data", 4); memcpy(hdr + 40, &data_size,   4);
        wf.write(hdr, WAV_HEADER_SIZE);
        wf.write(pkt.pcmBuf, pkt.pcmSize);
        wf.close();
        LOGV("OK  saved %s\n", path);
      } else { Serial.printf("ERR failed to open %s\n", path); }
      free(pkt.pcmBuf);
    }

    if (pkt.endImgBuf && pkt.endImgLen > 0) {
      snprintf(path, sizeof(path), "%s/img_%lu_%lu.jpg", bucketDir, pkt.idx, pkt.ts_ms);
      if (File img = SD.open(path, FILE_WRITE)) { img.write(pkt.endImgBuf, pkt.endImgLen); img.close(); LOGV("OK  saved %s\n", path); }
      else { Serial.printf("ERR failed to open %s\n", path); }
      free(pkt.endImgBuf);
    }

    if (pkt.midImgBuf && pkt.midImgLen > 0) {
      snprintf(path, sizeof(path), "%s/mid_%lu_%lu.jpg", bucketDir, pkt.idx, pkt.ts_ms);
      if (File mf = SD.open(path, FILE_WRITE)) { mf.write(pkt.midImgBuf, pkt.midImgLen); mf.close(); LOGV("OK  saved %s\n", path); }
      else { Serial.printf("ERR failed to open %s\n", path); }
      free(pkt.midImgBuf);
    }

    xSemaphoreGive(gSdMutex);

    const uint32_t writeMs = millis() - writeStart;
    if (writeMs > 950) Serial.printf("WARN writer slow: pkt %lu took %lu ms\n", pkt.idx, writeMs);
  }
}

/********* Web Task *********/
static WebServer gServer(80);

static const char* mimeType(const char* path) {
  if (strstr(path, ".html")) return "text/html";
  if (strstr(path, ".css"))  return "text/css";
  if (strstr(path, ".js"))   return "application/javascript";
  if (strstr(path, ".json")) return "application/json";
  if (strstr(path, ".jpg"))  return "image/jpeg";
  return "application/octet-stream";
}

static void serveFromLittleFS(const String& uri) {
  if (!LittleFS.exists(uri)) { gServer.send(404, "text/plain", "Not found"); return; }
  File f = LittleFS.open(uri, "r");
  gServer.streamFile(f, mimeType(uri.c_str()));
  f.close();
}

struct ZipEntry { char rel[96]; uint32_t offset, crc32, size; };

static void streamZipFolder(const char* folder) {
  const int MAX_ZIP_FILES = 600;
  ZipEntry* entries = (ZipEntry*)malloc(MAX_ZIP_FILES * sizeof(ZipEntry));
  if (!entries) { gServer.send(500, "text/plain", "OOM"); return; }
  int entryCount = 0;

  xSemaphoreTake(gSdMutex, portMAX_DELAY);
  File root = SD.open(folder);
  if (!root || !root.isDirectory()) {
    xSemaphoreGive(gSdMutex); free(entries);
    gServer.send(404, "text/plain", "Folder not found"); return;
  }
  File bkt = root.openNextFile();
  while (bkt && entryCount < MAX_ZIP_FILES) {
    if (bkt.isDirectory()) {
      char bktPath[64];
      snprintf(bktPath, sizeof(bktPath), "%s/%s", folder, bkt.name());
      File bDir = SD.open(bktPath);
      if (bDir) {
        File f = bDir.openNextFile();
        while (f && entryCount < MAX_ZIP_FILES) {
          ZipEntry& e = entries[entryCount++];
          snprintf(e.rel, sizeof(e.rel), "%s/%s/%s", folder + 1, bkt.name(), f.name());
          e.size = f.size(); e.offset = 0; e.crc32 = 0;
          f.close(); f = bDir.openNextFile();
        }
        bDir.close();
      }
    }
    bkt.close(); bkt = root.openNextFile();
  }
  root.close();
  xSemaphoreGive(gSdMutex);

  String folderName = String(folder).substring(1);
  WiFiClient client = gServer.client();
  String header = "HTTP/1.1 200 OK\r\nContent-Type: application/zip\r\n";
  header += "Content-Disposition: attachment; filename=\"" + folderName + ".zip\"\r\n";
  header += "Transfer-Encoding: chunked\r\nConnection: close\r\n\r\n";
  client.print(header);

  auto writeChunk = [&](const uint8_t* data, size_t len) {
    char hex[12]; snprintf(hex, sizeof(hex), "%X\r\n", (unsigned)len);
    client.print(hex); client.write(data, len); client.print("\r\n");
  };
  auto writeChunkStr = [&](const char* s, size_t len) {
    writeChunk((const uint8_t*)s, len);
  };

  uint32_t streamOffset = 0;
  uint8_t  fileBuf[512];

  for (int i = 0; i < entryCount; i++) {
    ZipEntry& e = entries[i];
    e.offset = streamOffset;
    uint16_t fnLen = strlen(e.rel);
    uint8_t lhdr[30] = {};
    lhdr[0]=0x50;lhdr[1]=0x4B;lhdr[2]=0x03;lhdr[3]=0x04;
    lhdr[4]=20; lhdr[6]=0x08;
    lhdr[26]=uint8_t(fnLen); lhdr[27]=uint8_t(fnLen>>8);
    writeChunk(lhdr, 30);
    writeChunkStr(e.rel, fnLen);
    streamOffset += 30 + fnLen;

    uint32_t crc = 0, written = 0;
    xSemaphoreTake(gSdMutex, portMAX_DELAY);
    char fullPath[128];
    snprintf(fullPath, sizeof(fullPath), "/%s", e.rel);
    File f = SD.open(fullPath, FILE_READ);
    if (f) {
      while (f.available()) {
        size_t n = f.read(fileBuf, sizeof(fileBuf));
        crc = crc32_update(crc, fileBuf, n);
        writeChunk(fileBuf, n);
        written += n;
      }
      f.close();
    }
    xSemaphoreGive(gSdMutex);
    e.crc32 = crc; e.size = written;
    streamOffset += written;

    uint8_t dd[16] = {};
    dd[0]=0x50;dd[1]=0x4B;dd[2]=0x07;dd[3]=0x08;
    memcpy(dd+4, &crc, 4); memcpy(dd+8, &written, 4); memcpy(dd+12, &written, 4);
    writeChunk(dd, 16);
    streamOffset += 16;
  }

  uint32_t cdOffset = streamOffset, cdSize = 0;
  for (int i = 0; i < entryCount; i++) {
    ZipEntry& e = entries[i];
    uint16_t fnLen = strlen(e.rel);
    uint8_t cdhdr[46] = {};
    cdhdr[0]=0x50;cdhdr[1]=0x4B;cdhdr[2]=0x01;cdhdr[3]=0x02;
    cdhdr[4]=20; cdhdr[6]=20; cdhdr[8]=0x08;
    memcpy(cdhdr+16, &e.crc32, 4); memcpy(cdhdr+20, &e.size, 4); memcpy(cdhdr+24, &e.size, 4);
    cdhdr[28]=uint8_t(fnLen); cdhdr[29]=uint8_t(fnLen>>8);
    memcpy(cdhdr+42, &e.offset, 4);
    writeChunk(cdhdr, 46);
    writeChunkStr(e.rel, fnLen);
    cdSize += 46 + fnLen;
  }

  uint8_t eocd[22] = {};
  eocd[0]=0x50;eocd[1]=0x4B;eocd[2]=0x05;eocd[3]=0x06;
  uint16_t cnt = entryCount;
  memcpy(eocd+8, &cnt, 2); memcpy(eocd+10, &cnt, 2);
  memcpy(eocd+12, &cdSize, 4); memcpy(eocd+16, &cdOffset, 4);
  writeChunk(eocd, 22);
  client.print("0\r\n\r\n");
  free(entries);
}

void webTask(void* arg) {
  if (!LittleFS.begin(true, "/lfs")) {
    sysLog("ERR LittleFS mount failed — web UI unavailable");
    vTaskDelete(NULL);
  }
  sysLog("OK. LittleFS mounted");

  gServer.on("/", []() {
    gServer.sendHeader("Location", "/viewer/index.html", true);
    gServer.send(302, "text/plain", "");
  });

  gServer.on("/api/status", []() {
    WebStatus ws;
    memcpy(&ws, &gWebStatus, sizeof(ws));

    JsonDocument doc;
    doc["recording"]    = (bool)gRecording;
    doc["packet_count"] = ws.packet_count;
    doc["uptime_ms"]    = ws.uptime_ms;

    JsonObject initObj = doc["init"].to<JsonObject>();
    initObj["sd"]          = gInitSD;
    initObj["camera"]      = gInitCamera;
    initObj["imu"]         = gInitIMU;
    initObj["magnetometer"]= hasMag;
    initObj["tof"]         = hasTOF;
    initObj["audio"]       = gInitAudio;
    initObj["wifi"]        = (bool)gWifiMode;

    JsonObject imuObj = doc["imu"].to<JsonObject>();
    imuObj["ax"]=ws.ax; imuObj["ay"]=ws.ay; imuObj["az"]=ws.az;
    imuObj["gx"]=ws.gx; imuObj["gy"]=ws.gy; imuObj["gz"]=ws.gz;

    // VL53L5CX: Mittelwert der 4 Zentrums-Zonen (Zonen 27, 28, 35, 36)
    JsonObject tofObj = doc["tof"].to<JsonObject>();
    tofObj["distance_mm"] = ws.tof_valid ? (int)ws.tof_center_mm : -1;
    tofObj["status"]      = ws.tof_valid ? 5 : -1;

    JsonObject magObj = doc["mag"].to<JsonObject>();
    magObj["x"]=ws.mag_x; magObj["y"]=ws.mag_y; magObj["z"]=ws.mag_z;

    JsonObject batObj = doc["battery"].to<JsonObject>();
    batObj["mv"]  = (int)ws.battery_mv;
    batObj["pct"] = ws.battery_pct;

    JsonArray logArr = doc["log"].to<JsonArray>();
    for (uint8_t i = 0; i < gLogCount; i++) {
      JsonObject e = logArr.add<JsonObject>();
      e["ts"] = gLog[i].ts_ms; e["msg"] = gLog[i].msg;
    }

    String out; serializeJson(doc, out);
    gServer.send(200, "application/json", out);
  });

  gServer.on("/api/start", HTTP_POST, []() {
    if (!gRecording) {
      int dirN = 0;
      do { snprintf(dataDir, sizeof(dataDir), "/data%d", dirN++); }
      while (SD.exists(dataDir));
      xSemaphoreTake(gSdMutex, portMAX_DELAY);
      SD.mkdir(dataDir);
      xSemaphoreGive(gSdMutex);
      mediaCounter = 0;
      gRecording = true;
      Serial.printf("Recording started: %s\n", dataDir);
    }
    gServer.send(200, "application/json", "{\"ok\":true}");
  });

  gServer.on("/api/stop", HTTP_POST, []() {
    gRecording = false;
    Serial.println("Recording stopped");
    gServer.send(200, "application/json", "{\"ok\":true}");
  });

  gServer.on("/api/photo", []() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { gServer.send(503, "text/plain", "Camera unavailable"); return; }
    WiFiClient client = gServer.client();
    String h = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: ";
    h += String(fb->len);
    h += "\r\nConnection: close\r\n\r\n";
    client.print(h);
    client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  gServer.on("/api/files", []() {
    JsonDocument doc;
    JsonArray sessions = doc.to<JsonArray>();

    xSemaphoreTake(gSdMutex, portMAX_DELAY);
    File root = SD.open("/");
    if (root) {
      File entry = root.openNextFile();
      while (entry) {
        if (entry.isDirectory()) {
          String name = String(entry.name());
          if (name.startsWith("data")) {
            JsonObject sess = sessions.add<JsonObject>();
            sess["session"] = name;
            String sessPath = "/" + name;
            sess["path"] = sessPath;
            JsonArray files = sess["files"].to<JsonArray>();
            File sessDir = SD.open(sessPath.c_str());
            if (sessDir) {
              File bkt = sessDir.openNextFile();
              while (bkt) {
                if (bkt.isDirectory()) {
                  String bktPath = sessPath + "/" + String(bkt.name());
                  File bDir = SD.open(bktPath.c_str());
                  if (bDir) {
                    File f = bDir.openNextFile();
                    while (f) {
                      files.add(bktPath + "/" + String(f.name()));
                      f.close(); f = bDir.openNextFile();
                    }
                    bDir.close();
                  }
                }
                bkt.close(); bkt = sessDir.openNextFile();
              }
              sessDir.close();
            }
          }
        }
        entry.close(); entry = root.openNextFile();
      }
      root.close();
    }
    xSemaphoreGive(gSdMutex);

    String out; serializeJson(doc, out);
    gServer.send(200, "application/json", out);
  });

  gServer.on("/api/download", []() {
    String path = gServer.arg("path");
    if (path.isEmpty()) { gServer.send(400, "text/plain", "Missing path"); return; }
    xSemaphoreTake(gSdMutex, portMAX_DELAY);
    if (!SD.exists(path.c_str())) {
      xSemaphoreGive(gSdMutex);
      gServer.send(404, "text/plain", "Not found"); return;
    }
    File f = SD.open(path.c_str(), FILE_READ);
    size_t fileSize = f.size();
    WiFiClient client = gServer.client();
    String fname = path.substring(path.lastIndexOf('/') + 1);
    String h = "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\n";
    h += "Content-Length: " + String(fileSize) + "\r\n";
    h += "Content-Disposition: attachment; filename=\"" + fname + "\"\r\n";
    h += "Connection: close\r\n\r\n";
    client.print(h);
    uint8_t buf[512];
    while (f.available()) { size_t n = f.read(buf, sizeof(buf)); client.write(buf, n); }
    f.close();
    xSemaphoreGive(gSdMutex);
  });

  gServer.on("/api/download-folder", []() {
    String path = gServer.arg("path");
    if (path.isEmpty()) { gServer.send(400, "text/plain", "Missing path"); return; }
    streamZipFolder(path.c_str());
  });

  gServer.onNotFound([]() {
    const String& uri = gServer.uri();
    if (uri.startsWith("/viewer/")) {
      serveFromLittleFS(uri);
    } else {
      gServer.send(404, "text/plain", "Not found");
    }
  });

  gServer.begin();
  sysLog("OK. Web server started on port 80");

  for (;;) {
    gServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
