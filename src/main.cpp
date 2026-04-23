// main.cpp — FreeRTOS-scheduled sensor logger (ESP32S3 Sense)
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <SparkFun_VL53L5CX_Library.h>
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

// Verbose per-packet logging (Window end / Packet sent / PKT / OK saved).
// 0 = nur Errors und Warnings im Loop — spart UART-Zeit und verhindert, dass
// der TX-Puffer blockiert, wenn SD-Spikes mehrere Logs pro Sekunde erzeugen.
// Auf 1 setzen zum Debuggen.
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
static constexpr uint16_t AUDIO_FS   = 16000;  // 16 kHz
static constexpr uint8_t  AUDIO_BITS = 16;     // 16-bit
static constexpr uint8_t  AUDIO_CHAN = 1;      // Mono
// Genau 1000 ms pro WAV-Datei. audioTask liest kontinuierlich — aufeinander-
// folgende WAVs enthalten lückenlos aufeinanderfolgende Samples aus dem I2S-
// DMA-Stream, ohne Überlappung.
static constexpr uint32_t AUDIO_PCM_SIZE = AUDIO_FS * (AUDIO_BITS / 8) * AUDIO_CHAN;
static constexpr uint32_t WAV_HEADER_SIZE = 44;
// IIR High-Pass α: y[n] = x[n] - x[n-1] + α·y[n-1]
// Cutoff ≈ (1-α)·fs/(2π) → mit 0.999 @ 16 kHz ≈ 2.5 Hz (DC-Removal)
static constexpr float AUDIO_HPF_ALPHA = 0.999f;
// Software-Gain für PDM-Mic-Empfindlichkeit (nach HPF, vor Clamp).
// hpf_y_prev bleibt unscaliert → kein Feedback-Divergenz-Problem.
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
  .grab_mode    = CAMERA_GRAB_LATEST   // neuestes Frame sofort, kein Warten auf nächstes
};

/********* IMU & ToF settings *********/
#define ICM20948_ADDR 0x68
ICM20948_WE      myIMU(ICM20948_ADDR);
SparkFun_VL53L5CX myImager;

constexpr uint32_t PACKET_INTERVAL_MS = 1000;
constexpr size_t   MAX_FIFO_SETS      = 120;
constexpr size_t   MAX_TOF_READINGS   = 16;  // 15 Hz × 1 s = 15 Frames, 16 mit Puffer
constexpr uint8_t  TOF_ZONES          = 64;  // 8×8 Auflösung
// FAT32-Directory-Scan ist O(n) pro SD.open(). Bei 4 Files/Paket wächst ein
// flaches Dir linear und Writer-Zeit steigt stetig. Alle 30 Pakete ein neues
// Sub-Dir → 1 Bucket = 30 s, jedes Verzeichnis bleibt typ. ≤ 120 Einträge.
constexpr uint32_t PACKETS_PER_BUCKET = 30;

/********* Task & Queue Configuration *********/
static constexpr uint32_t JSON_DOC_SIZE = 32768;

static constexpr size_t PACKET_QUEUE_LEN    = 16;  // absorbiert SD-Erase/GC-Spikes bis ~16 s
static constexpr size_t TOF_QUEUE_LEN       = 20;  // 15 Hz × ~1.3 s Puffer
static constexpr size_t AUDIO_QUEUE_LEN     = 4;  // Jitter-Puffer für kontinuierlichen I2S-Stream
static constexpr size_t MID_PHOTO_QUEUE_LEN = 2;
static constexpr size_t END_PHOTO_QUEUE_LEN = 2;

static constexpr uint32_t SAMPLER_STACK_SIZE   = 8192;
static constexpr uint32_t WRITER_STACK_SIZE    = 8192;
static constexpr uint32_t TOF_STACK_SIZE       = 6144;
static constexpr uint32_t AUDIO_STACK_SIZE     = 6144;
static constexpr uint32_t MID_PHOTO_STACK_SIZE = 6144;
static constexpr uint32_t END_PHOTO_STACK_SIZE = 6144;

// Higher number = higher priority. Audio must be highest to avoid losing samples.
static constexpr UBaseType_t AUDIO_PRIORITY      = 5;
static constexpr UBaseType_t SAMPLER_PRIORITY    = 4;
static constexpr UBaseType_t WRITER_PRIORITY     = 3;
static constexpr UBaseType_t TOF_PRIORITY        = 3;
static constexpr UBaseType_t MID_PHOTO_PRIORITY  = 2;  // unterhalb Writer — Bild nicht zeitkritisch
static constexpr UBaseType_t END_PHOTO_PRIORITY  = 2;

bool hasMag = false;
bool hasTOF = false;

struct IMUSample { float ax, ay, az, gx, gy, gz; };
struct TOFSample  { uint32_t ts_ms; int16_t distance_mm[TOF_ZONES]; uint8_t target_status[TOF_ZONES]; };
struct TOFReading { uint32_t ts_offset_ms; int16_t distance_mm[TOF_ZONES]; uint8_t target_status[TOF_ZONES]; };
struct MidPhoto   { uint8_t* buf; size_t len; };
struct EndPhoto   { uint8_t* buf; size_t len; };

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


// Packet sent from Sampler to Writer
struct Packet {
  uint32_t idx;
  uint32_t ts_ms;
  size_t   imuCount;
  IMUSample imu[MAX_FIFO_SETS];
  xyzFloat mag;
  size_t   tofCount;
  TOFReading tof[MAX_TOF_READINGS];
  uint8_t *pcmBuf;    // malloc'ed raw PCM audio buffer (ownership passed to writer)
  size_t   pcmSize;
  uint8_t *midImgBuf; // malloc'ed mid-window JPEG (ownership passed to writer)
  size_t   midImgLen;
  uint8_t *endImgBuf; // malloc'ed end-of-window JPEG (ownership passed to writer)
  size_t   endImgLen;
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
uint8_t* create_wav_file_buffer(const uint8_t* pcm_data, size_t pcm_data_size, size_t* wav_file_size);

/********* Arduino setup *********/
void setup() {
  delay(8000);                    // allow USB-serial to connect
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);          // I2C Fast-Mode (400 kHz)
  Serial.println("Booting …");

  /* ---- Sensors & Peripherals ---- */
  setupIMU();
  setupTOF();
  setupAudio(); // New audio setup

  /* ---- Camera ---- */
  if (esp_camera_init(&cfg) == ESP_OK) {
    Serial.println("OK. Camera ready");
  } else {
    Serial.println("ERR Camera init failed");
    while(1);
  }

  /* ---- SD card ---- */
  // Default SD.begin() nutzt nur 4 MHz SPI-Clock — viel zu langsam für unseren
  // Datenstrom (~80 KB/s payload + 4 file open/close pro Paket). 40 MHz ist
  // das SPI-Maximum des ESP32S3; bei kurzen Leitungen und einer Class-10/A1-Karte
  // auf der XIAO-Sense-Verdrahtung stabil und liefert ~10× Durchsatz gegenüber Default.
  Serial.println("Mounting SD …");
  if (!SD.begin(SD_CS_PIN, SPI, 40000000)) {
    Serial.println("ERR SD mount failed");
    while(1);
  }
  Serial.printf("OK. SD ready (SPI @ 40 MHz, card size %llu MB)\n",
                SD.cardSize() / (1024ULL * 1024ULL));

  // Find next available data directory
  int n = 0;
  do {
    snprintf(dataDir, sizeof(dataDir), "/data%d", n++);
  } while (SD.exists(dataDir));

  if (SD.mkdir(dataDir)) {
    Serial.printf("OK. Created data directory: %s\n", dataDir);
  } else {
    Serial.printf("ERR failed to create %s\n", dataDir);
    // continue, maybe it will work anyway
  }

  // Create queues
  gPacketQueue   = xQueueCreate(PACKET_QUEUE_LEN,    sizeof(Packet));
  gTofQueue      = xQueueCreate(TOF_QUEUE_LEN,       sizeof(TOFSample));
  gAudioQueue    = xQueueCreate(AUDIO_QUEUE_LEN,     sizeof(uint8_t*));
  gMidPhotoQueue = xQueueCreate(MID_PHOTO_QUEUE_LEN, sizeof(MidPhoto));
  gEndPhotoQueue = xQueueCreate(END_PHOTO_QUEUE_LEN, sizeof(EndPhoto));
  if (!gPacketQueue || !gTofQueue || !gAudioQueue || !gMidPhotoQueue || !gEndPhotoQueue) {
    Serial.println("ERR queue create failed");
    while(1);
  }

  // Create tasks
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

/********* Arduino loop *********/
void loop() {
  // All work is handled in FreeRTOS tasks, so this one is not needed.
  vTaskDelete(NULL);
}

// --- NEW AUDIO IMPLEMENTATION ---

/********* Audio Init *********/
void setupAudio() {
  Serial.println("Initialising microphone using I2S library...");
  // Using the Seeed Studio XIAO ESP32S3 Sense board-specific library
  // For ESP32 core v3.0.x and later
  I2S.setPinsPdmRx(42, 41);
  if (!I2S.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("ERR Failed to initialize I2S!");
    while(1);
  }
  Serial.println("OK. I2S driver ready.");
}

/********* Audio Task (kontinuierlich — liest I2S-DMA ohne Unterbrechung) *********/
void audioTask(void* arg) {
  // I2S-DMA läuft ab I2S.begin() kontinuierlich. Wir lesen fortlaufend
  // 1000-ms-Blöcke — aufeinanderfolgende Blöcke enthalten lückenlos
  // aufeinanderfolgende Samples. Keine Notification-Synchronisation, damit
  // der DMA-Ringpuffer nie überläuft.
  for (;;) {
    uint8_t* pcm_buf = (uint8_t*)malloc(AUDIO_PCM_SIZE);
    if (!pcm_buf) {
        Serial.println("ERR audioTask failed to allocate pcm_buf");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
    }

    // Read a block of audio data using the correct readBytes function.
    size_t bytes_read = I2S.readBytes((char*)pcm_buf, AUDIO_PCM_SIZE);

    // Check if the expected number of bytes were read
    if (bytes_read != AUDIO_PCM_SIZE) {
      Serial.printf("WARN I2S read only %u of %u bytes\n", bytes_read, (unsigned int)AUDIO_PCM_SIZE);
    }

    // IIR High-Pass Filter: entfernt DC-Offset und verhindert Inter-Buffer-Pops.
    // Zustand ist static → bleibt über Chunk-Grenzen hinweg konsistent.
    {
      static float hpf_x_prev = 0.0f;
      static float hpf_y_prev = 0.0f;
      int16_t* samples = reinterpret_cast<int16_t*>(pcm_buf);
      const size_t n = bytes_read / sizeof(int16_t);
      for (size_t i = 0; i < n; ++i) {
        float x = static_cast<float>(samples[i]);
        float y = x - hpf_x_prev + AUDIO_HPF_ALPHA * hpf_y_prev;
        hpf_x_prev = x;
        hpf_y_prev = y;          // unscaliert speichern — Feedback-Pfad muss DC-frei bleiben
        float out = y * AUDIO_GAIN;
        if      (out >  32767.0f) out =  32767.0f;
        else if (out < -32768.0f) out = -32768.0f;
        samples[i] = static_cast<int16_t>(out);
      }
    }

    // Non-blocking: wenn Queue voll (Sampler hängt hinterher), Buffer verwerfen
    // und sofort den nächsten I2S-Read starten — DMA-Stream darf nicht stocken.
    if (xQueueSend(gAudioQueue, &pcm_buf, 0) != pdPASS) {
      Serial.println("WARN audio queue full, dropping audio data");
      free(pcm_buf);
    }
  }
}


/********* Mid-window Photo Task (Priorität 2 — nicht zeitkritisch) *********/
void midPhotoTask(void* arg) {
  for (;;) {
    // Warten auf Fenster-Start-Notification von samplerTask
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // ~500 ms warten — vTaskDelay reicht, Timing ist zweitrangig
    vTaskDelay(pdMS_TO_TICKS(PACKET_INTERVAL_MS / 2));

    // Foto aufnehmen, JPEG kopieren, Frame-Buffer sofort freigeben
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

    MidPhoto mp = {buf, len};
    if (xQueueSend(gMidPhotoQueue, &mp, pdMS_TO_TICKS(50)) != pdPASS) {
      if (buf) free(buf);
      Serial.println("WARN mid-photo queue full, dropping");
    }
  }
}

/********* End-of-window Photo Task — entkoppelt Kamera vom writerTask *********/
void endPhotoTask(void* arg) {
  for (;;) {
    // Notified von samplerTask bei Fenster-Start — capture spät im Fenster,
    // damit das Foto rechtzeitig in der Queue liegt, bevor samplerTask am
    // nächsten Fenster-Ende (~1000 ms später) abfragt.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Warten bis kurz vor Fenster-Ende (~600 ms), dann capturen.
    // esp_camera_fb_get + memcpy braucht typ. 200-400 ms → fertig um ~900 ms.
    vTaskDelay(pdMS_TO_TICKS((PACKET_INTERVAL_MS * 6) / 10));

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

    EndPhoto ep = {buf, len};
    if (xQueueSend(gEndPhotoQueue, &ep, pdMS_TO_TICKS(50)) != pdPASS) {
      if (buf) free(buf);
      Serial.println("WARN end-photo queue full, dropping");
    }
  }
}

/********* WAV file helper *********/
uint8_t* create_wav_file_buffer(const uint8_t* pcm_data, size_t pcm_data_size, size_t* wav_file_size) {
    *wav_file_size = WAV_HEADER_SIZE + pcm_data_size;
    uint8_t* wav_buf = (uint8_t*)malloc(*wav_file_size);
    if (!wav_buf) {
        return nullptr;
    }

    // RIFF chunk descriptor
    memcpy(wav_buf, "RIFF", 4);
    uint32_t chunk_size = *wav_file_size - 8;
    memcpy(wav_buf + 4, &chunk_size, 4);
    memcpy(wav_buf + 8, "WAVE", 4);

    // "fmt " sub-chunk
    memcpy(wav_buf + 12, "fmt ", 4);
    uint32_t subchunk1_size = 16; // 16 for PCM
    memcpy(wav_buf + 16, &subchunk1_size, 4);
    uint16_t audio_format = 1; // 1 for PCM
    memcpy(wav_buf + 20, &audio_format, 2);
    uint16_t num_channels = AUDIO_CHAN;
    memcpy(wav_buf + 22, &num_channels, 2);
    uint32_t sample_rate = AUDIO_FS;
    memcpy(wav_buf + 24, &sample_rate, 4);
    uint32_t byte_rate = AUDIO_FS * AUDIO_CHAN * (AUDIO_BITS / 8);
    memcpy(wav_buf + 28, &byte_rate, 4);
    uint16_t block_align = AUDIO_CHAN * (AUDIO_BITS / 8);
    memcpy(wav_buf + 32, &block_align, 2);
    uint16_t bits_per_sample = AUDIO_BITS;
    memcpy(wav_buf + 34, &bits_per_sample, 2);

    // "data" sub-chunk
    memcpy(wav_buf + 36, "data", 4);
    uint32_t subchunk2_size = pcm_data_size;
    memcpy(wav_buf + 40, &subchunk2_size, 4);

    // Copy PCM data
    memcpy(wav_buf + WAV_HEADER_SIZE, pcm_data, pcm_data_size);

    return wav_buf;
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

  // Range gewählt nach data19-Analyse: bei ±2g clippen 30.8% der Samples,
  // bei ±250 dps 27% → beides zu klein für Fuß-Dynamik. ±8g / ±1000 dps
  // gibt Headroom für Gehen+leichtes Laufen, DLPF_6 filtert das 4× höhere
  // Quantisierungsrauschen weg.
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

/********* ToF init *********/
void setupTOF() {
  if (!myImager.begin()) {
    Serial.println("WARN VL53L5CX init failed — continuing without");
    return;
  }
  myImager.setResolution(8 * 8);     // 8×8 = 64 Zonen
  myImager.setRangingFrequency(15);  // 15 Hz (Hardware-Maximum bei 8×8)
  myImager.startRanging();
  hasTOF = true;
  Serial.println("OK. VL53L5CX ready (8x8, 15 Hz, continuous)");
}

/********* ToF Task (continuous reading) *********/
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
    vTaskDelay(pdMS_TO_TICKS(5));  // ~5 ms Polling-Intervall
  }
}

/********* Sampler Task (precise 1 s windows) *********/
void samplerTask(void* arg) {
  const TickType_t periodTicks = pdMS_TO_TICKS(PACKET_INTERVAL_MS);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t windowStartMs = millis();

  // FIFO einmal starten und dann *dauerhaft* laufen lassen — kein reset/start
  // pro Fenster. So gehen auch Samples, die während des I2C-Readouts (~57 ms)
  // entstehen, nicht verloren; sie landen einfach im nächsten Paket.
  myIMU.resetFifo();
  myIMU.startFifo(ICM20948_FIFO_ACC_GYR);
  // Stale ToF-Samples verwerfen, bevor das erste Fenster beginnt
  { TOFSample tmp; while (xQueueReceive(gTofQueue, &tmp, 0) == pdPASS) {} }
  // audioTask läuft kontinuierlich von selbst — kein Notify nötig.
  if (gMidPhotoTaskHandle) xTaskNotifyGive(gMidPhotoTaskHandle);
  if (gEndPhotoTaskHandle) xTaskNotifyGive(gEndPhotoTaskHandle);

  for (;;) {
    // Maintain strict cadence per window.
    vTaskDelayUntil(&lastWake, periodTicks);
    LOGV("Sampler task: Window end at %lu ms.\n", (unsigned long)millis());

    // --- Prepare packet ---
    // static: Packet (~6 KB) liegt im BSS, nicht auf dem Task-Stack
    static Packet pkt;
    pkt = {};
    pkt.idx = mediaCounter;
    pkt.ts_ms = windowStartMs;

    // --- Fetch audio buffer for the just-finished window ---
    uint8_t* pcmBuf = nullptr;
    if (xQueueReceive(gAudioQueue, &pcmBuf, pdMS_TO_TICKS(200)) == pdPASS) {
      pkt.pcmBuf = pcmBuf;
      pkt.pcmSize = AUDIO_PCM_SIZE;
    } else {
      Serial.println("WARN sampler task missing audio buffer.");
      pkt.pcmBuf = nullptr;
      pkt.pcmSize = 0;
    }

    // --- Read IMU FIFO (alle bis jetzt gesammelten Samples) ---
    // FIFO läuft kontinuierlich. Wir lesen *genau* die vorhandenen Samples
    // und rufen KEIN resetFifo/stopFifo — Samples, die während des I2C-Reads
    // entstehen, bleiben für das nächste Paket im FIFO. So bilden aufein-
    // anderfolgende Pakete den vollen Sample-Stream ohne Lücken ab.
    {
      int16_t rawCount = myIMU.getNumberOfFifoDataSets();
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
        pkt.tof[pkt.tofCount].ts_offset_ms = sample.ts_ms - pkt.ts_ms;
        for (int z = 0; z < TOF_ZONES; ++z) {
          pkt.tof[pkt.tofCount].distance_mm[z]   = sample.distance_mm[z];
          pkt.tof[pkt.tofCount].target_status[z] = sample.target_status[z];
        }
        pkt.tofCount++;
      }
    }

    // Mid-window Foto abholen — von midPhotoTask im Hintergrund gecaptured
    {
      MidPhoto mp = {};
      xQueueReceive(gMidPhotoQueue, &mp, 0);  // nicht-blockierend
      pkt.midImgBuf = mp.buf;
      pkt.midImgLen = mp.len;
    }
    // End-of-window Foto abholen — von endPhotoTask im Hintergrund gecaptured
    {
      EndPhoto ep = {};
      xQueueReceive(gEndPhotoQueue, &ep, 0);  // nicht-blockierend
      pkt.endImgBuf = ep.buf;
      pkt.endImgLen = ep.len;
    }

    // --- Start next window ---
    // FIFO läuft durchgehend weiter — kein reset/start hier.
    // windowStartMs auf strikte 1000-ms-Cadence fortschreiben, damit Pakete
    // exakt aneinander anschließen (kein Jitter durch Verarbeitungszeit).
    windowStartMs += PACKET_INTERVAL_MS;
    if (gMidPhotoTaskHandle) xTaskNotifyGive(gMidPhotoTaskHandle);
    if (gEndPhotoTaskHandle) xTaskNotifyGive(gEndPhotoTaskHandle);
    // lastWake wird von vTaskDelayUntil automatisch fortgeführt — nicht
    // manuell überschreiben, sonst driftet die Cadence.

    // Enqueue for writer
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



/********* JSON helper ***********/ 
void buildJson(const Packet& pkt, JsonDocument& doc) {
    doc["ts"] = pkt.ts_ms;
    JsonObject magObj = doc["mag"].to<JsonObject>();
    magObj["x"] = pkt.mag.x;
    magObj["y"] = pkt.mag.y;
    magObj["z"] = pkt.mag.z;
    
    JsonArray tofArr = doc["tof"].to<JsonArray>();
    for (size_t i = 0; i < pkt.tofCount; ++i) {
      JsonObject e    = tofArr.add<JsonObject>();
      e["t"]          = pkt.tof[i].ts_offset_ms;
      JsonArray dArr  = e["d"].to<JsonArray>();
      JsonArray sArr  = e["s"].to<JsonArray>();
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
      a["x"] = pkt.imu[i].ax;
      a["y"] = pkt.imu[i].ay;
      a["z"] = pkt.imu[i].az;
      JsonObject g = e["g"].to<JsonObject>();
      g["x"] = pkt.imu[i].gx;
      g["y"] = pkt.imu[i].gy;
      g["z"] = pkt.imu[i].gz;
    }
}

/********* Writer Task (SD + Serial + Photo) *********/
void writerTask(void* arg) {
  // static: Packet (~6 KB) liegt im BSS, nicht auf dem Task-Stack
  static Packet pkt;
  char bucketDir[32];
  uint32_t currentBucket = UINT32_MAX;  // forces mkdir on first packet
  for (;;) {
    pkt = {};
    if (xQueueReceive(gPacketQueue, &pkt, portMAX_DELAY) != pdPASS) {
      continue;
    }

    const uint32_t writeStart = millis();

    // Bucket-Subdir pro PACKETS_PER_BUCKET Pakete anlegen — hält FAT-Dir klein.
    const uint32_t bucket = pkt.idx / PACKETS_PER_BUCKET;
    if (bucket != currentBucket) {
      snprintf(bucketDir, sizeof(bucketDir), "%s/%03lu", dataDir, (unsigned long)bucket);
      if (!SD.exists(bucketDir) && !SD.mkdir(bucketDir)) {
        Serial.printf("ERR failed to create bucket %s\n", bucketDir);
      }
      currentBucket = bucket;
    }

    // 1) Build JSON from packet
    JsonDocument doc;
    buildJson(pkt, doc);

    // Kurze Statuszeile — kein Full-JSON-Dump (würde bei 115200 Baud >1 s blockieren)
    LOGV("PKT %lu  ts=%lu  imu=%u  tof=%u  audio=%u\n",
         pkt.idx, pkt.ts_ms,
         (unsigned)pkt.imuCount, (unsigned)pkt.tofCount,
         (unsigned)pkt.pcmSize);

    // 2) Save JSON
    char path[64];
    snprintf(path, sizeof(path), "%s/pkt_%lu.json", bucketDir, pkt.idx);
    if (File f = SD.open(path, FILE_WRITE)) {
      serializeJson(doc, f);
      f.close();
      LOGV("OK  saved %s\n", path);
    } else {
      Serial.printf("ERR failed to open %s\n", path);
    }

    // 3) WAV direkt schreiben — Header inline, kein 32-KB-Zwischenbuffer
    if (pkt.pcmBuf && pkt.pcmSize > 0) {
      snprintf(path, sizeof(path), "%s/rec_%lu_%lu.wav", bucketDir, pkt.idx, pkt.ts_ms);
      if (File wf = SD.open(path, FILE_WRITE)) {
        // 44-Byte WAV-Header direkt aufbauen und schreiben
        uint8_t hdr[WAV_HEADER_SIZE];
        const uint32_t data_size  = pkt.pcmSize;
        const uint32_t chunk_size = WAV_HEADER_SIZE - 8 + data_size;
        const uint32_t byte_rate  = AUDIO_FS * AUDIO_CHAN * (AUDIO_BITS / 8);
        const uint16_t block_align = AUDIO_CHAN * (AUDIO_BITS / 8);
        memcpy(hdr +  0, "RIFF",           4);
        memcpy(hdr +  4, &chunk_size,       4);
        memcpy(hdr +  8, "WAVE",           4);
        memcpy(hdr + 12, "fmt ",           4);
        const uint32_t fmt_size = 16;    memcpy(hdr + 16, &fmt_size,  4);
        const uint16_t pcm_fmt  = 1;     memcpy(hdr + 20, &pcm_fmt,   2);
        const uint16_t channels = AUDIO_CHAN; memcpy(hdr + 22, &channels, 2);
        // AUDIO_FS ist uint16_t — für das 4-Byte WAV-Sample-Rate-Feld auf uint32_t promoten,
        // sonst liest memcpy 2 Bytes OOB und das Feld enthält Müll (→ Browser lehnt WAV ab).
        const uint32_t sample_rate = AUDIO_FS; memcpy(hdr + 24, &sample_rate, 4);
        memcpy(hdr + 28, &byte_rate,        4);
        memcpy(hdr + 32, &block_align,      2);
        const uint16_t bits = AUDIO_BITS; memcpy(hdr + 34, &bits,      2);
        memcpy(hdr + 36, "data",           4);
        memcpy(hdr + 40, &data_size,        4);
        wf.write(hdr, WAV_HEADER_SIZE);
        wf.write(pkt.pcmBuf, pkt.pcmSize);
        wf.close();
        LOGV("OK  saved %s (%u bytes)\n", path, (unsigned)(WAV_HEADER_SIZE + data_size));
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.pcmBuf);
    }

    // 4) End-of-window Foto speichern — wurde vom endPhotoTask im Hintergrund
    //    aufgenommen und als RAM-Buffer übergeben. Kein esp_camera_fb_get im
    //    Writer-Hotpath mehr — entkoppelt Kamera-Latenz vom SD-Write-Pfad.
    if (pkt.endImgBuf && pkt.endImgLen > 0) {
      snprintf(path, sizeof(path), "%s/img_%lu_%lu.jpg", bucketDir, pkt.idx, pkt.ts_ms);
      if (File img = SD.open(path, FILE_WRITE)) {
        img.write(pkt.endImgBuf, pkt.endImgLen);
        img.close();
        LOGV("OK  saved %s (%u bytes)\n", path, (unsigned)pkt.endImgLen);
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.endImgBuf);
    }

    // 5) Mid-window Foto speichern (bereits in samplerTask gecaptured)
    if (pkt.midImgBuf && pkt.midImgLen > 0) {
      snprintf(path, sizeof(path), "%s/mid_%lu_%lu.jpg", bucketDir, pkt.idx, pkt.ts_ms);
      if (File mf = SD.open(path, FILE_WRITE)) {
        mf.write(pkt.midImgBuf, pkt.midImgLen);
        mf.close();
        LOGV("OK  saved %s (%u bytes)\n", path, (unsigned)pkt.midImgLen);
      } else {
        Serial.printf("ERR failed to open %s\n", path);
      }
      free(pkt.midImgBuf);
    }

    // Writer-Durchsatz-Check: erst ab ~1 s wird es für den 1-Hz-Paketstrom
    // wirklich knapp. 950 ms meldet nur noch die relevanten Ausreißer.
    const uint32_t writeMs = millis() - writeStart;
    if (writeMs > 950) {
      Serial.printf("WARN writer slow: pkt %lu took %lu ms\n", pkt.idx, writeMs);
    }
  }
}
