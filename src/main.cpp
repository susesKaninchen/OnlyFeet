// main.cpp — FreeRTOS-scheduled sensor logger (ESP32S3 Sense)
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>

#include "esp_camera.h"
#include "camera_pins.h"   // XIAO Sense cam pins (CAMERA_MODEL_XIAO_ESP32S3)

#include <driver/i2s_pdm.h>    // For PDM/PCM interface
#include "FS.h"
#include "SD.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/********* Audio / SD configuration *********/
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
static constexpr int MIC_DATA_PIN = 42;
static constexpr int MIC_CLK_PIN  = 41;
static constexpr int SD_CS_PIN    = 21;
static constexpr uint16_t AUDIO_FS   = 16000;  // 16 kHz
static constexpr uint8_t  AUDIO_BITS = 16;     // 16-bit
static constexpr uint8_t  AUDIO_CHAN = 1;      // Mono
static constexpr uint32_t AUDIO_PCM_SIZE = AUDIO_FS * (AUDIO_BITS / 8) * AUDIO_CHAN; // 1 second of audio
static constexpr uint32_t WAV_HEADER_SIZE = 44;

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
  .jpeg_quality = 12, .fb_count     = 2,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_WHEN_EMPTY
};

/********* IMU & ToF settings *********/
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU(ICM20948_ADDR);
VL53L1X      tof;

constexpr uint32_t PACKET_INTERVAL_MS = 1000;
constexpr size_t   MAX_FIFO_SETS      = 120;
constexpr size_t   MAX_TOF_READINGS   = 30;

/********* Task & Queue Configuration *********/
static constexpr uint32_t JSON_DOC_SIZE = 16384;

static constexpr size_t PACKET_QUEUE_LEN = 2;
static constexpr size_t TOF_QUEUE_LEN    = 50;
static constexpr size_t AUDIO_QUEUE_LEN  = 2; // For passing buffer pointers

static constexpr uint32_t SAMPLER_STACK_SIZE = 8192;
static constexpr uint32_t WRITER_STACK_SIZE  = 8192;
static constexpr uint32_t TOF_STACK_SIZE     = 4096;
static constexpr uint32_t AUDIO_STACK_SIZE   = 4096;

// Higher number = higher priority. Audio must be highest to avoid losing samples.
static constexpr UBaseType_t AUDIO_PRIORITY   = 5;
static constexpr UBaseType_t SAMPLER_PRIORITY = 4;
static constexpr UBaseType_t WRITER_PRIORITY  = 3;
static constexpr UBaseType_t TOF_PRIORITY     = 3;

bool hasMag = false;
bool hasTOF = false;

struct IMUSample { float ax, ay, az, gx, gy, gz; };
struct TOFSample { uint32_t ts_ms; uint16_t range; uint8_t status; };
struct TOFReading { uint32_t ts_offset_ms; uint16_t range; uint8_t status; };

/********* Globals *********/
uint32_t  mediaCounter = 0;
char      dataDir[16]  = "/data0";
QueueHandle_t gPacketQueue = nullptr;
QueueHandle_t gTofQueue    = nullptr;
QueueHandle_t gAudioQueue  = nullptr; // For passing audio buffer pointers
i2s_chan_handle_t rx_handle = NULL;

// Packet sent from Sampler to Writer
struct Packet {
  uint32_t idx;
  uint32_t ts_ms;
  size_t   imuCount;
  IMUSample imu[MAX_FIFO_SETS];
  xyzFloat mag;
  size_t   tofCount;
  TOFReading tof[MAX_TOF_READINGS];
  uint8_t *pcmBuf; // malloc'ed raw PCM audio buffer (ownership passed to writer)
  size_t   pcmSize;
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
    for (;;) {}
  }

  /* ---- SD card ---- */
  Serial.println("Mounting SD …");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERR SD mount failed");
    for (;;) {}
  }
  Serial.println("OK. SD ready");

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
  gPacketQueue = xQueueCreate(PACKET_QUEUE_LEN, sizeof(Packet));
  gTofQueue = xQueueCreate(TOF_QUEUE_LEN, sizeof(TOFSample));
  gAudioQueue = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(uint8_t*)); // Pointer queue
  if (!gPacketQueue || !gTofQueue || !gAudioQueue) {
    Serial.println("ERR queue create failed");
    for(;;) {}
  }

  // Create tasks
  BaseType_t ok1 = xTaskCreatePinnedToCore(samplerTask, "sampler", SAMPLER_STACK_SIZE, nullptr, SAMPLER_PRIORITY, nullptr, 0);
  BaseType_t ok2 = xTaskCreatePinnedToCore(writerTask,  "writer",  WRITER_STACK_SIZE, nullptr, WRITER_PRIORITY, nullptr, 1);
  BaseType_t ok3 = xTaskCreatePinnedToCore(tofTask, "tof", TOF_STACK_SIZE, nullptr, TOF_PRIORITY, nullptr, 0);
  BaseType_t ok4 = xTaskCreatePinnedToCore(audioTask, "audio", AUDIO_STACK_SIZE, nullptr, AUDIO_PRIORITY, nullptr, 0);
  if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS) {
    Serial.println("ERR task create failed");
    for(;;) {}
  }
}

/********* Arduino loop **********/
void loop() {
  // All work is handled in FreeRTOS tasks, so this one is not needed.
  vTaskDelete(NULL);
}

// --- NEW AUDIO IMPLEMENTATION ---

/********* Audio Init *********/
void setupAudio() {
  Serial.println("Initialising microphone using I2S driver...");

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 4;
  chan_cfg.dma_frame_num = 1024;
  esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  if (err != ESP_OK) {
    Serial.printf("ERR I2S new channel failed: %d\n", err);
    for(;;)
 {}
  }

  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(AUDIO_FS),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
        .clk = (gpio_num_t)MIC_CLK_PIN,
        .din = (gpio_num_t)MIC_DATA_PIN,
    },
  };

  err = i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
  if (err != ESP_OK) {
    Serial.printf("ERR I2S init PDM RX mode failed: %d\n", err);
    for(;;)
 {}
  }

  err = i2s_channel_enable(rx_handle);
  if (err != ESP_OK) {
    Serial.printf("ERR I2S channel start failed: %d\n", err);
    for(;;)
 {}
  }
  Serial.println("OK. I2S driver ready.");
}

/********* Audio Task (asynchronous recording) *********/
void audioTask(void* arg) {
  size_t bytes_read = 0;

  for (;;) {
    // Allocate a new buffer for each audio chunk.
    // This buffer will be passed to other tasks and freed by the writerTask.
    uint8_t* pcm_buf = (uint8_t*)malloc(AUDIO_PCM_SIZE);
    if (!pcm_buf) {
        Serial.println("ERR audioTask failed to allocate pcm_buf");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
    }

    // Read from I2S DMA buffer into the new PCM buffer.
    // This call blocks until AUDIO_PCM_SIZE bytes are read.
    esp_err_t err = i2s_channel_read(rx_handle, pcm_buf, AUDIO_PCM_SIZE, &bytes_read, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("ERR I2S read failed: %d\n", err);
      free(pcm_buf); // Free the buffer if read fails
      continue; // Try again
    }

    // Check if the expected number of bytes were read
    if (bytes_read != AUDIO_PCM_SIZE) {
      Serial.printf("WARN I2S read only %u of %u bytes\n", bytes_read, (unsigned int)AUDIO_PCM_SIZE);
    }

    // Try to send the pointer of the *filled* buffer to the sampler task.
    // If the queue is full, we must free the buffer to avoid a memory leak.
    if (xQueueSend(gAudioQueue, &pcm_buf, pdMS_TO_TICKS(50)) != pdPASS) {
      Serial.println("WARN audio queue full, dropping audio data");
      free(pcm_buf);
    }
    // If send was successful, the writerTask now owns the buffer and is responsible for freeing it.
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
    tof.startContinuous(50);
    hasTOF = true;
    Serial.println("OK. VL53L1X ready (continuous)");
  } else {
    Serial.println("WARN VL53L1X init failed — continuing without");
  }
}

/********* ToF Task (continuous reading) *********/
void tofTask(void* arg) {
  if (!hasTOF) vTaskDelete(NULL);

  struct TOFSample sample;
  for (;;) {
    // Block until data is ready, more efficient than polling.
    tof.read(true);
    
    sample.ts_ms = millis();
    sample.range = tof.ranging_data.range_mm;
    sample.status = static_cast<uint8_t>(tof.ranging_data.range_status);
    
    // Send to queue. If full, the sample will be dropped.
    xQueueSend(gTofQueue, &sample, (TickType_t)0);
  }
}

/********* Sampler Task (precise 1 s windows) *********/
void samplerTask(void* arg) {
  const TickType_t periodTicks = pdMS_TO_TICKS(PACKET_INTERVAL_MS);
  TickType_t lastWake = xTaskGetTickCount();

  Serial.println("Sampler task: Waiting for initial audio buffer...");
  // Pre-fetch the first audio buffer to align windows
  uint8_t* pcmBuf = nullptr;
  if (xQueueReceive(gAudioQueue, &pcmBuf, portMAX_DELAY) != pdPASS) {
      Serial.println("ERR failed to get initial audio buffer");
      // Can't continue without the first buffer, so halt.
      for(;;){ vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  Serial.println("Sampler task: Initial audio buffer received.");

  for (;;) {
    // Maintain strict cadence. This call starts the 1s window.
    vTaskDelayUntil(&lastWake, periodTicks);
    uint32_t ts = millis();
    Serial.printf("Sampler task: New window at %lu ms.\n", ts);

    // --- Prepare packet ---
    Packet pkt{};
    pkt.idx = mediaCounter;
    pkt.ts_ms = ts;
    
    // The pcmBuf for this packet is the one we received in the *previous* iteration.
    // This ensures the audio is correctly aligned with the 1-second window.
    pkt.pcmBuf = pcmBuf;
    pkt.pcmSize = pcmBuf ? AUDIO_PCM_SIZE : 0;

    // --- Asynchronously fetch the audio buffer for the *next* packet ---
    Serial.println("Sampler task: Waiting for next audio buffer...");
    if (xQueueReceive(gAudioQueue, &pcmBuf, pdMS_TO_TICKS(PACKET_INTERVAL_MS + 100)) != pdPASS) {
        Serial.println("WARN sampler task timed out waiting for audio buffer.");
        // To prevent memory leaks, we must not lose the pointer to the current buffer.
        // Setting pcmBuf to null means we won't have audio for the next packet.
        pcmBuf = nullptr; 
    } else {
        Serial.println("Sampler task: Next audio buffer received.");
    }

    // --- Gather other sensor data at end of window ---
    myIMU.findFifoBegin();
    size_t count = myIMU.getNumberOfFifoDataSets();
    if (count > MAX_FIFO_SETS) count = MAX_FIFO_SETS;
    pkt.imuCount = count;
    for (size_t i = 0; i < count; ++i) {
      xyzFloat acc, gyr;
      myIMU.getGValuesFromFifo(&acc);
      myIMU.getGyrValuesFromFifo(&gyr);
      pkt.imu[i] = {acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z};
    }

    pkt.mag = {0,0,0};
    if (hasMag) {
      myIMU.readSensor();
      myIMU.getMagValues(&pkt.mag);
    }

    pkt.tofCount = 0;
    if (hasTOF) {
      TOFSample sample;
      while(xQueueReceive(gTofQueue, &sample, 0) == pdPASS && pkt.tofCount < MAX_TOF_READINGS) {
        pkt.tof[pkt.tofCount].ts_offset_ms = sample.ts_ms - pkt.ts_ms;
        pkt.tof[pkt.tofCount].range = sample.range;
        pkt.tof[pkt.tofCount].status = sample.status;
        pkt.tofCount++;
      }
    }

    // Enqueue for writer
    if (xQueueSend(gPacketQueue, &pkt, pdMS_TO_TICKS(50)) != pdPASS) {
      if (pkt.pcmBuf) free(pkt.pcmBuf);
      Serial.println("WARN packet queue full — dropping packet");
    } else {
      Serial.println("Sampler task: Packet sent to writer.");
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
      JsonObject e = tofArr.add<JsonObject>();
      e["t"] = pkt.tof[i].ts_offset_ms;
      e["r"] = pkt.tof[i].range;
      e["s"] = pkt.tof[i].status;
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
  for (;;) {
    Packet pkt;
    if (xQueueReceive(gPacketQueue, &pkt, portMAX_DELAY) != pdPASS) {
      continue;
    }

    // 1) Build JSON from packet
    JsonDocument doc;
    buildJson(pkt, doc);

    // Serial out
    serializeJson(doc, Serial);
    Serial.println();

    // 2) Save JSON
    char path[48];
    snprintf(path, sizeof(path), "%s/pkt_%lu.json", dataDir, pkt.idx);
    if (File f = SD.open(path, FILE_WRITE)) {
      serializeJson(doc, f);
      f.close();
      Serial.printf("OK  saved %s\n", path);
    } else {
      Serial.printf("ERR failed to open %s\n", path);
    }

    // 3) Create WAV file from PCM data and save
    if (pkt.pcmBuf && pkt.pcmSize > 0) {
      size_t wav_size = 0;
      uint8_t* wav_buffer = create_wav_file_buffer(pkt.pcmBuf, pkt.pcmSize, &wav_size);
      
      // The original PCM buffer is no longer needed, free it.
      free(pkt.pcmBuf);

      if (wav_buffer) {
        snprintf(path, sizeof(path), "%s/rec_%lu_%lu.wav", dataDir, pkt.idx, pkt.ts_ms);
        if (File wf = SD.open(path, FILE_WRITE)) {
          wf.write(wav_buffer, wav_size);
          wf.close();
          Serial.printf("OK  saved %s (%u bytes)\n", path, (unsigned)wav_size);
        } else {
          Serial.printf("ERR failed to open %s\n", path);
        }
        // Free the final WAV buffer
        free(wav_buffer);
      } else {
        Serial.println("ERR failed to create WAV buffer");
      }
    }

    // 4) Capture and save photo (non-critical timing)
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      snprintf(path, sizeof(path), "%s/img_%lu_%lu.jpg", dataDir, pkt.idx, pkt.ts_ms);
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