# OnlyFeet – XIAO ESP32S3 Sense Data Logger

This PlatformIO project logs multi-sensor data from a Seeed Studio XIAO ESP32S3 Sense to a microSD card.

**Core Functionality:**
- **High-Frequency ToF:** Reads the VL53L1X Time-of-Flight sensor continuously (~20 Hz) in a dedicated, high-priority task.
- **Continuous IMU:** Samples the ICM-20948 IMU (accelerometer and gyroscope) at ~100 Hz, reading data from a continuously running FIFO buffer.
- **Per-Second Packets:** Every second, the system:
  - Records 1 second of mono 16-kHz audio from the PDM microphone.
  - Captures a QVGA JPEG image from the camera.
  - Bundles all sensor data (all ToF and IMU readings from the last second), the audio, and the image into a packet.
  - Writes the data to the SD card (`.json`, `.wav`, `.jpg`) and prints the JSON to the serial port.

The code is in `src/main.cpp`; camera GPIOs are defined in `include/camera_pins.h`.

## Hardware
- Board: Seeed Studio XIAO ESP32S3 Sense (with PSRAM)
- IMU: ICM‑20948 (I2C address 0x68)
- ToF: VL53L1X (I2C)
- Camera: integrated on Sense module (parallel)
- Microphone: PDM, read via I2S
- Storage: microSD over SPI

Key pins (as used by this firmware):
- PDM mic: `DATA=GPIO42`, `CLK=GPIO41`
- microSD: `CS=GPIO21` (other SPI pins use board defaults)
- Camera: see `include/camera_pins.h` for the full mapping

## Build & Flash
Prerequisites (Windows recommended steps):
- VS Code with the PlatformIO IDE extension, or PlatformIO Core (CLI)
- USB drivers for the board; identify your `COM` port

Common commands (PowerShell):
```powershell
# Build
pio run

# Upload (adjust port if needed)
pio run -t upload --upload-port COM5

# Serial monitor (baud must match firmware)
pio device monitor --port COM5 --baud 115200
```

Project environment (`platformio.ini`):
- Platform: `espressif32`
- Board: `seeed_xiao_esp32s3`
- Framework: `arduino`
- Build flags: `CAMERA_MODEL_XIAO_ESP32S3`, PSRAM enabled
- Libs: `esp32-camera`, `ArduinoJson`, `ICM20948_WE`, `VL53L1X`

## Firmware Behavior
- **Scheduling:** The system uses three main FreeRTOS tasks:
  - `tofTask`: A high-priority task that continuously reads the ToF sensor as fast as data is available.
  - `samplerTask`: Runs on a strict 1-second cadence to record audio and collect sensor data from the IMU and ToF queues.
  - `writerTask`: A lower-priority task that handles the slower operations of writing all files (JSON, WAV, JPG) to the SD card.
- **Data Flow & Synchronization:**
  - The ToF sensor runs continuously. A dedicated task reads the data and places it in a queue, overwriting the oldest sample if the queue is full.
  - The IMU FIFO runs continuously without being reset.
  - Once per second, the `samplerTask` gathers all pending ToF readings and all available IMU data from the last second into a single data packet for the `writerTask`.
- **Boot:** waits ~8 s to allow USB‑Serial to connect, initializes I2C (400 kHz), sensors, camera, SD, and I2S mic.
- **Camera:** QVGA (320×240), JPEG quality 12, 24 MHz XCLK, 2 frame buffers in PSRAM.
- **Audio:** 16 kHz, 16‑bit, mono; recorded in 1 s chunks.

## Files Written to SD (root)
- JSON packet: `/pkt_<index>.json`
- Photo: `/img_<index>_<ts>.jpg`
- Audio: `/rec_<index>_<ts>.wav`

Where `<index>` is a monotonically increasing counter and `<ts>` is `millis()` at capture time.

Example JSON fields:
```json
{
  "ts": 123456,
  "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
  "tof": [
    {"t": 10, "r": 1230, "s": 0},
    {"t": 60, "r": 1231, "s": 0},
    {"t": 110, "r": 1231, "s": 0}
    // ... up to ~20 entries per second
  ],
  "IMU": [
    {"i": 0, "a": {"x": 0.01, "y": 0.02, "z": 0.98}, "g": {"x": 0.1, "y": -0.1, "z": 0.0}}
    // ... up to ~100 entries per second
  ]
}
```

## Notes & Limitations
- Timing: The `samplerTask` maintains the 1 s grid; SD/photo writes are offloaded to a `writerTask` to avoid jitter.
- SD layout: files are written to the card root. A session folder structure is not implemented yet.
- Serial encoding: some status messages may include non‑ASCII symbols depending on your terminal font/encoding.

## Customization
- Adjust camera frame size/quality in `src/main.cpp` (camera config)
- Change audio sample rate/length via `AUDIO_FS` and `AUDIO_SEC`
- Tweak IMU FIFO sample rate/dividers in `setupIMU()`
- Change microSD CS or mic pins in the constants at the top of `src/main.cpp`
- Adjust task priorities, stack sizes, and queue lengths in the configuration section of `src/main.cpp`.

## Troubleshooting
- SD init fails: confirm wiring and card format (FAT32), check `CS=GPIO21`, try a different card
- Garbled Serial: ensure monitor baud `115200` matches `Serial.begin(115200)`
- Camera init errors: ensure PSRAM is enabled (see `platformio.ini`) and adequate 3.3 V supply
- Quiet/No audio: verify PDM pins 41/42 and microphone orientation; keep sample rate at 16 kHz to start

## Roadmap
- Session directories with `meta.json` (e.g., `/OF/SESSION_.../`)
- Decouple blocking operations (photo/audio) from the sampler task tick to further reduce jitter.
- Basic serial command interface (start/stop/status)

## License
Add a license if you plan to publish (e.g., MIT/Apache‑2.0).