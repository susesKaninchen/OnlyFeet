# OnlyFeet – XIAO ESP32S3 Sense Data Logger

This PlatformIO project logs multi-sensor data on a Seeed Studio XIAO ESP32S3 Sense to a microSD card. Each second it:
- Reads ICM‑20948 IMU FIFO (accel + gyro, optional magnetometer)
- Reads VL53L1X ToF range
- Captures a JPEG frame from the on‑board camera
- Records 1 s of mono 16‑kHz audio from the PDM microphone
- Emits a JSON packet over Serial and writes JSON/JPEG/WAV files to the SD card root

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
- Boot: waits ~8 s to allow USB‑Serial to connect, initializes I2C (400 kHz), sensors, camera, SD, and I2S mic
- Scheduling: FreeRTOS tasks with strict 1 s cadence via `vTaskDelayUntil`
- Synchronization: IMU FIFO is reset at the start of each 1 s audio window; audio and IMU are aligned to the same window
- Camera: QVGA (320×240), JPEG quality 12, 24 MHz XCLK, 2 frame buffers in PSRAM
- Audio: 16 kHz, 16‑bit, mono; recorded in 1 s chunks and written by a writer task
- IMU: ACC+GYR FIFO in continuous mode (~100 Hz effective), optional magnetometer
- ToF: continuous mode with 50 ms timing budget (long‑distance mode)

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
  "tof": {"r": 1234, "s": 0},
  "IMU": [
    {"i": 0, "a": {"x": 0.01, "y": 0.02, "z": 0.98}, "g": {"x": 0.1, "y": -0.1, "z": 0.0}}
    // ... up to ~120 entries per second
  ]
}
```

## Notes & Limitations
- Timing: The sampler task maintains the 1 s grid; SD/photo writes are offloaded to a writer task to avoid jitter.
- SD layout: files are written to the card root. A session folder structure and `meta.json` are not implemented yet.
- Serial encoding: some status messages may include non‑ASCII symbols depending on your terminal font/encoding.

## Customization
- Adjust camera frame size/quality in `src/main.cpp` (camera config)
- Change audio sample rate/length via `AUDIO_FS` and `AUDIO_SEC`
- Tweak IMU FIFO sample rate/dividers in `setupIMU()`
- Change microSD CS or mic pins in the constants at the top of `src/main.cpp`

## Troubleshooting
- SD init fails: confirm wiring and card format (FAT32), check `CS=GPIO21`, try a different card
- Garbled Serial: ensure monitor baud `115200` matches `Serial.begin(115200)`
- Camera init errors: ensure PSRAM is enabled (see `platformio.ini`) and adequate 3.3 V supply
- Quiet/No audio: verify PDM pins 41/42 and microphone orientation; keep sample rate at 16 kHz to start

## Roadmap
- Session directories with `meta.json` (e.g., `/OF/SESSION_.../`)
- Decouple/blocking operations (photo/audio) from the 1 s tick
- Basic serial command interface (start/stop/status)

## License
Add a license if you plan to publish (e.g., MIT/Apache‑2.0).
