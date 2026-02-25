# OnlyFeet – XIAO ESP32S3 Sense Data Logger

This PlatformIO project logs multi-sensor data from a Seeed Studio XIAO ESP32S3 Sense to a microSD card.

**Core Functionality:**
- **High-Frequency ToF:** Reads the VL53L5CX Time-of-Flight sensor continuously (15 Hz, 8×8 = 64 zones) in a dedicated task. Each frame yields a 64-element distance and status array.
- **Continuous IMU:** Samples the ICM-20948 IMU (accelerometer and gyroscope) at ~100 Hz, reading data from a continuously running FIFO buffer.
- **Per-Second Packets:** Every second, the system:
  - Records 1 second of mono 16-kHz audio from the PDM microphone.
  - Captures a QVGA JPEG image from the camera.
  - Bundles all sensor data (all ToF frames and IMU readings from the last second), the audio, and the image into a packet.
  - Writes the data to the SD card (`.json`, `.wav`, `.jpg`) and prints the JSON to the serial port.

The code is in `src/main.cpp`; camera GPIOs are defined in `include/camera_pins.h`.

## Hardware
- Board: Seeed Studio XIAO ESP32S3 Sense (with PSRAM)
- IMU: ICM‑20948 (I2C address 0x68)
- ToF: VL53L5CX (I2C) — 8×8 multizone, 15 Hz
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
- Libs: `esp32-camera`, `ArduinoJson`, `ICM20948_WE`, `SparkFun VL53L5CX Arduino Library`

## Firmware Behavior
- **Scheduling:** The system uses four FreeRTOS tasks:
  - `audioTask`: Highest-priority task that records 1 s chunks when triggered by the sampler and hands off buffers via a queue.
  - `tofTask`: Runs continuously (not window-triggered). Polls `isDataReady()` every 5 ms, reads the 64-zone frame via `getRangingData()`, and pushes it to a queue.
  - `samplerTask`: Runs on a 1-second window cadence. Flushes the ToF queue at each window boundary to ensure temporal isolation, then assembles a packet from audio, the IMU FIFO, and queued ToF frames, and triggers the next audio window.
  - `writerTask`: Handles the slower operations of writing JSON, WAV, and JPG files to the SD card.
- **Data Flow & Synchronization:**
  - The sampler defines each window boundary. At the start and end of every window the ToF queue is flushed so each packet contains only frames measured during that window.
  - The VL53L5CX delivers frames at its hardware rate (15 Hz, ~67 ms apart). Each frame covers all 64 zones simultaneously. Approximately 15 frames are collected per 1 s window.
  - The IMU FIFO runs continuously within each window. The sampler task stops and reads the FIFO once per window, then resets and restarts it.
  - At the end of each window, `samplerTask` gathers the queued ToF frames, IMU samples, and the audio buffer into a single data packet for the `writerTask`.
- **Boot:** waits ~8 s before acquisition so recordings start after handling artifacts, then initializes I2C (400 kHz), sensors, camera, SD, and I2S mic.
- **Camera:** QVGA (320×240), JPEG quality 30, 24 MHz XCLK, 2 frame buffers in PSRAM.
- **Audio:** 16 kHz, 16-bit, mono; recorded in 1 s chunks.

## Files Written to SD (`/dataN`)
On boot, the firmware creates the next available `/dataN` folder (N increments) so each session is kept separate.

- JSON packet: `/dataN/pkt_<index>.json`
- Photo: `/dataN/img_<index>_<ts>.jpg`
- Audio: `/dataN/rec_<index>_<ts>.wav`

Where `<index>` is a monotonically increasing counter and `<ts>` is `millis()` at capture time.

Example JSON fields:
```json
{
  "ts": 123456,
  "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
  "tof": [
    {"t": 12,  "d": [820, 815, 830, ...64 values...], "s": [5, 5, 5, ...64 values...]},
    {"t": 79,  "d": [821, 816, 831, ...64 values...], "s": [5, 5, 5, ...64 values...]},
    {"t": 146, "d": [819, 814, 829, ...64 values...], "s": [5, 5, 5, ...64 values...]}
  ],
  "IMU": [
    {"i": 0, "a": {"x": 0.01, "y": 0.02, "z": 0.98}, "g": {"x": 0.1, "y": -0.1, "z": 0.0}}
  ]
}
```

`"d"` contains the 64 distance values in mm (8×8 grid, row-major). `"s"` contains the corresponding target status codes (5 and 9 = valid measurement). `"t"` is the millisecond offset from the packet's `"ts"` timestamp.

**Note on zone layout:** The ST library returns zone data transposed relative to the datasheet's zone map. For correct spatial orientation, mirror horizontally (decreasing x, increasing y).

## Notes & Limitations
- Timing: The `samplerTask` maintains the 1 s grid; SD/photo writes are offloaded to a `writerTask` to avoid jitter.
- ToF rate: 8×8 resolution is hardware-limited to 15 Hz. For higher rates (up to 60 Hz) switch to 4×4 (`setResolution(4*4)`, `setRangingFrequency(60)`), which halves zone count to 16.
- SD layout: files are written to a new `/dataN` session folder on each boot.
- Serial encoding: some status messages may include non-ASCII symbols depending on your terminal font/encoding.

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
- ToF init fails: VL53L5CX requires several seconds to upload its firmware over I2C on first boot — the 8 s boot delay covers this; verify 3.3 V supply and I2C wiring (SDA/SCL)

## Viewer (`viewer/`)

A browser-based tool for replaying and visualizing recorded sensor sessions. Open `viewer/index.html` in a browser, select a `dataN` folder, and press Play.

**Panels:**
- **Camera** – Shows the JPEG image for each packet (full-frame fill).
- **IMU 3D Orientation** – A Three.js scene with a 3D box whose rotation is computed by a Madgwick AHRS filter (fusing accelerometer, gyroscope, and magnetometer). Includes axis arrows (R=X, G=Y, B=Z) and a red trail line showing the movement path.
- **ToF 8×8 Heightmap** – A Three.js 3D bar chart of the 64-zone distance data. Color encodes distance (red = near, blue = far). Animates through all ToF frames within each packet.
- **Accelerometer / Gyroscope / Magnetometer Charts** – Rolling-buffer Chart.js graphs updated at 100 Hz (sample-by-sample).

**Playback:**
- Timeline slider, play/pause, prev/next packet, speed control (0.5×–4×).
- Audio playback of the WAV file for each packet with volume control.
- All panels are synchronized to the playback clock.

**Dependencies (loaded via CDN):**
- Three.js r137 (with OrbitControls)
- Chart.js 4.4.1

**Local files:**
| File | Purpose |
|------|---------|
| `js/madgwick.js` | Standalone Madgwick AHRS filter (from [psiphi75/ahrs](https://github.com/psiphi75/ahrs), MIT) |
| `js/imu-orientation.js` | 3D board scene — rotation (AHRS) + position (double integration with ZUPT) |
| `js/imu-charts.js` | Rolling-buffer IMU charts at 100 Hz |
| `js/tof-view.js` | Three.js ToF heightmap |
| `js/timeline.js` | Playback engine, IMU sample-level animation |
| `js/audio-player.js` | Web Audio API WAV playback |
| `js/data-loader.js` | Folder/file loading via File API |
| `js/app.js` | Main orchestration |

**Known issues:**
- 3D position tracking (double integration of linear acceleration) drifts significantly. The rotation (Madgwick AHRS) works well, but the spatial movement path is unreliable and needs a better algorithm.

## Roadmap
- Session directories with `meta.json` (e.g., `/OF/SESSION_.../`)
- Decouple blocking operations (photo/audio) from the sampler task tick to further reduce jitter.
- Basic serial command interface (start/stop/status)
- Fix 3D position tracking in viewer (drift correction / alternative approach)

## License
Add a license if you plan to publish (e.g., MIT/Apache‑2.0).
