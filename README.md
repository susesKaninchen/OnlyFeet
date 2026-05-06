# OnlyFeet – XIAO ESP32S3 Sense Data Logger

This PlatformIO project logs multi-sensor data from a Seeed Studio XIAO ESP32S3 Sense to a microSD card. Two firmware variants exist — the main build and a legacy build for older hardware.

---

## Build Variants

| Environment | File | Hardware |
|---|---|---|
| `xiao_esp32s3_sense` | `src/main.cpp` | ICM-20948 (SPI), VL53L5CX 8×8 ToF |
| `xiao_esp32s3_sense_legacy` | `src/main_legacy.cpp` | ICM-20948 (I2C), VL53L1X single-point ToF, optional WiFi web server |

---

## Legacy Build (`xiao_esp32s3_sense_legacy`)

### Hardware
- Board: Seeed Studio XIAO ESP32S3 Sense (with PSRAM)
- IMU: ICM-20948 (I2C, address 0x68)
- ToF: VL53L1X single-point (I2C, Long mode, 55 ms period)
- Camera: OV2640 or OV3660 (auto-detected), QVGA JPEG, 20 MHz XCLK
- Microphone: PDM via I2S (`DATA=GPIO42`, `CLK=GPIO41`)
- Storage: microSD over SPI (`CS=GPIO21`)

### WiFi Web Server Mode

During the ~8-second boot window the firmware scans for a configured WiFi hotspot. If found, it connects and starts a web server on port 80. If not found, WiFi is disabled and the device records normally.

**WiFi credentials** — default values are hardcoded in the firmware:
```cpp
#define WIFI_SSID_DEFAULT "MeinHotspot"
#define WIFI_PASS_DEFAULT "MeinPasswort"
```

Override without reflashing by placing a `/wifi.cfg` file on the SD card:
```
ssid=MyNetwork
pass=MyPassword
```

#### Web Interface

Open `http://<device-ip>/` in a browser (device IP is printed to serial on connect).

| URL | Description |
|---|---|
| `/` | Redirects to the Viewer |
| `/viewer/index.html` | Full sensor data viewer (served from LittleFS) |
| `/api/status` | Live sensor data + init log (JSON, poll at 500 ms) |
| `/api/start` | POST — start recording to SD |
| `/api/stop` | POST — stop recording |
| `/api/photo` | Live camera JPEG (current frame) |
| `/api/files` | List all sessions and their files on SD (JSON) |
| `/api/download?path=...` | Stream a single file from SD |
| `/api/download-folder?path=/data0` | Download entire session as ZIP |

#### `/api/status` Response

```json
{
  "recording": false,
  "packet_count": 7,
  "uptime_ms": 12400,
  "init": {
    "sd": true, "camera": true, "imu": true,
    "magnetometer": false, "tof": true, "audio": true, "wifi": true
  },
  "imu": { "ax": 0.01, "ay": -0.02, "az": 1.00, "gx": 0.1, "gy": 0.0, "gz": -0.1 },
  "tof": { "distance_mm": 342, "status": 0 },
  "mag": { "x": 12.3, "y": -5.1, "z": 44.2 },
  "log": [
    { "ts": 100, "msg": "OK. ICM20948 connected via I2C (0x68)" },
    { "ts": 200, "msg": "OK. VL53L1X ready (continuous, long mode)" },
    { "ts": 300, "msg": "OK. Camera init done" },
    { "ts": 500, "msg": "OK. SD ready (28413 MB)" },
    { "ts": 800, "msg": "WiFi: connected 192.168.1.42" }
  ],
  "recent_errors": []
}
```

#### Recording via Web

In web mode the device does **not** record automatically. Use the dashboard buttons or POST to `/api/start` and `/api/stop`. Each start creates a new `/dataN` directory.

#### Viewer from Device

The embedded viewer (`/viewer/index.html`) adds a **"Von Gerät laden"** button. Click it to browse sessions stored on the SD card and load them directly — no file transfer to PC needed. WAV and image files are fetched on demand via the REST API.

The existing "Foot-Daten laden" button (local folder from PC) remains available.

### Files Written to SD

Session folders: `/dataN/BBBB/` (bucket `BBBB` = 4-digit zero-padded number, 30 packets per bucket).

| File | Description |
|---|---|
| `pkt_<idx>.json` | Sensor data packet |
| `rec_<idx>_<ts>.wav` | 1-second 16 kHz mono PCM audio |
| `img_<idx>_<ts>.jpg` | End-of-window JPEG (at ~750 ms) |
| `mid_<idx>_<ts>.jpg` | Mid-window JPEG (at ~250 ms) |

Bucket layout supports up to 9999 buckets × 30 packets = **299,970 packets (~83 hours)**.

### Build & Flash

```powershell
# Build only
pio run -e xiao_esp32s3_sense_legacy

# Build + flash firmware
pio run -e xiao_esp32s3_sense_legacy --target upload --upload-port COM14

# Upload LittleFS (viewer files) — required once after first flash or viewer update
pio run -e xiao_esp32s3_sense_legacy --target uploadfs --upload-port COM14

# Serial monitor
pio device monitor --port COM14 --baud 115200
```

> **Note:** LittleFS contains the viewer HTML/JS files (`data/viewer/`). Flash it once after the first firmware upload, and again whenever viewer files change.

### Partition Layout

Custom `partitions.csv` (8 MB flash):

| Partition | Size | Purpose |
|---|---|---|
| nvs | 20 KB | Non-volatile storage |
| phy_init | 4 KB | PHY calibration |
| app0 | 3 MB | Firmware |
| spiffs | 1 MB | LittleFS (viewer files) |

---

## Main Build (`xiao_esp32s3_sense`)

**Hardware:** ICM-20948 via SPI, VL53L5CX 8×8 ToF.

### Core Functionality
- **High-Frequency ToF:** VL53L5CX at 15 Hz, 8×8 = 64 zones
- **Continuous IMU:** ICM-20948 at ~100 Hz via FIFO
- **Per-Second Packets:** Every second bundles IMU, ToF, audio, and two JPEG images into `.json` + `.wav` + `.jpg` files on SD

---

## Viewer (`viewer/`)

A browser-based tool for replaying and visualizing recorded sensor sessions. Open `viewer/index.html` in a browser, select a `dataN` folder, and press Play.

**Panels:**
- **Camera** – JPEG images per packet (mid-window + end-window, switches at ~500 ms)
- **IMU 3D Orientation** – Three.js scene with Madgwick AHRS rotation and ZUPT-aided position path
- **ToF Heightmap** – Three.js 3D bar chart of distance data
- **Accelerometer / Gyroscope / Magnetometer** – Rolling Chart.js graphs at 100 Hz
- **Audio Waveforms** – Full-session WAV visualization

**Playback:** Timeline slider, play/pause, speed control (0.5×–4×), audio with volume control.

**Dependencies (CDN):** Three.js r137, Chart.js 4.4.1, Leaflet 1.9.4

| File | Purpose |
|---|---|
| `js/madgwick.js` | Madgwick AHRS filter |
| `js/imu-orientation.js` | 3D board scene (rotation + ZUPT position) |
| `js/imu-charts.js` | Rolling IMU charts |
| `js/tof-view.js` | ToF heightmap |
| `js/timeline.js` | Playback engine |
| `js/audio-player.js` | Web Audio WAV playback |
| `js/data-loader.js` | File loading (local folder + device REST API) |
| `js/app.js` | Main orchestration + device-load UI |

---

## Key Pins

| Signal | GPIO |
|---|---|
| PDM mic DATA | 42 |
| PDM mic CLK | 41 |
| microSD CS | 21 |
| Camera | see `include/camera_pins.h` |
| ICM-20948 I2C | SDA=40, SCL=39 (legacy) |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| SD init fails | Wrong CS pin, FAT32 format required, check wiring |
| Camera shows green/purple pixels | XCLK too high for OV3660; firmware uses 20 MHz |
| ToF array empty in JSON | `startContinuous(0)` is invalid; firmware uses 55 ms |
| WiFi not connecting | Check SSID/password in firmware or `/wifi.cfg` on SD |
| LittleFS mount failed | Run `uploadfs` target after first flash |
| Viewer "Von Gerät laden" shows no sessions | Device not in WiFi mode, or SD has no `dataN` folders |

---

## License
Add a license if you plan to publish (e.g., MIT/Apache-2.0).
