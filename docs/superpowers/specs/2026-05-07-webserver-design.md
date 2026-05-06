# Web Server Mode — Design Spec
**Datum:** 2026-05-07  
**Scope:** `src/main_legacy.cpp`, `data/viewer/`, `partitions.csv`, `platformio.ini`

---

## Ziel

Optionaler WiFi-Web-Server-Modus für die Legacy-Hardware. Während des Boot-Delays scannt der ESP nach einem bekannten Hotspot. Findet er ihn, verbindet er sich und startet einen Web-Server mit Live-Dashboard, Viewer und SD-Download. Findet er ihn nicht, bleibt WiFi aus und der ESP nimmt normal auf.

---

## Architektur

### Boot-Ablauf

```
setup() startet
  └─ 8s delay (bestehend)
       ├─ WiFi-Scan: sucht konfiguriertes SSID
       │    ├─ Gefunden → STA verbinden → webTask starten → Web-Modus
       │    └─ Nicht gefunden → WiFi aus → normaler Aufnahme-Modus
       └─ restliche Initialisierung (IMU, ToF, Audio, Kamera, SD)
```

### FreeRTOS Tasks im Web-Modus

Alle bestehenden Tasks laufen weiter:
- `samplerTask` (Core 0) — IMU/ToF/Audio/Foto sammeln
- `writerTask` (Core 1) — schreibt auf SD **nur wenn** `gRecording == true`
- `tofTask`, `audioTask`, `midPhotoTask`, `endPhotoTask` — unverändert

Neuer Task:
- `webTask` (Core 1, Prio 2) — synchroner WebServer auf Port 80

### Globale Zustands-Flags

```cpp
volatile bool gRecording = false;   // Start/Stop per API
volatile bool gWifiMode  = false;   // true wenn Web-Modus aktiv
SemaphoreHandle_t gSdMutex = nullptr; // SD-Bus-Schutz: writerTask + webTask
```

`gSdMutex` wird in `setup()` erstellt. Jeder SD-Zugriff in `writerTask` und `webTask` nimmt den Mutex (max. 500 ms Timeout). So können Downloads und Aufnahme gleichzeitig laufen ohne Bus-Konflikt.

### Live-Daten Shared State

Ein kleines `WebStatus`-Struct wird von `samplerTask` nach jedem Paket atomar aktualisiert und von `webTask` für `/api/status` gelesen (kein Mutex nötig, single-writer/single-reader, primitiv genug):

```cpp
struct WebStatus {
  float ax, ay, az, gx, gy, gz;
  float mag_x, mag_y, mag_z;
  int16_t tof_mm;
  uint8_t tof_status;
  bool tof_valid;
  uint32_t uptime_ms;
  uint32_t packet_count;
};
volatile WebStatus gWebStatus;
```

---

## WiFi-Konfiguration

### Standard (hardcoded in Firmware)

```cpp
#define WIFI_SSID_DEFAULT "MeinHotspot"
#define WIFI_PASS_DEFAULT "MeinPasswort"
```

### Override via SD-Karte

Datei `/wifi.cfg` auf SD-Karte (wird beim Boot gelesen, vor WiFi-Scan):

```
ssid=MeinHotspot
pass=MeinPasswort
```

Zeilenbasiertes Key-Value-Format, kein JSON, kein CRLF-Problem.

---

## LittleFS

### Partition Table (`partitions.csv`)

```
# Name,   Type, SubType, Offset,  Size,    Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x1C0000,
app1,     app,  ota_1,   0x1D0000,0x1C0000,
spiffs,   data, spiffs,  0x390000,0x80000,
```

512 KB LittleFS (spiffs-SubType, von Arduino-LittleFS-Library genutzt).

### Dateistruktur

```
/viewer/index.html         ← angepasst (neuer "Von Gerät laden" Button)
/viewer/css/style.css      ← unverändert
/viewer/js/madgwick.js     ← unverändert
/viewer/js/data-loader.js  ← angepasst (Device-Modus)
/viewer/js/app.js          ← minimal angepasst
/viewer/js/timeline.js     ← unverändert
/viewer/js/tof-view.js     ← unverändert
/viewer/js/imu-charts.js   ← unverändert
/viewer/js/imu-orientation.js ← unverändert
/viewer/js/audio-player.js ← unverändert
/viewer/js/audio-waveforms.js ← unverändert
/viewer/js/gps-map.js      ← unverändert
/viewer/js/phone-loader.js ← unverändert
/viewer/js/sync.js         ← unverändert
```

CDN-Links (Three.js, Chart.js, Leaflet) bleiben im HTML — Browser hat Internetzugang über den Hotspot.

Upload via: `pio run -e xiao_esp32s3_sense_legacy --target uploadfs`

---

## REST API

| Endpunkt | Methode | Beschreibung |
|---|---|---|
| `/` | GET | `index.html` aus LittleFS |
| `/viewer/*` | GET | Statische Dateien aus LittleFS |
| `/api/status` | GET | Live-Status JSON (siehe unten) |
| `/api/start` | POST | `gRecording = true`, neues `/dataN` anlegen |
| `/api/stop` | POST | `gRecording = false` |
| `/api/photo` | GET | Aktuelles Kamerabild als JPEG |
| `/api/files` | GET | JSON-Liste aller Sessions + Dateien auf SD |
| `/api/download?path=...` | GET | Einzelne Datei von SD streamen |
| `/api/download-folder?path=...` | GET | Ordner als unkomprimiertes ZIP streamen |

### `/api/status` Response

```json
{
  "recording": false,
  "packet_count": 7,
  "uptime_ms": 12400,
  "init": {
    "sd":          true,
    "camera":      true,
    "imu":         true,
    "magnetometer":false,
    "tof":         true,
    "audio":       true,
    "wifi":        true
  },
  "imu": { "ax": 0.01, "ay": -0.02, "az": 1.00,
           "gx": 0.10, "gy":  0.00, "gz": -0.10 },
  "tof": { "distance_mm": 342, "status": 0 },
  "mag": { "x": 12.3, "y": -5.1, "z": 44.2 },
  "log": [
    { "ts": 100, "msg": "IMU: ICM20948 OK (0x68)" },
    { "ts": 200, "msg": "TOF: VL53L1X OK (Long, 55ms)" },
    { "ts": 300, "msg": "Camera: OK" },
    { "ts": 500, "msg": "SD: mounted, 28413 MB free" },
    { "ts": 800, "msg": "WiFi: connected 192.168.1.42" }
  ],
  "recent_errors": [
    { "ts": 11500, "msg": "mid-photo: stale idx=3" }
  ]
}
```

### `/api/files` Response

```json
[
  {
    "session": "data0",
    "path": "/data0",
    "packets": 47,
    "size_kb": 1240
  },
  {
    "session": "data1",
    "path": "/data1",
    "packets": 12,
    "size_kb": 318
  }
]
```

### `/api/download-folder` ZIP-Streaming

Unkomprimiertes ZIP (Store-Modus), Datei für Datei direkt von SD gestreamt.  
Kein vollständiger ZIP im RAM — jede Datei wird header-by-header geschrieben.  
`Content-Disposition: attachment; filename="data0.zip"`

---

## Viewer-Anpassung

### `index.html` — neuer Button im Header

```html
<button id="btn-load-device" class="btn btn-secondary">Von Gerät laden</button>
```

Öffnet ein Modal mit der Session-Liste von `/api/files`.

### `data-loader.js` — Device-Modus

Neue Funktion `loadFromDevice(sessionPath)`:
1. Fetch `/api/files` → Session-Liste
2. User wählt Session
3. Fetch alle `pkt_N.json` via `/api/download?path=...`
4. Fetch WAV-Dateien analog
5. Übergibt Daten an bestehende App-Pipeline (identisches Format)

Bestehender File-Input-Modus bleibt unverändert.

---

## System-Log

Ein globales Ring-Log (max. 20 Einträge) wird während `setup()` gefüllt:

```cpp
struct LogEntry { uint32_t ts_ms; char msg[80]; };
static LogEntry gLog[20];
static uint8_t  gLogCount = 0;

void sysLog(const char* msg) {
  Serial.println(msg);
  if (gLogCount < 20) {
    gLog[gLogCount++] = { millis(), "" };
    strncpy(gLog[gLogCount-1].msg, msg, 79);
  }
}
```

Alle bestehenden `Serial.println("OK. ...")` werden auf `sysLog(...)` umgestellt.  
Recent-Errors kommen aus dem bestehenden `pkt.errorMsgs`-Mechanismus (letztes Paket).

---

## Fehlerbehandlung

- WiFi-Verbindung schlägt nach 10s fehl → WiFi aus, normaler Modus
- LittleFS mount fehlschlägt → Web-Modus deaktiviert, Warnung auf Serial
- SD nicht verfügbar bei `/api/files` → leeres Array zurück
- `/api/download` Datei nicht gefunden → HTTP 404

---

## Nicht im Scope

- OTA-Updates
- Authentifizierung / Passwortschutz des Web-Servers
- WebSocket (Polling reicht)
- AP-Modus als Fallback
