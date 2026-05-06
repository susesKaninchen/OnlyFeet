/**
 * gps-map.js – Leaflet-based OpenStreetMap view of the phone GPS track.
 *
 * Activated when phone data is loaded: replaces the IMU 3D orientation panel
 * content with a map showing the full GPS polyline plus a live marker that
 * follows playback. foot-time → phone-time conversion uses Sync.offsetMs.
 */
const GpsMap = (() => {
  let map = null;
  let trackLayer = null;
  let marker = null;
  let gpsData = [];        // sorted by ts (absolute unix-ms)
  let offsetMs = 0;        // phoneTs ≈ footTs + offsetMs
  let container = null;
  let speedOverlay = null;

  function ensureContainer() {
    container = document.getElementById('gps-map-container');
    speedOverlay = document.getElementById('map-speed-overlay');
    if (!container) return false;
    return true;
  }

  /** Attach / build map the first time it's shown. */
  function initIfNeeded() {
    if (map) return true;
    if (!ensureContainer()) return false;
    if (typeof L === 'undefined') {
      console.warn('[GpsMap] Leaflet not loaded');
      return false;
    }
    
    // Ensure container is visible and has a height before initializing map
    const originalDisplay = container.style.display;
    container.style.display = 'block';
    if (container.clientHeight === 0) {
        container.style.height = '100%';
    }
    
    try {
        map = L.map(container, { zoomControl: true, attributionControl: true });
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
          maxZoom: 19,
          attribution: '© OpenStreetMap'
        }).addTo(map);
        map.setView([0, 0], 2);
    } catch (e) {
        console.error('[GpsMap] L.map init failed:', e);
    }
    
    // Restore display or keep it block if show() was called
    if (originalDisplay === 'none') {
        container.style.display = 'none';
    }

    return true;
  }

  /** Show the map container and hide the Three.js orientation canvas. */
  function show() {
    const orient = document.getElementById('imu-orient-container');
    const title  = document.getElementById('imu-orient-title');
    if (!ensureContainer()) return;
    
    if (orient) orient.style.display = 'none';
    container.style.display = 'block';
    if (title) title.textContent = 'GPS Track (OpenStreetMap)';
    
    if (!map) initIfNeeded();

    // Leaflet needs multiple resize kicks after becoming visible to ensure container has layout
    if (map) {
      setTimeout(() => map.invalidateSize(), 50);
      setTimeout(() => map.invalidateSize(), 200);
    }
  }

  function hide() {
    const orient = document.getElementById('imu-orient-container');
    const title  = document.getElementById('imu-orient-title');
    if (orient) orient.style.display = '';
    if (container) container.style.display = 'none';
    if (title) title.textContent = 'IMU 3D Orientation';
  }

  /** Render a GPS track (unix-ms timestamps) + anchor the foot→phone offset. */
  function loadTrack(gps, syncOffsetMs) {
    if (!ensureContainer()) return;
    if (!initIfNeeded()) return;
    gpsData = (gps || []).filter(p =>
      isFinite(p.lat) && isFinite(p.lon) && Math.abs(p.lat) <= 90 && Math.abs(p.lon) <= 180
    );
    offsetMs = syncOffsetMs || 0;

    if (trackLayer) { trackLayer.remove(); trackLayer = null; }
    if (marker)     { marker.remove();     marker = null; }

    if (!gpsData.length) {
      // No valid fixes — just recenter on 0/0 so Leaflet shows something
      map.setView([0, 0], 2);
    } else {
      const latlngs = gpsData.map(p => [p.lat, p.lon]);
      trackLayer = L.polyline(latlngs, { color: '#e94560', weight: 3, opacity: 0.9 }).addTo(map);
      marker = L.circleMarker(latlngs[0], {
        radius: 6, color: '#fff', weight: 2, fillColor: '#e94560', fillOpacity: 1
      }).addTo(map);

      if (latlngs.length > 1) {
        map.fitBounds(trackLayer.getBounds(), { padding: [20, 20] });
      } else {
        map.setView(latlngs[0], 17);
      }
    }
    
    // Kick resize again after loading track
    setTimeout(() => map.invalidateSize(), 100);
  }

  /** Move the marker to the GPS fix closest to the current foot-time. */
  function setFootTime(footTsMs) {
    if (!marker || !gpsData.length) return;
    const phoneTs = footTsMs + offsetMs;
    // Binary search for floor index
    let lo = 0, hi = gpsData.length - 1, idx = 0;
    while (lo <= hi) {
      const m = (lo + hi) >> 1;
      if (gpsData[m].ts <= phoneTs) { idx = m; lo = m + 1; } else { hi = m - 1; }
    }
    const a = gpsData[idx];
    const b = gpsData[idx + 1];
    let lat = a.lat, lon = a.lon, speed = a.speed || 0;
    if (b && b.ts > a.ts) {
      const t = Math.max(0, Math.min(1, (phoneTs - a.ts) / (b.ts - a.ts)));
      lat = a.lat + (b.lat - a.lat) * t;
      lon = a.lon + (b.lon - a.lon) * t;
      speed = a.speed + (b.speed - a.speed) * t;
    }
    marker.setLatLng([lat, lon]);
    if (speedOverlay) {
        speedOverlay.textContent = (speed * 3.6).toFixed(1) + ' km/h';
    }
  }

  function onResize() { if (map) map.invalidateSize(); }

  return { loadTrack, setFootTime, show, hide, onResize };
})();
