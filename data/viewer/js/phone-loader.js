/**
 * phone-loader.js – Parse the companion-phone session folder.
 *
 * Expected files (flat folder, name must match):
 *   imu.csv         columns: timestamp_ms, sensor, x, y, z   (sensor ∈ {accel, gyro})
 *   gps.csv         columns: timestamp_ms, latitude, longitude, altitude_m,
 *                            accuracy_m, speed_ms, heading_deg
 *   audio_phone.wav continuous phone-mic recording
 *   sync.json       { session, total_jumps, jump_events:[{jump_index, timestamp_ms, magnitude_ms2}] }
 *
 * Timestamps in all phone files are absolute unix-epoch milliseconds. Firmware
 * packets are `millis()` since boot — the sync module reconciles the two.
 */
const PhoneLoader = (() => {

  function parseCSV(text) {
    const lines = text.split(/\r?\n/).filter(l => l.trim().length);
    if (!lines.length) return { header: [], rows: [] };
    const header = lines[0].split(',').map(s => s.trim());
    const rows = new Array(lines.length - 1);
    for (let i = 1; i < lines.length; i++) {
      rows[i - 1] = lines[i].split(',');
    }
    return { header, rows };
  }

  async function load(fileList) {
    const files = Array.from(fileList);
    if (!files.length) return null;

    const firstPath = files[0].webkitRelativePath || files[0].name;
    const folderName = firstPath.includes('/') ? firstPath.split('/')[0] : 'phone';

    let imuFile = null, gpsFile = null, wavFile = null, syncFile = null;
    for (const f of files) {
      const n = f.name.toLowerCase();
      if (n === 'imu.csv')         imuFile  = f;
      else if (n === 'gps.csv')    gpsFile  = f;
      else if (n === 'sync.json')  syncFile = f;
      else if (n.endsWith('.wav')) wavFile  = f;  // audio_phone.wav
    }

    const out = { folderName, imu: [], gps: [], syncJumps: [], wavUrl: null };

    if (imuFile) {
      const txt = await imuFile.text();
      const { header, rows } = parseCSV(txt);
      const iTs = header.indexOf('timestamp_ms');
      const iS  = header.indexOf('sensor');
      const iX  = header.indexOf('x');
      const iY  = header.indexOf('y');
      const iZ  = header.indexOf('z');
      const imu = new Array(rows.length);
      let n = 0;
      for (const r of rows) {
        const ts = +r[iTs];
        if (!isFinite(ts)) continue;
        const sensor = (r[iS] || '').trim().toLowerCase();
        imu[n++] = {
          ts,
          sensor: sensor,
          x: +r[iX], y: +r[iY], z: +r[iZ]
        };
      }
      imu.length = n;
      out.imu = imu;
    }

    if (gpsFile) {
      const txt = await gpsFile.text();
      const { header, rows } = parseCSV(txt);
      const iTs  = header.indexOf('timestamp_ms');
      const iLat = header.indexOf('latitude');
      const iLon = header.indexOf('longitude');
      const iAlt = header.indexOf('altitude_m');
      const iAcc = header.indexOf('accuracy_m');
      const iSpd = header.indexOf('speed_ms');
      const iHdg = header.indexOf('heading_deg');
      const gps = new Array(rows.length);
      let n = 0;
      for (const r of rows) {
        const ts = +r[iTs];
        if (!isFinite(ts)) continue;
        const lat = +r[iLat];
        const lon = +r[iLon];
        if (isNaN(lat) || isNaN(lon)) continue;
        
        gps[n++] = {
          ts,
          lat, lon, 
          alt: +r[iAlt] || 0,
          acc: +r[iAcc] || 0, 
          speed: isFinite(+r[iSpd]) ? +r[iSpd] : 0, 
          heading: +r[iHdg] || 0
        };
      }
      gps.length = n;
      out.gps = gps;
    }

    if (syncFile) {
      try {
        const obj = JSON.parse(await syncFile.text());
        out.syncJumps = (obj.jump_events || []).map(j => ({
          idx: j.jump_index,
          ts:  j.timestamp_ms,
          mag: j.magnitude_ms2
        })).sort((a, b) => a.ts - b.ts);
      } catch (e) { console.warn('[PhoneLoader] sync.json parse failed', e); }
    }

    if (wavFile) out.wavUrl = URL.createObjectURL(wavFile);

    return out;
  }

  /** Binary search: index of largest ts <= target, or -1 if before first */
  function floorIndex(arr, ts, key = 'ts') {
    let lo = 0, hi = arr.length - 1, ans = -1;
    while (lo <= hi) {
      const m = (lo + hi) >> 1;
      if (arr[m][key] <= ts) { ans = m; lo = m + 1; } else { hi = m - 1; }
    }
    return ans;
  }

  /** Nearest sample to target ts (within maxDeltaMs or null). */
  function nearest(arr, ts, maxDeltaMs = 1000, key = 'ts') {
    if (!arr.length) return null;
    const i = floorIndex(arr, ts, key);
    const cands = [];
    if (i >= 0)            cands.push(arr[i]);
    if (i + 1 < arr.length) cands.push(arr[i + 1]);
    let best = null, bestDt = Infinity;
    for (const c of cands) {
      const dt = Math.abs(c[key] - ts);
      if (dt < bestDt) { bestDt = dt; best = c; }
    }
    return (best && bestDt <= maxDeltaMs) ? best : null;
  }

  return { load, nearest, floorIndex };
})();
