/**
 * app.js – Main entry: folder loading, orchestration
 */
(() => {
  const folderInput       = document.getElementById('folder-input');
  const phoneFolderInput  = document.getElementById('phone-folder-input');
  const phoneFolderLabel  = document.getElementById('phone-folder-label');
  const sessionInfo       = document.getElementById('session-info');
  const sessionHealth     = document.getElementById('session-health');
  const syncInfo          = document.getElementById('sync-info');
  const cameraImg         = document.getElementById('camera-img');
  const cameraLabel       = document.getElementById('camera-label');
  const volumeSlider      = document.getElementById('volume-slider');
  
  // Full foot-session data (untrimmed).
  let footData = null;
  let phoneData = null;
  let syncResult = null;

  /** Analyse packets for gaps, drops, missing media. Returns a summary object. */
  function analyzeHealth(packets) {
    const issues = [];
    const info = [];
    let tsGaps = 0;
    let lowImu = 0;
    let lowTof = 0;
    let missingJson = 0, missingWav = 0, missingMid = 0, missingEnd = 0;

    const firstIdx = packets[0].index;
    const lastIdx  = packets[packets.length - 1].index;
    const expected = lastIdx - firstIdx + 1;
    const gotIndices = new Set(packets.map(p => p.index));
    let indexGaps = 0;
    for (let i = firstIdx; i <= lastIdx; i++) {
      if (!gotIndices.has(i)) indexGaps++;
    }

    const lastPkt = packets[packets.length - 1];
    const lastPktIncomplete = lastPkt.json &&
      (!lastPkt.wavUrl || !lastPkt.imgUrl || !lastPkt.midImgUrl);

    let prevTs = null;
    for (let i = 0; i < packets.length; i++) {
      const p = packets[i];
      const isLast = (i === packets.length - 1);
      if (!p.json) missingJson++;
      if (!(isLast && lastPktIncomplete)) {
        if (!p.wavUrl)    missingWav++;
        if (!p.midImgUrl) missingMid++;
        if (!p.imgUrl)    missingEnd++;
      }
      if (p.json) {
        const imu = (p.json.IMU || []).length;
        const tof = (p.json.tof || []).length;
        if (imu < 100) lowImu++;
        if (tof < 12)  lowTof++;
        if (prevTs !== null) {
          const dt = p.ts - prevTs;
          if (Math.abs(dt - 1000) > 50 && dt > 0) tsGaps++;
        }
        prevTs = p.ts;
      }
    }

    if (indexGaps > 0)   issues.push(`${indexGaps} Paket${indexGaps>1?'e':''} verloren (queue drops)`);
    if (tsGaps > 0)      issues.push(`${tsGaps} Timestamp-Sprünge`);
    if (missingJson > 0) issues.push(`${missingJson}× JSON fehlt`);
    if (missingWav > 0)  issues.push(`${missingWav}× WAV fehlt`);
    if (missingMid > 0)  issues.push(`${missingMid}× mid-Foto fehlt`);
    if (missingEnd > 0)  issues.push(`${missingEnd}× end-Foto fehlt`);
    if (lowImu > 0)      issues.push(`${lowImu}× IMU < 100 Samples`);
    if (lowTof > 0)      issues.push(`${lowTof}× ToF < 12 Frames`);

    if (lastPktIncomplete) info.push(`letztes Paket #${lastPkt.index} unvollständig (Session-Ende)`);

    return { ok: issues.length === 0, issues, info, expected, got: packets.length };
  }

  function renderHealth(h) {
    const infoSuffix = (h.info && h.info.length) ? ` · ${h.info.join(' · ')}` : '';
    if (h.ok) {
      sessionHealth.className = 'health-ok';
      sessionHealth.innerHTML = `<strong>✓ OK</strong><span class="health-detail">${h.got}/${h.expected} Pakete${infoSuffix}</span>`;
    } else {
      const dropPct = (h.expected - h.got) / h.expected;
      const cls = (dropPct > 0.05) ? 'health-bad' : 'health-warn';
      sessionHealth.className = cls;
      sessionHealth.innerHTML = `<strong>⚠ Issues</strong><span class="health-detail">${h.got}/${h.expected} Pakete · ${h.issues.join(' · ')}${infoSuffix}</span>`;
    }
  }

  function renderSync(s) {
    if (!s) { syncInfo.className = ''; syncInfo.innerHTML = ''; return; }
    if (!s.ok && !s.weak) {
      syncInfo.className = 'sync-bad';
      syncInfo.innerHTML = `<strong>⚠ Sync fehlgeschlagen</strong><span class="sync-detail">${s.reason || 'unbekannt'}</span>`;
      return;
    }
    const cls = s.ok ? 'sync-ok' : 'sync-weak';
    const mark = s.ok ? '✓ Sync' : '⚠ Sync schwach';
    const anchorFoot = (s.anchorFootTs / 1000).toFixed(2) + ' s';
    const anchorPhoneDate = new Date(s.anchorPhoneTs).toISOString().slice(11, 23);
    syncInfo.className = cls;
    syncInfo.innerHTML = `<strong>${mark}</strong><span class="sync-detail">` +
      `${s.matchCount} Jump${s.matchCount>1?'s':''} gematcht · Ø ${Math.round(s.residualMs)} ms · ` +
      `Anker: Foot t=${anchorFoot} ↔ Phone ${anchorPhoneDate}` +
      `</span>`;
  }

  let currentPkt = null;

  function photoLabel(pkt, which) {
    const ts = which === 'mid' ? pkt.midTs : pkt.endTs;
    const fallback = which === 'mid' ? '~250' : '~750';
    return `${which}  ${ts != null ? ts : fallback} ms`;
  }

  function showMidPhoto(pkt) {
    const url = pkt.midImgUrl || pkt.imgUrl || null;
    if (url) cameraImg.src = url;
    else cameraImg.removeAttribute('src');
    cameraLabel.textContent = pkt.midImgUrl ? photoLabel(pkt, 'mid') : photoLabel(pkt, 'end');
  }

  function showEndPhoto(pkt) {
    const url = pkt.imgUrl || pkt.midImgUrl || null;
    if (url) cameraImg.src = url;
    else cameraImg.removeAttribute('src');
    cameraLabel.textContent = pkt.imgUrl ? photoLabel(pkt, 'end') : photoLabel(pkt, 'mid');
  }

  function photoSwitchThreshold(pkt) {
    const mid = pkt.midTs ?? 250;
    const end = pkt.endTs ?? 750;
    return (mid + end) / 2;
  }

  // Init components
  TofView.init();
  ImuCharts.init();
  ImuOrientation.init();
  AudioWaveforms.init();
  AudioPlayer.init();

  volumeSlider.addEventListener('input', () => {
    AudioPlayer.setVolume(parseFloat(volumeSlider.value));
  });

  function applySession(packets, fullFirstIndex, sessionWavUrl) {
    ImuCharts.reset();
    ImuCharts.setSessionData(packets, phoneData, syncResult ? syncResult.offsetMs : 0);
    
    ImuOrientation.reset();
    ImuOrientation.precomputePath(packets);

    AudioPlayer.loadSession(sessionWavUrl, {
      firstIndex: fullFirstIndex,
      packetDurationSec: 1.0
    });

    Timeline.init(packets, {
      onPacketChange: (pkt, idx, ctx) => {
        currentPkt = pkt;
        showMidPhoto(pkt);
        if (ctx && ctx.reason === 'seek') AudioPlayer.seekToPacket(pkt.index);
      },
      onTimeUpdate: (footTsMs) => {
        ImuCharts.setFootTime(footTsMs);
        AudioWaveforms.setFootTime(footTsMs);
        if (phoneData && syncResult && syncResult.offsetMs != null) {
          GpsMap.setFootTime(footTsMs);
        }
      },
      onPlay:  () => AudioPlayer.play(),
      onPause: () => AudioPlayer.pause(),
      onSpeedChange: (speed) => AudioPlayer.setPlaybackRate(speed),
      onImuSample: (sample, mag, sampleIdx, deltaTimeSec, absoluteSampleIdx) => {
        ImuOrientation.updateSample(sample, mag, deltaTimeSec);
      },
      onTofFrame: (frame, frameIdx) => {
        TofView.updateFrame(frame);
        if (currentPkt) {
          const tMs       = (frame && frame.t != null) ? frame.t : 0;
          const threshold = photoSwitchThreshold(currentPkt);
          if (tMs < threshold) showMidPhoto(currentPkt);
          else                 showEndPhoto(currentPkt);
        }
      }
    });
  }

  folderInput.addEventListener('change', async (e) => {
    const files = e.target.files;
    if (!files || !files.length) return;
    sessionInfo.textContent = 'Loading...';
    try {
      const data = await DataLoader.load(files);
      if (!data || !data.packets.length) {
        sessionInfo.textContent = 'No valid packets found.';
        return;
      }
      footData = data;
      phoneData = null;
      syncResult = null;
      renderSync(null);
      ImuCharts.disablePhoneOverlay();
      GpsMap.hide();

      sessionInfo.textContent = `Session: ${data.folderName}  (${data.packets.length} packets)`;
      renderHealth(analyzeHealth(data.packets));
      applySession(data.packets, data.packets[0].index, data.sessionWavUrl);

      AudioWaveforms.show();
      AudioWaveforms.load({
        footWavUrl:   data.sessionWavUrl,
        footStartMs:  data.packets[0].ts,
        phoneWavUrl:  null,
        phoneStartMs: null,
        sessionDurationMs: data.packets.length > 0 ? (data.packets[data.packets.length-1].ts - data.packets[0].ts + 1000) : 1000,
        viewStartMs:  data.packets[0].ts,
        viewEndMs:    data.packets[data.packets.length-1].ts + 1000
      });

      phoneFolderLabel.classList.remove('disabled');
      phoneFolderInput.disabled = false;
    } catch (err) {
      console.error('Error loading foot data:', err);
      sessionInfo.textContent = 'Error loading data: ' + err.message;
    }
  });

  phoneFolderInput.addEventListener('change', async (e) => {
    const files = e.target.files;
    if (!files || !files.length || !footData) return;

    syncInfo.className = '';
    syncInfo.innerHTML = '<span class="sync-detail">Phone-Daten werden verarbeitet…</span>';

    try {
      const phone = await PhoneLoader.load(files);
      if (!phone) { syncInfo.innerHTML = 'Phone-Ordner leer'; return; }
      phoneData = phone;

      const s = Sync.compute(footData.packets, phone);
      syncResult = s;
      renderSync(s);

      if (!s.ok && !s.weak) {
        return;
      }

      const clipped = Sync.clipPackets(footData.packets, s.commonStartFoot, s.commonEndFoot);
      if (!clipped.length) {
        renderHealth({ ok: false, expected: footData.packets.length, got: 0,
          issues: ['Gemeinsamer Bereich ist leer'], info: [] });
        return;
      }
      renderHealth(analyzeHealth(clipped));

      applySession(clipped, footData.packets[0].index, footData.sessionWavUrl);

      ImuCharts.enablePhoneOverlay();

      GpsMap.show();
      GpsMap.loadTrack(phone.gps || [], s.offsetMs);
      GpsMap.setFootTime(clipped[0].ts);

      // Audio-Waveforms: Wir nutzen den ersten IMU/GPS Punkt als Startzeit des Handy-Audios, 
      // falls in der WAV kein Header-Timestamp ist. 
      const phoneStarts = [];
      if (phone.imu && phone.imu.length) phoneStarts.push(phone.imu[0].ts);
      if (phone.gps && phone.gps.length) phoneStarts.push(phone.gps[0].ts);
      const phoneStartUnix  = phoneStarts.length ? Math.min(...phoneStarts) : null;
      
      // WICHTIG: Die Handy-WAV beginnt bei phoneStartUnix. In der Fuß-Zeitachse
      // (ms since boot) ist das: phoneStartUnix - s.offsetMs
      const phoneStartFoot = phoneStartUnix != null ? (phoneStartUnix - s.offsetMs) : null;
      
      AudioWaveforms.show();
      AudioWaveforms.load({
        footWavUrl:   footData.sessionWavUrl,
        footStartMs:  footData.packets[0].ts,
        phoneWavUrl:  phone.wavUrl || null,
        phoneStartMs: phoneStartFoot,
        sessionDurationMs: footData.packets[footData.packets.length-1].ts - footData.packets[0].ts + 1000,
        viewStartMs:  clipped[0].ts,
        viewEndMs:    clipped[clipped.length-1].ts + 1000
      }).then(() => AudioWaveforms.setFootTime(clipped[0].ts));
    } catch (err) {
      console.error('Error loading phone data:', err);
      syncInfo.className = 'sync-bad';
      syncInfo.innerHTML = `<strong>⚠ Sync-Fehler</strong><span class="sync-detail">${err.message}</span>`;
    }
  });

  window.addEventListener('resize', () => {
    TofView.onResize();
    ImuOrientation.onResize();
    GpsMap.onResize();
  });

  // ── Device load ──────────────────────────────────────────────────────────────
  const btnLoadDevice    = document.getElementById('btn-load-device');
  const modalOverlay     = document.getElementById('device-modal-overlay');
  const modalClose       = document.getElementById('device-modal-close');
  const sessionList      = document.getElementById('device-session-list');
  const modalStatus      = document.getElementById('device-modal-status');

  btnLoadDevice.addEventListener('click', async () => {
    modalOverlay.classList.add('open');
    sessionList.innerHTML = '';
    modalStatus.textContent = 'Lade Session-Liste…';
    try {
      const resp = await fetch('/api/files');
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const sessions = await resp.json();
      modalStatus.textContent = '';
      if (!sessions.length) {
        modalStatus.textContent = 'Keine Sessions auf der SD-Karte.';
        return;
      }
      for (const s of sessions) {
        const li = document.createElement('li');
        li.innerHTML = `<div class="sess-name">${s.session}</div>
          <div class="sess-info">${s.files.length} Dateien</div>`;
        li.addEventListener('click', () => loadDeviceSession(s));
        sessionList.appendChild(li);
      }
    } catch (e) {
      modalStatus.textContent = 'Fehler: ' + e.message;
    }
  });

  modalClose.addEventListener('click', () => {
    modalOverlay.classList.remove('open');
  });

  async function loadDeviceSession(s) {
    modalStatus.textContent = 'Lade Pakete…';
    sessionList.innerHTML = '';
    try {
      const data = await DataLoader.loadFromDevice(s.files, s.session, (loaded, total) => {
        modalStatus.textContent = `Lade… ${loaded}/${total}`;
      });
      modalOverlay.classList.remove('open');
      if (!data || !data.packets.length) {
        sessionInfo.textContent = 'Keine gültigen Pakete gefunden.';
        return;
      }
      footData    = data;
      phoneData   = null;
      syncResult  = null;
      renderSync(null);
      ImuCharts.disablePhoneOverlay();
      GpsMap.hide();
      sessionInfo.textContent = `Session: ${data.folderName}  (${data.packets.length} packets) [Gerät]`;
      renderHealth(analyzeHealth(data.packets));
      applySession(data.packets, data.packets[0].index, data.sessionWavUrl);
      AudioWaveforms.show();
      AudioWaveforms.load({
        footWavUrl:   data.sessionWavUrl,
        footStartMs:  data.packets[0].ts,
        phoneWavUrl:  null,
        phoneStartMs: null,
        sessionDurationMs: data.packets[data.packets.length-1].ts - data.packets[0].ts + 1000,
        viewStartMs:  data.packets[0].ts,
        viewEndMs:    data.packets[data.packets.length-1].ts + 1000
      });
      phoneFolderLabel.classList.remove('disabled');
      phoneFolderInput.disabled = false;
    } catch (e) {
      modalStatus.textContent = 'Fehler: ' + e.message;
      console.error(e);
    }
  }
})();
