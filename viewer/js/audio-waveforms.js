/**
 * audio-waveforms.js – Präzisions-Sync via Kreuz-Korrelation auf absolutem Zeitraster
 */
const AudioWaveforms = (() => {
  let panel, canvas, ctx, cursorEl, infoEl;
  let footEnv = null, phoneEnv = null;
  let totalStartMs = 0, totalEndMs = 0;
  let initDone = false;

  const H_PAD = 8;
  const BAR_COLOR_FOOT  = 'rgba(233, 69, 96, 0.85)';
  const BAR_COLOR_PHONE = 'rgba(54, 162, 235, 0.65)';

  function init() {
    if (initDone) return;
    panel    = document.getElementById('audio-waveforms-panel');
    canvas   = document.getElementById('wf-canvas');
    cursorEl = document.getElementById('wf-cursor');
    infoEl   = document.getElementById('wf-info');
    if (!panel || !canvas) return;
    ctx = canvas.getContext('2d');
    initDone = true;
    new ResizeObserver(() => { resizeCanvas(); redraw(); }).observe(canvas);
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (rect.width > 0) {
        canvas.width  = Math.floor(rect.width  * dpr);
        canvas.height = Math.floor(rect.height * dpr);
    }
  }

  async function parseWavData(url) {
    if (!url) return null;
    try {
      const resp = await fetch(url);
      const buf  = await resp.arrayBuffer();
      if (buf.byteLength < 44) return null;
      const dv = new DataView(buf);
      if (dv.getUint32(0, true) !== 0x46464952) return null; 
      
      let offset = 12;
      let samples = null, channels = 1, sampleRate = 16000;
      while (offset < buf.byteLength - 8) {
        const chunkId = dv.getUint32(offset, true);
        const chunkSize = dv.getUint32(offset + 4, true);
        if (chunkId === 0x20746d66) { 
          channels = dv.getUint16(offset + 10, true);
          sampleRate = dv.getUint32(offset + 12, true);
        } else if (chunkId === 0x61746164) {
          const byteOffset = offset + 8;
          samples = new Int16Array(buf.slice(byteOffset, byteOffset + chunkSize));
          break; 
        }
        offset += 8 + chunkSize;
        if (chunkSize % 2 !== 0) offset++;
      }
      return samples ? { samples, channels, sampleRate, durationMs: (samples.length / (sampleRate * channels)) * 1000 } : null;
    } catch (e) { return null; }
  }

  // Erstellt ein Zeitraster mit 10ms Auflösung (100Hz) für den mathematischen Vergleich
  function buildTimeGrid(data, durationMs) {
    const msPerBin = 10;
    const bins = Math.ceil(durationMs / msPerBin);
    const grid = new Float32Array(bins);
    const samplesPerBin = Math.floor(data.samples.length / bins);
    
    for (let i = 0; i < bins; i++) {
      let maxAbs = 0;
      const start = i * samplesPerBin;
      const end = Math.min(data.samples.length, start + samplesPerBin);
      for (let k = start; k < end; k++) {
        const v = Math.abs(data.samples[k] / 32768);
        if (v > maxAbs) maxAbs = v;
      }
      // Quadrieren, um Klopfer (Peaks) gegenüber Rauschen massiv zu verstärken
      grid[i] = maxAbs * maxAbs; 
    }
    return grid;
  }

  async function load({ footWavUrl, footStartMs, phoneWavUrl, phoneStartMs, sessionDurationMs, viewStartMs, viewEndMs }) {
    init();
    const footData = await parseWavData(footWavUrl);
    if (!footData) return;
    
    const footGrid = buildTimeGrid(footData, footData.durationMs);
    footEnv = { grid: footGrid, durationMs: footData.durationMs, startMs: footStartMs };

    const phoneData = await parseWavData(phoneWavUrl);
    if (phoneData) {
      const phoneGrid = buildTimeGrid(phoneData, phoneData.durationMs);
      const baseStart = isFinite(phoneStartMs) ? phoneStartMs : footStartMs;
      phoneEnv = { grid: phoneGrid, durationMs: phoneData.durationMs, startMs: baseStart };

      // --- MATHEMATISCHER PRÄZISIONS-SYNC ---
      console.log('[AudioSync] Absolute cross-correlation start...');
      let bestShiftMs = 0, maxCorr = -1;
      
      // Suche im Bereich von +/- 60 Sekunden (6000 Bins)
      const searchRangeBins = 6000; 
      const msPerBin = 10;

      for (let shiftBins = -searchRangeBins; shiftBins <= searchRangeBins; shiftBins++) {
        let corr = 0;
        // Wir vergleichen den Bereich, in dem beide existieren
        const startI = Math.max(0, -shiftBins);
        const endI = Math.min(footGrid.length, phoneGrid.length - shiftBins);
        
        for (let i = startI; i < endI; i++) {
          corr += footGrid[i] * phoneGrid[i + shiftBins];
        }
        
        if (corr > maxCorr) {
          maxCorr = corr;
          bestShiftMs = shiftBins * msPerBin;
        }
      }
      
      console.log(`[AudioSync] Best match found at shift: ${bestShiftMs}ms`);
      // Shift positiv bedeutet: phone audio ist verzögert -> phone audio begann früher -> startMs abziehen
      phoneEnv.startMs = footStartMs - bestShiftMs;
    }

    totalStartMs = viewStartMs !== undefined ? viewStartMs : footStartMs;
    totalEndMs   = viewEndMs !== undefined ? viewEndMs : (footStartMs + (sessionDurationMs || footData.durationMs));
    
    if (infoEl) infoEl.textContent = (footEnv ? "Foot " : "") + (phoneEnv ? "· Phone (Auto-Matched)" : "");
    redraw();
  }

  function redraw() {
    if (!ctx || !canvas || canvas.width <= 0 || !footEnv) return;
    const W = canvas.width, H = canvas.height, span = totalEndMs - totalStartMs;
    ctx.fillStyle = '#0f1a2e'; ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = 'rgba(255,255,255,0.05)'; ctx.beginPath(); ctx.moveTo(0, H/2); ctx.lineTo(W, H/2); ctx.stroke();
    
    const drawStream = (env, color) => {
      if (!env || span <= 0) return;
      const x1 = H_PAD + ((env.startMs - totalStartMs) / span) * (W - 2 * H_PAD);
      const x2 = H_PAD + ((env.startMs + env.durationMs - totalStartMs) / span) * (W - 2 * H_PAD);
      const w = x2 - x1;
      ctx.fillStyle = color;
      const bins = env.grid.length;
      for (let i = 0; i < bins; i++) {
        const val = Math.sqrt(env.grid[i]); // Zurück zur Amplitude für die Anzeige
        const h = val * H * 0.8;
        const barX = x1 + (i / bins) * w;
        if (barX >= 0 && barX <= W) {
            ctx.fillRect(barX, (H - h) / 2, Math.max(1, w / bins), h);
        }
      }
    };
    drawStream(footEnv, BAR_COLOR_FOOT);
    drawStream(phoneEnv, BAR_COLOR_PHONE);
  }

  function setFootTime(footTsMs) {
    if (!cursorEl || !canvas || canvas.clientWidth <= 0) return;
    const span = totalEndMs - totalStartMs;
    if (span <= 0) return;
    const frac = (footTsMs - totalStartMs) / span;
    cursorEl.style.left = (H_PAD + Math.max(0, Math.min(1, frac)) * (canvas.clientWidth - 2 * H_PAD)) + 'px';
  }

  return { init, load, setFootTime, show: () => { if (panel) panel.hidden = false; redraw(); }, hide: () => { if (panel) panel.hidden = true; } };
})();
