/**
 * timeline.js – Play/pause, timeline slider, sync clock
 */
const Timeline = (() => {
  let packets = [];
  let currentIndex = 0;
  let playing = false;
  let speed = 1;
  let timerId = null;
  let tofTimers = [];          // setTimeout IDs für ToF-Frames
  let currentTofFrame = 0;
  let imuTimerId = null;
  let currentImuSample = 0;

  // Callbacks
  let onPacketChange = null;
  let onTofFrame = null;
  let onImuSample = null;
  let onTimeUpdate = null;
  let onPlay = null;
  let onPause = null;
  let onSpeedChange = null;

  // DOM
  const slider = document.getElementById('timeline-slider');
  const timeDisplay = document.getElementById('time-display');
  const btnPlay = document.getElementById('btn-play');
  const btnPrev = document.getElementById('btn-prev');
  const btnNext = document.getElementById('btn-next');
  const speedSelect = document.getElementById('speed-select');

  function init(pkts, callbacks) {
    packets = pkts;
    onPacketChange = callbacks.onPacketChange || (() => {});
    onTofFrame = callbacks.onTofFrame || (() => {});
    onImuSample = callbacks.onImuSample || (() => {});
    onTimeUpdate = callbacks.onTimeUpdate || (() => {});
    onPlay  = callbacks.onPlay  || (() => {});
    onPause = callbacks.onPause || (() => {});
    onSpeedChange = callbacks.onSpeedChange || (() => {});
    currentIndex = 0;
    playing = false;

    slider.max = Math.max(0, packets.length - 1);
    slider.value = 0;
    updateDisplay();
    emitPacket('seek');
  }

  /** Tatsächliche Paket-Dauer in ms aus Index-Delta (passend zum Audio-Padding) */
  function getDurationMs() {
    if (currentIndex < packets.length - 1) {
      const idxDiff = packets[currentIndex + 1].index - packets[currentIndex].index;
      return idxDiff * 1000;
    }
    return 1000;
  }

  function emitPacket(reason) {
    if (!packets.length) return;
    const pkt = packets[currentIndex];
    slider.value = currentIndex;
    updateDisplay();
    onPacketChange(pkt, currentIndex, { reason: reason || 'advance' });

    // When paused: show first frame/sample only, no animation
    if (playing) {
      const dur = getDurationMs();
      startTofAnimation(pkt);          // verwendet frame.t (eigene Timestamps)
      startImuAnimation(pkt, dur);     // skaliert Interval auf echte Paket-Dauer
    } else {
      showStaticFrame(pkt);
    }
  }

  /** Show first ToF frame + all IMU samples instantly (for pause/seek) */
  function showStaticFrame(pkt) {
    stopTofAnimation();
    stopImuAnimation();
    if (!pkt.json) return;

    onTimeUpdate(pkt.ts);

    if (pkt.json.tof && pkt.json.tof.length) {
      onTofFrame(pkt.json.tof[0], 0);
    }
    if (pkt.json.IMU && pkt.json.IMU.length) {
      const mag = pkt.json.mag || null;
      const dt = 1.0 / pkt.json.IMU.length;
      
      // Nutze die ZEIT seit Start statt Paketsummen für den Index
      const absoluteBaseIdx = Math.floor((pkt.ts - packets[0].ts) / 10);
      
      // Push all samples at once so charts/orientation reflect this packet
      for (let i = 0; i < pkt.json.IMU.length; i++) {
        onImuSample(pkt.json.IMU[i], i === 0 ? mag : null, i, dt, absoluteBaseIdx + i);
      }
    }
  }

  function startTofAnimation(pkt) {
    stopTofAnimation();
    if (!pkt.json || !pkt.json.tof || !pkt.json.tof.length) return;

    const frames = pkt.json.tof;
    currentTofFrame = 0;
    onTofFrame(frames[0], 0);

    if (frames.length <= 1) return;

    // Jedes Frame zum tatsächlichen t-Offset anzeigen (speed-skaliert).
    // frame.t ist der ms-Offset ab Paket-Start laut Firmware-Timestamp.
    const t0 = frames[0].t;
    for (let i = 1; i < frames.length; i++) {
      const delayMs = Math.max(0, (frames[i].t - t0) / speed);
      const id = setTimeout(() => {
        currentTofFrame = i;
        onTofFrame(frames[i], i);
      }, delayMs);
      tofTimers.push(id);
    }
  }

  function startImuAnimation(pkt, durationMs) {
    stopImuAnimation();
    
    // Wenn das Paket keine IMU Daten hat, trotzdem die Zeitachse weiterbewegen
    const samples = (pkt.json && pkt.json.IMU) ? pkt.json.IMU : [];
    const mag = (pkt.json && pkt.json.mag) ? pkt.json.mag : null;
    const n = samples.length;
    const dataDurationMs = n > 0 ? 1000 : 0; // Daten sind max 1 Sekunde lang, der Rest ist Lücke
    const dtReal = n > 0 ? (1.0 / n) : 0;
    
    const absoluteBaseIdx = Math.floor((pkt.ts - packets[0].ts) / 10);

    let startTime = performance.now();
    currentImuSample = -1;

    function anim(now) {
      if (!playing) return;
      const elapsed = (now - startTime) * speed;
      
      onTimeUpdate(pkt.ts + elapsed);

      if (n > 0) {
          const sampleIdx = Math.floor(elapsed / (dataDurationMs / n));
          if (sampleIdx > currentImuSample && currentImuSample < n - 1) {
            for (let i = currentImuSample + 1; i <= Math.min(sampleIdx, n - 1); i++) {
              onImuSample(samples[i], i === 0 ? mag : null, i, dtReal, absoluteBaseIdx + i);
            }
            currentImuSample = sampleIdx;
          }
      }
      
      if (elapsed < durationMs) {
        imuTimerId = requestAnimationFrame(anim);
      } else if (currentImuSample < n - 1 && n > 0) {
        // Reste flushen
        for (let i = currentImuSample + 1; i <= n - 1; i++) {
          onImuSample(samples[i], i === 0 ? mag : null, i, dtReal, absoluteBaseIdx + i);
        }
      }
    }
    imuTimerId = requestAnimationFrame(anim);
  }


  function stopImuAnimation() {
    if (imuTimerId !== null) {
      cancelAnimationFrame(imuTimerId);
      imuTimerId = null;
    }
  }

  function stopTofAnimation() {
    tofTimers.forEach(id => clearTimeout(id));
    tofTimers = [];
  }

  function play() {
    if (!packets.length) return;
    playing = true;
    btnPlay.textContent = '⏸ Pause';
    onPlay();
    scheduleNext();
  }

  function pause() {
    playing = false;
    btnPlay.textContent = '▶ Play';
    if (timerId !== null) {
      clearTimeout(timerId);
      timerId = null;
    }
    stopTofAnimation();
    stopImuAnimation();
    onPause();
  }

  function togglePlay() {
    if (playing) pause();
    else play();
  }

  function scheduleNext() {
    if (!playing) return;
    timerId = setTimeout(() => {
      if (!playing) return;
      if (currentIndex < packets.length - 1) {
        currentIndex++;
        emitPacket('advance');
        scheduleNext();
      } else {
        pause(); // end of session
      }
    }, getDurationMs() / speed);
  }

  function seekTo(idx) {
    const wasPlaying = playing;
    if (playing) pause();
    currentIndex = Math.max(0, Math.min(idx, packets.length - 1));
    emitPacket('seek');
    if (wasPlaying) play();
  }

  function updateDisplay() {
    const cur = currentIndex;
    const total = Math.max(0, packets.length - 1);
    const curSec = packets.length ? Math.round((packets[cur].ts - packets[0].ts) / 1000) : 0;
    const totalSec = packets.length > 1 ? Math.round((packets[total].ts - packets[0].ts) / 1000) : 0;
    timeDisplay.textContent = `${fmtTime(curSec)} / ${fmtTime(totalSec)}`;
  }

  function fmtTime(s) {
    const m = Math.floor(s / 60);
    const sec = s % 60;
    return `${m}:${String(sec).padStart(2, '0')}`;
  }

  // Event listeners
  slider.addEventListener('input', () => seekTo(parseInt(slider.value)));
  btnPlay.addEventListener('click', togglePlay);
  btnPrev.addEventListener('click', () => { if (currentIndex > 0) seekTo(currentIndex - 1); });
  btnNext.addEventListener('click', () => { if (currentIndex < packets.length - 1) seekTo(currentIndex + 1); });
  speedSelect.addEventListener('change', () => {
    speed = parseFloat(speedSelect.value);
    onSpeedChange(speed);
    if (playing) {
      pause();
      play();
    }
  });

  return { init, play, pause, seekTo, getCurrentIndex: () => currentIndex };
})();
