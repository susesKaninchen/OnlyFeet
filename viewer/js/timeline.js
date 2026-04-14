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
    currentIndex = 0;
    playing = false;

    slider.max = Math.max(0, packets.length - 1);
    slider.value = 0;
    updateDisplay();
    emitPacket();
  }

  /** Tatsächliche Paket-Dauer in ms aus Timestamp-Delta (gemeinsame Quelle für alle Animationen) */
  function getDurationMs() {
    if (currentIndex < packets.length - 1) {
      const dt = packets[currentIndex + 1].ts - packets[currentIndex].ts;
      if (dt > 0 && dt < 5000) return dt;
    }
    return 1000;
  }

  function emitPacket() {
    if (!packets.length) return;
    const pkt = packets[currentIndex];
    slider.value = currentIndex;
    updateDisplay();
    onPacketChange(pkt, currentIndex);

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

    if (pkt.json.tof && pkt.json.tof.length) {
      onTofFrame(pkt.json.tof[0], 0);
    }
    if (pkt.json.IMU && pkt.json.IMU.length) {
      const mag = pkt.json.mag || null;
      // Push all samples at once so charts/orientation reflect this packet
      for (let i = 0; i < pkt.json.IMU.length; i++) {
        onImuSample(pkt.json.IMU[i], i === 0 ? mag : null, i);
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
    if (!pkt.json || !pkt.json.IMU || !pkt.json.IMU.length) return;

    const samples = pkt.json.IMU;
    const mag = pkt.json.mag || null;
    currentImuSample = 0;
    onImuSample(samples[0], mag, 0);

    if (samples.length <= 1) return;

    // Interval aus tatsächlicher Paket-Dauer ableiten statt 10ms hardcoded.
    // Dadurch läuft die IMU-Animation genau so lang wie das Paket.
    const intervalMs = (durationMs / speed) / samples.length;
    imuTimerId = setInterval(() => {
      currentImuSample++;
      if (currentImuSample < samples.length) {
        onImuSample(samples[currentImuSample], null, currentImuSample);
      } else {
        stopImuAnimation();
      }
    }, intervalMs);
  }

  function stopImuAnimation() {
    if (imuTimerId !== null) {
      clearInterval(imuTimerId);
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
        emitPacket();
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
    emitPacket();
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
    if (playing) {
      pause();
      play();
    }
  });

  return { init, play, pause, seekTo, getCurrentIndex: () => currentIndex };
})();
