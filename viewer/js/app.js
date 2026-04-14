/**
 * app.js – Main entry: folder loading, orchestration
 */
(() => {
  const folderInput  = document.getElementById('folder-input');
  const sessionInfo  = document.getElementById('session-info');
  const cameraImg    = document.getElementById('camera-img');
  const cameraLabel  = document.getElementById('camera-label');
  const volumeSlider = document.getElementById('volume-slider');

  // Track the currently displayed packet so onTofFrame can access it
  let currentPkt = null;

  /** Show the mid-window photo (t≈500ms) and update label */
  function showMidPhoto(pkt) {
    const url = pkt.midImgUrl || pkt.imgUrl || null;
    if (url) cameraImg.src = url;
    else cameraImg.removeAttribute('src');
    cameraLabel.textContent = pkt.midImgUrl ? 'mid  ≈500 ms' : 'end  ≈1000 ms+';
  }

  /** Show the end-of-window photo (t≈1000ms+) and update label */
  function showEndPhoto(pkt) {
    const url = pkt.imgUrl || pkt.midImgUrl || null;
    if (url) cameraImg.src = url;
    else cameraImg.removeAttribute('src');
    cameraLabel.textContent = pkt.imgUrl ? 'end  ≈1000 ms+' : 'mid  ≈500 ms';
  }

  // Initialize components
  TofView.init();
  ImuCharts.init();
  ImuOrientation.init();
  AudioPlayer.init();

  // Volume control
  volumeSlider.addEventListener('input', () => {
    AudioPlayer.setVolume(parseFloat(volumeSlider.value));
  });

  // Folder selection
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

      sessionInfo.textContent = `Session: ${data.folderName}  (${data.packets.length} packets)`;
      ImuCharts.reset();
      ImuOrientation.reset();

      // Pre-compute 3D path using ZUPT-aided INS
      ImuOrientation.precomputePath(data.packets);

      // Wire up timeline
      Timeline.init(data.packets, {
        onPacketChange: (pkt, idx) => {
          currentPkt = pkt;
          // Bei Paketwechsel immer mit dem mid-Foto starten (t≈0..500 ms)
          showMidPhoto(pkt);
          // Audio
          AudioPlayer.play(pkt.wavUrl);
        },
        onImuSample: (sample, mag, sampleIdx) => {
          ImuCharts.pushImuSample(sample);
          if (mag) ImuCharts.pushMag(mag);
          ImuOrientation.updateSample(sample, mag);
        },
        onTofFrame: (frame, frameIdx) => {
          TofView.updateFrame(frame);
          // Foto synchron zum ToF-Zeitstempel wechseln:
          // frame.t < 500ms  → mid-Foto (erste Fensterhälfte)
          // frame.t >= 500ms → end-Foto  (zweite Fensterhälfte)
          if (currentPkt) {
            const tMs = (frame && frame.t != null) ? frame.t : 0;
            if (tMs < 500) showMidPhoto(currentPkt);
            else           showEndPhoto(currentPkt);
          }
        }
      });
    } catch (err) {
      console.error('Error loading data:', err);
      sessionInfo.textContent = 'Error loading data: ' + err.message;
    }
  });

  // Handle window resize for ToF + IMU orientation
  window.addEventListener('resize', () => {
    TofView.onResize();
    ImuOrientation.onResize();
  });
})();
