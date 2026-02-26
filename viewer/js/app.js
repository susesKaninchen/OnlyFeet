/**
 * app.js – Main entry: folder loading, orchestration
 */
(() => {
  const folderInput = document.getElementById('folder-input');
  const sessionInfo = document.getElementById('session-info');
  const cameraImg = document.getElementById('camera-img');
  const volumeSlider = document.getElementById('volume-slider');

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
          // Camera image
          if (pkt.imgUrl) {
            cameraImg.src = pkt.imgUrl;
          } else {
            cameraImg.removeAttribute('src');
          }

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
