/**
 * audio-player.js – Plays a single concatenated session WAV.
 *
 * The whole session is pre-assembled into one WAV by data-loader. This player
 * just loads that URL once and controls playback via play/pause/seek. Avoids
 * the per-1-s-clip src-swap glitches (noise between packets) and the initial
 * auto-play on load (now silent until the user presses play).
 */
const AudioPlayer = (() => {
  let audioEl = null;
  let volume = 0.8;
  let sessionDurationSec = 0;
  let packetDurationSec = 1.0;
  let firstPacketIndex = 0;

  function init() {
    if (audioEl) return;
    audioEl = new Audio();
    audioEl.preload = 'auto';
    audioEl.volume = volume;
  }

  /**
   * Load a pre-built session WAV. `opts` = { firstIndex, packetDurationSec }
   * so seekToPacket(idx) can compute the right time offset.
   */
  function loadSession(url, opts = {}) {
    init();
    firstPacketIndex   = opts.firstIndex ?? 0;
    packetDurationSec  = opts.packetDurationSec ?? 1.0;
    sessionDurationSec = 0;

    audioEl.pause();
    audioEl.src = url || '';
    audioEl.currentTime = 0;

    if (!url) return;
    audioEl.addEventListener('loadedmetadata', () => {
      sessionDurationSec = audioEl.duration || 0;
    }, { once: true });
  }

  function play() {
    if (!audioEl || !audioEl.src) return;
    const p = audioEl.play();
    if (p && typeof p.catch === 'function') {
      p.catch(err => console.warn('[AudioPlayer] play blocked:', err.message));
    }
  }

  function pause() {
    if (audioEl) audioEl.pause();
  }

  /** Jump to the audio position corresponding to a given packet index. */
  function seekToPacket(idx) {
    if (!audioEl) return;
    const t = (idx - firstPacketIndex) * packetDurationSec;
    if (!isFinite(t) || t < 0) return;
    try { audioEl.currentTime = t; } catch (_) { /* ignored — seek before metadata */ }
  }

  function setVolume(v) {
    volume = v;
    if (audioEl) audioEl.volume = v;
  }

  function setPlaybackRate(r) {
    if (audioEl && isFinite(r) && r > 0) audioEl.playbackRate = r;
  }

  return { init, loadSession, play, pause, seekToPacket, setVolume, setPlaybackRate };
})();
