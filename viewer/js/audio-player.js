/**
 * audio-player.js – Web Audio API WAV playback
 */
const AudioPlayer = (() => {
  let audioCtx = null;
  let currentSource = null;
  let gainNode = null;
  let volume = 0.8;

  function init() {}

  function ensureContext() {
    if (!audioCtx) {
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) { console.warn('Web Audio API not supported'); return; }
      audioCtx = new AC();
      gainNode = audioCtx.createGain();
      gainNode.gain.value = volume;
      gainNode.connect(audioCtx.destination);
    }
    if (audioCtx.state === 'suspended') {
      audioCtx.resume();
    }
  }

  async function play(wavUrl) {
    if (!wavUrl) return;
    ensureContext();
    if (!audioCtx) return;
    stop();

    try {
      const response = await fetch(wavUrl);
      const arrayBuffer = await response.arrayBuffer();
      const audioBuffer = await audioCtx.decodeAudioData(arrayBuffer);

      currentSource = audioCtx.createBufferSource();
      currentSource.buffer = audioBuffer;
      currentSource.connect(gainNode);
      currentSource.start(0);
      currentSource.onended = () => { currentSource = null; };
    } catch (e) {
      console.warn('Audio playback error:', e);
    }
  }

  function stop() {
    if (currentSource) {
      try { currentSource.stop(); } catch (_) {}
      currentSource = null;
    }
  }

  function setVolume(v) {
    volume = v;
    if (gainNode) gainNode.gain.value = v;
  }

  return { init, play, stop, setVolume };
})();
