/**
 * data-loader.js – Parse folder files, build packet index
 */
const DataLoader = (() => {

  /**
   * Load all files from a webkitdirectory input.
   * Returns { folderName, packets: [{index, ts, json, imgUrl, midImgUrl, wavUrl}, ...] }
   */
  async function load(fileList) {
    const files = Array.from(fileList);
    if (!files.length) return null;

    // Determine folder name from first file path
    const firstPath = files[0].webkitRelativePath || files[0].name;
    const folderName = firstPath.split('/')[0];

    // Group files by type and packet index
    const pktJsons    = {};  // index → File
    const pktImgs     = {};  // index → File  (end-of-window: img_N_ts.jpg)
    const pktMidImgs  = {};  // index → File  (mid-window:    mid_N_ts.jpg)
    const pktWavs     = {};  // index → File

    const reJson   = /pkt_(\d+)\.json$/;
    const reImg    = /img_(\d+)_(\d+)\.jpg$/;
    const reMidImg = /mid_(\d+)_(\d+)\.jpg$/;
    const reWav    = /rec_(\d+)_(\d+)\.wav$/;

    for (const file of files) {
      const name = file.name;
      let m;
      if ((m = reJson.exec(name))) {
        pktJsons[parseInt(m[1])] = file;
      } else if ((m = reMidImg.exec(name))) {
        pktMidImgs[parseInt(m[1])] = file;   // mid_ vor img_ prüfen
      } else if ((m = reImg.exec(name))) {
        pktImgs[parseInt(m[1])] = file;
      } else if ((m = reWav.exec(name))) {
        pktWavs[parseInt(m[1])] = file;
      }
    }

    // Collect all packet indices
    const indices = new Set([
      ...Object.keys(pktJsons).map(Number),
      ...Object.keys(pktImgs).map(Number),
      ...Object.keys(pktMidImgs).map(Number),
      ...Object.keys(pktWavs).map(Number),
    ]);
    const sortedIndices = [...indices].sort((a, b) => a - b);

    // Parse JSON files and build packets
    const packets = [];
    for (const idx of sortedIndices) {
      const pkt = { index: idx, ts: 0, json: null, imgUrl: null, midImgUrl: null, wavUrl: null, wavFile: null, midTs: null, endTs: null };

      if (pktJsons[idx]) {
        const text = await pktJsons[idx].text();
        pkt.json = JSON.parse(text);
        pkt.ts    = pkt.json.ts;
        pkt.midTs = pkt.json.midTs ?? null;
        pkt.endTs = pkt.json.endTs ?? null;
      }

      if (pktImgs[idx]) {
        pkt.imgUrl = URL.createObjectURL(pktImgs[idx]);
      }

      if (pktMidImgs[idx]) {
        pkt.midImgUrl = URL.createObjectURL(pktMidImgs[idx]);
      }

      if (pktWavs[idx]) {
        pkt.wavUrl = URL.createObjectURL(pktWavs[idx]);
        pkt.wavFile = pktWavs[idx];
      }

      packets.push(pkt);
    }

    // Build a single concatenated session WAV so audio playback can run as one
    // continuous stream (avoids the load/glitch when swapping src per 1-s clip).
    // Missing packets are filled with 1 s of silence so the time axis stays exact:
    // seek(audio, (pkt.index - firstIdx) * 1.0) always lands on the right packet.
    const sessionWavUrl = await buildSessionWav(packets);

    return { folderName, packets, sessionWavUrl };
  }

  async function buildSessionWav(packets) {
    if (!packets.length) return null;
    let sampleRate = 16000, channels = 1, bitsPerSample = 16;
    
    for (const p of packets) {
      if (p.wavFile) {
        const buf = await p.wavFile.arrayBuffer();
        const dv = new DataView(buf);
        channels      = dv.getUint16(22, true);
        sampleRate    = dv.getUint32(24, true);
        bitsPerSample = dv.getUint16(34, true);
        break;
      }
    }
    const bytesPerSec = sampleRate * channels * (bitsPerSample / 8);

    const firstTs = packets[0].ts;
    const lastPkt = packets[packets.length - 1];
    
    // Wir berechnen die genaue Dauer basierend auf den Timestamps
    let maxTsMs = lastPkt.ts - firstTs + 1000;
    
    // Ermittle die tatsächlichen Größen der WAVs
    const pcmBlocks = [];
    for (const p of packets) {
      if (p.wavFile) {
        const buf = await p.wavFile.arrayBuffer();
        const dv = new DataView(buf);
        const dataSize = dv.getUint32(40, true);
        const dataBytes = new Uint8Array(buf, 44, dataSize);
        pcmBlocks.push({ ts: p.ts, data: dataBytes });
        const endTs = (p.ts - firstTs) + (dataSize / bytesPerSec) * 1000;
        if (endTs > maxTsMs) maxTsMs = endTs;
      }
    }

    const totalBytes = Math.ceil((maxTsMs / 1000) * bytesPerSec);
    // Align to sample block size (e.g. 2 bytes for 16-bit mono)
    const blockAlign = channels * (bitsPerSample / 8);
    const alignedTotalBytes = totalBytes + (blockAlign - (totalBytes % blockAlign)) % blockAlign;

    const headerSize = 44;
    const out = new ArrayBuffer(headerSize + alignedTotalBytes);
    const u8 = new Uint8Array(out);
    const dv = new DataView(out);
    const setStr = (off, s) => { for (let i = 0; i < s.length; i++) dv.setUint8(off + i, s.charCodeAt(i)); };
    
    setStr(0, 'RIFF');
    dv.setUint32(4, headerSize - 8 + alignedTotalBytes, true);
    setStr(8, 'WAVE');
    setStr(12, 'fmt ');
    dv.setUint32(16, 16, true);
    dv.setUint16(20, 1, true);
    dv.setUint16(22, channels, true);
    dv.setUint32(24, sampleRate, true);
    dv.setUint32(28, bytesPerSec, true);
    dv.setUint16(32, blockAlign, true);
    dv.setUint16(34, bitsPerSample, true);
    setStr(36, 'data');
    dv.setUint32(40, alignedTotalBytes, true);

    // Fülle mit Stille (0)
    u8.fill(0, headerSize);

    // Platziere die WAV-Blöcke exakt an ihrem Timestamp-Offset
    for (const b of pcmBlocks) {
      const offsetMs = b.ts - firstTs;
      const offsetBytes = Math.floor((offsetMs / 1000) * bytesPerSec);
      const alignedOffset = headerSize + (offsetBytes - (offsetBytes % blockAlign));
      
      // Kopiere die Daten, passe auf dass wir nicht über den Puffer hinausschreiben
      const copyLen = Math.min(b.data.length, u8.length - alignedOffset);
      if (copyLen > 0 && alignedOffset >= headerSize) {
         u8.set(b.data.subarray(0, copyLen), alignedOffset);
      }
    }

    return URL.createObjectURL(new Blob([out], { type: 'audio/wav' }));
  }

  return { load };
})();
