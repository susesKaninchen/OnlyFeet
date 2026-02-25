/**
 * data-loader.js – Parse folder files, build packet index
 */
const DataLoader = (() => {

  /**
   * Load all files from a webkitdirectory input.
   * Returns { folderName, packets: [{index, ts, json, imgUrl, wavUrl}, ...] }
   */
  async function load(fileList) {
    const files = Array.from(fileList);
    if (!files.length) return null;

    // Determine folder name from first file path
    const firstPath = files[0].webkitRelativePath || files[0].name;
    const folderName = firstPath.split('/')[0];

    // Group files by type and packet index
    const pktJsons = {};  // index → File
    const pktImgs = {};   // index → File
    const pktWavs = {};   // index → File

    const reJson = /pkt_(\d+)\.json$/;
    const reImg = /img_(\d+)_(\d+)\.jpg$/;
    const reWav = /rec_(\d+)_(\d+)\.wav$/;

    for (const file of files) {
      const name = file.name;
      let m;
      if ((m = reJson.exec(name))) {
        pktJsons[parseInt(m[1])] = file;
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
      ...Object.keys(pktWavs).map(Number),
    ]);
    const sortedIndices = [...indices].sort((a, b) => a - b);

    // Parse JSON files and build packets
    const packets = [];
    for (const idx of sortedIndices) {
      const pkt = { index: idx, ts: 0, json: null, imgUrl: null, wavUrl: null };

      if (pktJsons[idx]) {
        const text = await pktJsons[idx].text();
        pkt.json = JSON.parse(text);
        pkt.ts = pkt.json.ts;
      }

      if (pktImgs[idx]) {
        pkt.imgUrl = URL.createObjectURL(pktImgs[idx]);
      }

      if (pktWavs[idx]) {
        pkt.wavUrl = URL.createObjectURL(pktWavs[idx]);
      }

      packets.push(pkt);
    }

    return { folderName, packets };
  }

  return { load };
})();
