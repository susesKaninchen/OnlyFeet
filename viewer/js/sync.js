/**
 * sync.js – Verbessertes Zeit-Alignment zwischen Handy und Fuß-Sensor
 */
const Sync = (() => {
  const FOOT_PEAK_THR_G = 1.8;       // Etwas höherer Schwellenwert für klarere Jumps
  const FOOT_PEAK_COOLDOWN_MS = 500;
  const MATCH_TOL_MS = 500;          // Toleranzfenster für das Matching
  const MIN_MATCHES = 1;

  function detectFootPeaks(packets) {
    const peaks = [];
    let lastPeakTs = -Infinity;
    
    for (let p = 0; p < packets.length; p++) {
      const pkt = packets[p];
      if (!pkt.json || !pkt.json.IMU) continue;
      const imu = pkt.json.IMU;
      const n = imu.length;
      const dt = 1000 / n;

      for (let i = 1; i < n - 1; i++) {
        const sPrev = imu[i-1].a, s = imu[i].a, sNext = imu[i+1].a;
        const mag = Math.sqrt(s.x**2 + s.y**2 + s.z**2);
        const magPrev = Math.sqrt(sPrev.x**2 + sPrev.y**2 + sPrev.z**2);
        const magNext = Math.sqrt(sNext.x**2 + sNext.y**2 + sNext.z**2);

        // Lokales Maximum finden
        if (mag > magPrev && mag > magNext && mag > FOOT_PEAK_THR_G) {
          const ts = pkt.ts + i * dt;
          if (ts - lastPeakTs > FOOT_PEAK_COOLDOWN_MS) {
            peaks.push({ ts, mag });
            lastPeakTs = ts;
          }
        }
      }
    }
    return peaks.sort((a, b) => b.mag - a.mag); // Stärkste Peaks zuerst
  }

  function scoreOffset(footPeaks, phoneJumps, offset) {
    let matches = 0;
    let residuals = [];
    const paired = [];

    for (const pj of phoneJumps) {
      const targetFoot = pj.ts - offset;
      let best = null, minDt = MATCH_TOL_MS;
      
      for (const fp of footPeaks) {
        const dt = Math.abs(fp.ts - targetFoot);
        if (dt < minDt) {
          minDt = dt;
          best = fp;
        }
      }
      
      if (best) {
        matches++;
        residuals.push(pj.ts - best.ts);
        paired.push({ foot: best.ts, phone: pj.ts });
      }
    }
    
    if (matches === 0) return { score: 0 };
    const avgOffset = residuals.reduce((a, b) => a + b, 0) / matches;
    // Score basiert auf Anzahl der Matches und wie "eng" sie beieinander liegen
    const variance = residuals.reduce((a, b) => a + Math.pow(b - avgOffset, 2), 0) / matches;
    const score = matches * 1000 - Math.sqrt(variance);
    
    return { score, matches, avgOffset, paired };
  }

  function compute(packets, phone) {
    const footPeaks = detectFootPeaks(packets);
    const phoneJumps = phone.syncJumps || [];

    console.log(`[Sync] Found ${footPeaks.length} foot peaks, ${phoneJumps.length} phone jumps`);

    if (footPeaks.length === 0 || phoneJumps.length === 0) {
      return { ok: false, reason: 'Keine Jumps zur Synchronisation gefunden' };
    }

    let bestSync = null;

    // Probiere alle Kombinationen aus, um den Anker zu finden
    for (const fp of footPeaks) {
      for (const pj of phoneJumps) {
        const candidateOffset = pj.ts - fp.ts;
        const res = scoreOffset(footPeaks, phoneJumps, candidateOffset);
        
        if (!bestSync || res.score > bestSync.score) {
          bestSync = { ...res, anchorFootTs: fp.ts, anchorPhoneTs: pj.ts };
        }
      }
    }

    if (!bestSync || bestSync.matches < MIN_MATCHES) {
      return { ok: false, reason: 'Konnte kein stabiles Zeit-Muster finden' };
    }

    const finalOffset = bestSync.avgOffset;
    console.log(`[Sync] Success! Offset: ${finalOffset}ms, Matches: ${bestSync.matches}`);

    // Gemeinsamen Zeitbereich festlegen
    const footStart = packets[0].ts;
    const footEnd = packets[packets.length - 1].ts + 1000;
    
    const phoneImuTs = phone.imu && phone.imu.length ? { start: phone.imu[0].ts, end: phone.imu[phone.imu.length - 1].ts } : null;
    const phoneStartFoot = phoneImuTs ? phoneImuTs.start - finalOffset : footStart;
    const phoneEndFoot = phoneImuTs ? phoneImuTs.end - finalOffset : footEnd;

    return {
      ok: true,
      offsetMs: finalOffset,
      matchCount: bestSync.matches,
      residualMs: 0, // In diesem Modell durch avgOffset ersetzt
      commonStartFoot: Math.max(footStart, phoneStartFoot),
      commonEndFoot: Math.min(footEnd, phoneEndFoot),
      anchorFootTs: bestSync.anchorFootTs,
      anchorPhoneTs: bestSync.anchorPhoneTs
    };
  }

  function clipPackets(packets, startFoot, endFoot) {
    return packets.filter(p => p.ts >= startFoot && p.ts <= endFoot - 1000);
  }

  return { compute, clipPackets };
})();
