/**
 * imu-charts.js – Graphen mit starrem Zeitraster (füllt Lücken mit 0)
 */
const ImuCharts = (() => {
  let accelChart, gyroChart, magChart;
  let sessionData = { foot: { a:[], g:[], m:[] }, phone: { a:[], g:[], m:[] } };
  let currentSampleIdx = 0;
  
  const WINDOW_SIZE = 400; // 4 Sekunden Sichtfenster
  const CENTER_IDX = WINDOW_SIZE / 2;
  let phoneOverlayOn = false;
  let dirty = false;

  const chartDefaults = {
    responsive: true, maintainAspectRatio: false, animation: false,
    plugins: { legend: { display: false } },
    scales: {
      x: { display: false, min: 0, max: WINDOW_SIZE },
      y: { ticks: { font: { size: 10 }, color: '#666' }, grid: { color: 'rgba(255,255,255,0.05)' } }
    },
    elements: { point: { radius: 0 }, line: { borderWidth: 1.1 } }
  };

  function init() {
    accelChart = createChart('accel-chart', ['X','Y','Z','PX','PY','PZ'], ['#ff6384','#36a2eb','#4bc0c0','#ffccd5','#d1e9ff','#d1f7f7']);
    gyroChart  = createChart('gyro-chart',  ['X','Y','Z','PX','PY','PZ'], ['#ff9f40','#9966ff','#ffcd56','#ffe5cc','#e5d1ff','#fff4d1']);
    magChart   = createChart('mag-chart',   ['X','Y','Z','PX','PY','PZ'], ['#ff6384','#36a2eb','#4bc0c0','#ffccd5','#d1e9ff','#d1f7f7']);
    renderLoop();
  }

  function createChart(canvasId, labels, colors) {
    const ctx = document.getElementById(canvasId).getContext('2d');
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: new Array(WINDOW_SIZE).fill(''),
        datasets: labels.map((label, i) => ({
          label, data: new Array(WINDOW_SIZE).fill(NaN),
          borderColor: colors[i], backgroundColor: 'transparent',
          borderDash: i >= 3 ? [3, 3] : [], hidden: i >= 3,
          spanGaps: true,
          borderWidth: 1.1
        }))
      },
      options: { ...chartDefaults }
    });
  }

  function enablePhoneOverlay() {
    phoneOverlayOn = true;
    [accelChart, gyroChart, magChart].forEach(c => {
      if (c && c.data && c.data.datasets.length >= 6) {
        c.data.datasets[3].hidden = false;
        c.data.datasets[4].hidden = false;
        c.data.datasets[5].hidden = false;
        c.update('none');
      }
    });
    dirty = true;
  }

  function disablePhoneOverlay() {
    phoneOverlayOn = false;
    [accelChart, gyroChart, magChart].forEach(c => {
      if (c && c.data && c.data.datasets.length >= 6) {
        c.data.datasets[3].hidden = true;
        c.data.datasets[4].hidden = true;
        c.data.datasets[5].hidden = true;
        c.update('none');
      }
    });
    dirty = true;
  }

  function setSessionData(footPackets, phoneData, offsetMs) {
    reset();
    if (!footPackets.length) return;
    
    const firstTs = footPackets[0].ts;
    sessionData.firstTs = firstTs;
    const lastTs = footPackets[footPackets.length - 1].ts + 1000;
    const totalSamples = Math.ceil((lastTs - firstTs) / 10);
    
    // Initialisiere mit NaN (verhindert Sprünge auf 0)
    for (let i = 0; i < totalSamples; i++) {
      sessionData.foot.a.push({x:NaN, y:NaN, z:NaN});
      sessionData.foot.g.push({x:NaN, y:NaN, z:NaN});
      sessionData.foot.m.push({x:NaN, y:NaN, z:NaN});
      sessionData.phone.a.push({x:NaN, y:NaN, z:NaN});
      sessionData.phone.g.push({x:NaN, y:NaN, z:NaN});
      sessionData.phone.m.push({x:NaN, y:NaN, z:NaN});
    }

    // Foot Daten an die exakten Zeit-Positionen schreiben
    footPackets.forEach(pkt => {
      if (!pkt.json || !pkt.json.IMU) return;
      const baseIdx = Math.floor((pkt.ts - firstTs) / 10);
      const m = pkt.json.mag || {x:NaN,y:NaN,z:NaN};
      pkt.json.IMU.forEach((s, i) => {
        const idx = baseIdx + i;
        if (idx >= 0 && idx < totalSamples) {
          sessionData.foot.a[idx] = s.a;
          sessionData.foot.g[idx] = s.g;
          sessionData.foot.m[idx] = m;
        }
      });
    });

    if (phoneData && phoneData.imu) {
      const imu = phoneData.imu;

      for (let i = 0; i < totalSamples; i++) {
        const targetTs = firstTs + i * 10 + offsetMs;
        const pIdx = PhoneLoader.floorIndex(imu, targetTs);
        
        let bestA = {x:NaN, y:NaN, z:NaN}, bestG = {x:NaN, y:NaN, z:NaN}, bestM = {x:NaN, y:NaN, z:NaN};
        let minDtA = 100, minDtG = 100, minDtM = 100;

        if (pIdx >= 0) {
          // Suche in der Nähe nach den besten (zeitlich nächsten) Werten
          for (let k = Math.max(0, pIdx - 20); k <= Math.min(imu.length - 1, pIdx + 20); k++) {
            const s = imu[k];
            const dt = Math.abs(s.ts - targetTs);
            if (dt < 100) {
              if (s.sensor === 'accel' && dt < minDtA) {
                bestA = { x: s.x/9.81, y: s.y/9.81, z: s.z/9.81 };
                minDtA = dt;
              }
              else if (s.sensor === 'gyro' && dt < minDtG) {
                bestG = { x: s.x*180/Math.PI, y: s.y*180/Math.PI, z: s.z*180/Math.PI };
                minDtG = dt;
              }
              else if (s.sensor === 'mag' && dt < minDtM) {
                bestM = { x: s.x, y: s.y, z: s.z };
                minDtM = dt;
              }
            }
          }
        }
        sessionData.phone.a[i] = bestA;
        sessionData.phone.g[i] = bestG;
        sessionData.phone.m[i] = bestM;
      }
    }
    dirty = true;
  }

  function renderLoop() {
    requestAnimationFrame(renderLoop);
    if (!dirty) return;
    dirty = false;
    const updateChart = (chart, footKey, phoneKey) => {
      const start = currentSampleIdx - CENTER_IDX;
      for (let i = 0; i < WINDOW_SIZE; i++) {
        const sIdx = start + i;
        const f = sessionData.foot[footKey][sIdx] || {x:NaN,y:NaN,z:NaN};
        const p = sessionData.phone[phoneKey][sIdx] || {x:NaN,y:NaN,z:NaN};
        chart.data.datasets[0].data[i] = f.x;
        chart.data.datasets[1].data[i] = f.y;
        chart.data.datasets[2].data[i] = f.z;
        if (phoneOverlayOn) {
          chart.data.datasets[3].data[i] = p.x;
          chart.data.datasets[4].data[i] = p.y;
          chart.data.datasets[5].data[i] = p.z;
        }
      }
      chart.update('none');
    };
    updateChart(accelChart, 'a', 'a');
    updateChart(gyroChart,  'g', 'g');
    updateChart(magChart,   'm', 'm');
  }

  function reset() {
    sessionData = { foot: { a:[], g:[], m:[] }, phone: { a:[], g:[], m:[] } };
    currentSampleIdx = 0; dirty = true;
  }

  function setFootTime(footTsMs) {
    if (sessionData.firstTs === undefined) return;
    currentSampleIdx = Math.floor((footTsMs - sessionData.firstTs) / 10);
    dirty = true;
  }

  return { init, setSessionData, setFootTime, 
           enablePhoneOverlay: () => { phoneOverlayOn = true; dirty = true; }, 
           disablePhoneOverlay: () => { phoneOverlayOn = false; dirty = true; }, reset };
})();
