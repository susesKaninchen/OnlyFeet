/**
 * imu-charts.js – Chart.js graphs for Accelerometer, Gyroscope, Magnetometer
 * Samples are pushed one-at-a-time at ~100Hz; chart redraws are batched to ~30fps.
 */
const ImuCharts = (() => {
  let accelChart, gyroChart, magChart;

  // Rolling sample buffer
  const MAX_SAMPLES = 500; // ~5s of data at 100Hz
  const accelBuf = { x: [], y: [], z: [] };
  const gyroBuf  = { x: [], y: [], z: [] };
  const magBuf   = { x: [], y: [], z: [] };

  let dirty = false;
  let rafId = null;

  const chartDefaults = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false, // we redraw at 30fps manually, no need for Chart.js animation
    plugins: {
      legend: {
        position: 'right',
        labels: { boxWidth: 8, padding: 4, font: { size: 10 }, color: '#999' }
      }
    },
    scales: {
      x: { display: false },
      y: {
        ticks: { font: { size: 10 }, color: '#666' },
        grid: { color: 'rgba(255,255,255,0.05)' },
      }
    },
    elements: {
      point: { radius: 0 },
      line: { borderWidth: 1.2 }
    }
  };

  function init() {
    accelChart = createChart('accel-chart', ['X', 'Y', 'Z'],
      ['#ff6384', '#36a2eb', '#4bc0c0']);
    gyroChart = createChart('gyro-chart', ['X', 'Y', 'Z'],
      ['#ff9f40', '#9966ff', '#ffcd56']);
    magChart = createChart('mag-chart', ['X', 'Y', 'Z'],
      ['#ff6384', '#36a2eb', '#4bc0c0']);

    // Start render loop
    renderLoop();
  }

  function createChart(canvasId, axes, colors) {
    const ctx = document.getElementById(canvasId).getContext('2d');
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: axes.map((axis, i) => ({
          label: axis,
          data: [],
          borderColor: colors[i],
          backgroundColor: 'transparent',
        }))
      },
      options: { ...chartDefaults }
    });
  }

  function pushBuf(buf, x, y, z) {
    buf.x.push(x);
    buf.y.push(y);
    buf.z.push(z);
    if (buf.x.length > MAX_SAMPLES) {
      buf.x.shift();
      buf.y.shift();
      buf.z.shift();
    }
  }

  /**
   * Push a single IMU sample (called at ~100Hz by timeline).
   * sample: { a: {x,y,z}, g: {x,y,z} }
   */
  function pushImuSample(sample) {
    if (!sample) return;
    if (sample.a) pushBuf(accelBuf, sample.a.x, sample.a.y, sample.a.z);
    if (sample.g) pushBuf(gyroBuf, sample.g.x, sample.g.y, sample.g.z);
    dirty = true;
  }

  /**
   * Push a magnetometer reading (called once per packet).
   * mag: {x, y, z}
   */
  function pushMag(mag) {
    if (!mag) return;
    pushBuf(magBuf, mag.x, mag.y, mag.z);
    dirty = true;
  }

  /** ~30fps render loop — only redraws when new data arrived */
  function renderLoop() {
    rafId = requestAnimationFrame(renderLoop);
    if (!dirty) return;
    dirty = false;

    const labels = accelBuf.x.map((_, i) => i);
    accelChart.data.labels = labels;
    accelChart.data.datasets[0].data = accelBuf.x;
    accelChart.data.datasets[1].data = accelBuf.y;
    accelChart.data.datasets[2].data = accelBuf.z;
    accelChart.update('none');

    gyroChart.data.labels = labels;
    gyroChart.data.datasets[0].data = gyroBuf.x;
    gyroChart.data.datasets[1].data = gyroBuf.y;
    gyroChart.data.datasets[2].data = gyroBuf.z;
    gyroChart.update('none');

    const magLabels = magBuf.x.map((_, i) => i);
    magChart.data.labels = magLabels;
    magChart.data.datasets[0].data = magBuf.x;
    magChart.data.datasets[1].data = magBuf.y;
    magChart.data.datasets[2].data = magBuf.z;
    magChart.update('none');
  }

  function reset() {
    for (const buf of [accelBuf, gyroBuf, magBuf]) {
      buf.x.length = 0;
      buf.y.length = 0;
      buf.z.length = 0;
    }
    dirty = true;
  }

  // Legacy bulk update — still useful for seek/manual navigation
  function update(pkt) {
    if (!pkt || !pkt.json) return;
    const json = pkt.json;
    if (json.IMU) {
      for (const s of json.IMU) pushImuSample(s);
    }
    if (json.mag) pushMag(json.mag);
  }

  return { init, update, pushImuSample, pushMag, reset };
})();
