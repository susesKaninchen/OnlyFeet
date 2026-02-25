/**
 * Madgwick AHRS filter — browser-global version.
 * Based on MadgwickAHRS.c by Sebastian Madgwick.
 * Source: https://github.com/psiphi75/ahrs (MIT License)
 *
 * Usage:
 *   const filter = new MadgwickAHRS({ sampleInterval: 10, beta: 0.4 });
 *   filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
 *   const q = filter.getQuaternion(); // { w, x, y, z }
 */
window.MadgwickAHRS = function MadgwickAHRS(options) {
  options = options || {};
  const sampleFreq = 1000 / (options.sampleInterval || 10);
  const beta = options.beta || 0.4;

  let q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
  let recipSampleFreq = 1.0 / sampleFreq;
  let initialised = false;

  function cross(ax, ay, az, bx, by, bz) {
    return { x: ay * bz - az * by, y: az * bx - ax * bz, z: ax * by - ay * bx };
  }

  function init(ax, ay, az, mx, my, mz) {
    const pitch = -Math.atan2(ax, Math.sqrt(ay * ay + az * az));
    const tmp1 = cross(ax, ay, az, 1, 0, 0);
    const tmp2 = cross(1, 0, 0, tmp1.x, tmp1.y, tmp1.z);
    const roll = Math.atan2(tmp2.y, tmp2.z);
    const cr = Math.cos(roll), sp = Math.sin(pitch), sr = Math.sin(roll);
    const yh = my * cr - mz * sr;
    const xh = mx * Math.cos(pitch) + my * sr * sp + mz * cr * sp;
    const heading = -Math.atan2(yh, xh);

    const cy = Math.cos(heading * 0.5), sy = Math.sin(heading * 0.5);
    const cp = Math.cos(pitch * 0.5), spp = Math.sin(pitch * 0.5);
    const crl = Math.cos(roll * 0.5), srl = Math.sin(roll * 0.5);
    const iq = {
      w: crl * cp * cy + srl * spp * sy,
      x: srl * cp * cy - crl * spp * sy,
      y: crl * spp * cy + srl * cp * sy,
      z: crl * cp * sy - srl * spp * cy
    };
    const rn = (iq.w * iq.w + iq.x * iq.x + iq.y * iq.y + iq.z * iq.z) ** -0.5;
    q0 = iq.w * rn; q1 = iq.x * rn; q2 = iq.y * rn; q3 = iq.z * rn;
    initialised = true;
  }

  function updateIMU(gx, gy, gz, ax, ay, az) {
    let recipNorm, s0, s1, s2, s3;
    let qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    let qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    let qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    let qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    if (!(ax === 0 && ay === 0 && az === 0)) {
      recipNorm = (ax * ax + ay * ay + az * az) ** -0.5;
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      const v2q0 = 2 * q0, v2q1 = 2 * q1, v2q2 = 2 * q2, v2q3 = 2 * q3;
      const v4q0 = 4 * q0, v4q1 = 4 * q1, v4q2 = 4 * q2, v8q1 = 8 * q1, v8q2 = 8 * q2;
      const q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
      s0 = v4q0 * q2q2 + v2q2 * ax + v4q0 * q1q1 - v2q1 * ay;
      s1 = v4q1 * q3q3 - v2q3 * ax + 4 * q0q0 * q1 - v2q0 * ay - v4q1 + v8q1 * q1q1 + v8q1 * q2q2 + v4q1 * az;
      s2 = 4 * q0q0 * q2 + v2q0 * ax + v4q2 * q3q3 - v2q3 * ay - v4q2 + v8q2 * q1q1 + v8q2 * q2q2 + v4q2 * az;
      s3 = 4 * q1q1 * q3 - v2q1 * ax + 4 * q2q2 * q3 - v2q2 * ay;
      recipNorm = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) ** -0.5;
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
      qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }

    q0 += qDot1 * recipSampleFreq; q1 += qDot2 * recipSampleFreq;
    q2 += qDot3 * recipSampleFreq; q3 += qDot4 * recipSampleFreq;
    recipNorm = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) ** -0.5;
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  }

  function update(gx, gy, gz, ax, ay, az, mx, my, mz, deltaTimeSec) {
    if (deltaTimeSec) recipSampleFreq = deltaTimeSec;
    if (!initialised && mx !== undefined && mx !== null) { init(ax, ay, az, mx, my, mz); }

    if (mx === undefined || mx === null || (mx === 0 && my === 0 && mz === 0)) {
      updateIMU(gx, gy, gz, ax, ay, az);
      return;
    }

    let recipNorm, s0, s1, s2, s3;
    let qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    let qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    let qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    let qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    if (!(ax === 0 && ay === 0 && az === 0)) {
      recipNorm = (ax * ax + ay * ay + az * az) ** -0.5;
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      recipNorm = (mx * mx + my * my + mz * mz) ** -0.5;
      mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

      const v2q0mx = 2 * q0 * mx, v2q0my = 2 * q0 * my, v2q0mz = 2 * q0 * mz, v2q1mx = 2 * q1 * mx;
      const v2q0 = 2 * q0, v2q1 = 2 * q1, v2q2 = 2 * q2, v2q3 = 2 * q3;
      const v2q0q2 = 2 * q0 * q2, v2q2q3 = 2 * q2 * q3;
      const q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
      const q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
      const q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

      const hx = mx * q0q0 - v2q0my * q3 + v2q0mz * q2 + mx * q1q1 + v2q1 * my * q2 + v2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      const hy = v2q0mx * q3 + my * q0q0 - v2q0mz * q1 + v2q1mx * q2 - my * q1q1 + my * q2q2 + v2q2 * mz * q3 - my * q3q3;
      const v2bx = Math.sqrt(hx * hx + hy * hy);
      const v2bz = -v2q0mx * q2 + v2q0my * q1 + mz * q0q0 + v2q1mx * q3 - mz * q1q1 + v2q2 * my * q3 - mz * q2q2 + mz * q3q3;
      const v4bx = 2 * v2bx, v4bz = 2 * v2bz;

      s0 = -v2q2 * (2 * q1q3 - v2q0q2 - ax) + v2q1 * (2 * q0q1 + v2q2q3 - ay) - v2bz * q2 * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) + (-v2bx * q3 + v2bz * q1) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) + v2bx * q2 * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz);
      s1 = v2q3 * (2 * q1q3 - v2q0q2 - ax) + v2q0 * (2 * q0q1 + v2q2q3 - ay) - 4 * q1 * (1 - 2 * q1q1 - 2 * q2q2 - az) + v2bz * q3 * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) + (v2bx * q2 + v2bz * q0) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) + (v2bx * q3 - v4bz * q1) * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz);
      s2 = -v2q0 * (2 * q1q3 - v2q0q2 - ax) + v2q3 * (2 * q0q1 + v2q2q3 - ay) - 4 * q2 * (1 - 2 * q1q1 - 2 * q2q2 - az) + (-v4bx * q2 - v2bz * q0) * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) + (v2bx * q1 + v2bz * q3) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) + (v2bx * q0 - v4bz * q2) * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz);
      s3 = v2q1 * (2 * q1q3 - v2q0q2 - ax) + v2q2 * (2 * q0q1 + v2q2q3 - ay) + (-v4bx * q3 + v2bz * q1) * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) + (-v2bx * q0 + v2bz * q2) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) + v2bx * q1 * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz);
      recipNorm = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) ** -0.5;
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
      qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }

    q0 += qDot1 * recipSampleFreq; q1 += qDot2 * recipSampleFreq;
    q2 += qDot3 * recipSampleFreq; q3 += qDot4 * recipSampleFreq;
    recipNorm = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) ** -0.5;
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  }

  return {
    update: update,
    init: init,
    getQuaternion: function () { return { w: q0, x: q1, y: q2, z: q3 }; }
  };
};
