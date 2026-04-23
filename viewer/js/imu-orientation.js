/**
 * imu-orientation.js – 3D board visualization from IMU data
 *
 * Rotation:    Madgwick AHRS filter fusing accel + gyro + mag → quaternion
 * Translation: Pre-computed INS with periodic drift correction.
 *
 * Approach: AHRS gives orientation → quaternion-based gravity removal →
 * rotate linear accel to world frame → double-integrate per segment →
 * retroactive velocity drift correction every ~1 second.
 *
 * Works for any motion: walking, waving, rotating, whatever.
 * Some drift is expected and acceptable.
 */
const ImuOrientation = (() => {
  const container = document.getElementById('imu-orient-container');
  let scene, camera, renderer, controls;
  let boardGroup;

  // --- AHRS filter (live, for smooth rotation during playback) ---
  let ahrs = null;
  let lastMag = null;

  // --- Display smoothing ---
  const displayQuat = new THREE.Quaternion();
  let hasTarget = false;
  const SLERP_ALPHA = 0.3;

  // --- Pre-computed path ---
  let precomputedPath = null;
  let sampleIndex = 0;

  // --- Position display ---
  const SCALE = 2.0;
  const currentPos = new THREE.Vector3();
  const displayPos = new THREE.Vector3();

  // --- Trail line ---
  const MAX_TRAIL_POINTS = 10000;
  let trailPositions;
  let trailLine;
  let trailGeom;
  let trailCount = 0;
  let trailAddCounter = 0;
  const TRAIL_INTERVAL = 3;

  // --- Origin marker ---
  let originMarker;

  // --- INS constants ---
  const DT = 0.01;
  const G_TO_MS2 = 9.81;
  const DEAD_ZONE = 0.02;           // small accel threshold to suppress pure noise
  // ZUPT (Zero-velocity UPdaTe) thresholds for foot-mounted IMU stance detection.
  // Stance = foot on ground = ω ≈ 0 AND |a| ≈ 1 g.
  const ZUPT_GYRO_THR_DEG = 25;     // deg/s: below = candidate stance
  const ZUPT_ACCEL_MAG_THR = 0.15;  // g:    |‖a‖ − 1| below = candidate stance
  const ZUPT_MIN_STANCE_LEN = 8;    // samples (~80 ms): shorter runs treated as noise
  const ZUPT_FILL_HOLE_MAX  = 3;    // samples: close short non-stance blips inside stance
  // Fallback (no stance found): fixed 1-s segments like the old behaviour.
  const SEGMENT_LENGTH = 100;

  function init() {
    ahrs = new MadgwickAHRS({ sampleInterval: 10, beta: 0.4 });

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a1a);

    camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 500);
    camera.position.set(5, 8, 12);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.update();

    scene.add(new THREE.AmbientLight(0x404060, 1.5));
    const dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.position.set(5, 10, 7);
    scene.add(dirLight);

    scene.add(new THREE.GridHelper(30, 30, 0x333355, 0x222244));
    scene.add(new THREE.ArrowHelper(
      new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 2, 0),
      2, 0xffff00, 0.3, 0.15
    ));

    originMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.15, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0x00ff88 })
    );
    scene.add(originMarker);

    boardGroup = new THREE.Group();
    scene.add(boardGroup);

    boardGroup.add(new THREE.Mesh(
      new THREE.BoxGeometry(0.6, 1.2, 0.6),
      new THREE.MeshPhongMaterial({ color: 0x3a7bd5 })
    ));

    const axisLen = 0.8, headLen = 0.12, headW = 0.06;
    boardGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), axisLen, 0xff0000, headLen, headW));
    boardGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), axisLen, 0x00ff00, headLen, headW));
    boardGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), axisLen, 0x4444ff, headLen, headW));

    trailPositions = new Float32Array(MAX_TRAIL_POINTS * 3);
    trailGeom = new THREE.BufferGeometry();
    trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
    trailGeom.setDrawRange(0, 0);
    trailLine = new THREE.Line(
      trailGeom,
      new THREE.LineBasicMaterial({ color: 0xff3333, linewidth: 2 })
    );
    scene.add(trailLine);

    animate();
  }

  function onResize() {
    if (!renderer) return;
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function animate() {
    requestAnimationFrame(animate);

    if (hasTarget && boardGroup) {
      const q = ahrs.getQuaternion();
      const targetQuat = new THREE.Quaternion(q.x, q.y, q.z, q.w);
      displayQuat.slerp(targetQuat, SLERP_ALPHA);
      boardGroup.quaternion.copy(displayQuat);

      displayPos.lerp(currentPos, 0.15);
      boardGroup.position.copy(displayPos);
    }

    controls.update();
    renderer.render(scene, camera);
  }

  function addTrailPoint(x, y, z) {
    if (trailCount >= MAX_TRAIL_POINTS) return;
    const i = trailCount * 3;
    trailPositions[i] = x;
    trailPositions[i + 1] = y;
    trailPositions[i + 2] = z;
    trailCount++;
    trailGeom.attributes.position.needsUpdate = true;
    trailGeom.setDrawRange(0, trailCount);
  }

  // NED → scene: X=north→Z, Y=east→X, Z=down→-Y
  function nedToScene(nx, ny, nz) {
    return { x: ny * SCALE, y: -nz * SCALE, z: nx * SCALE };
  }

  // ─── Quaternion helpers ───

  function removeGravity(ax, ay, az, qw, qx, qy, qz) {
    // Rotate world gravity [0,0,1] into sensor frame via conjugate quaternion
    const cqw = qw, cqx = -qx, cqy = -qy, cqz = -qz;
    const tx = 2 * cqy;
    const ty = -2 * cqx;
    const gSensorX = cqw * tx - cqz * ty;
    const gSensorY = cqw * ty + cqz * tx;
    const gSensorZ = 1 + (cqx * ty - cqy * tx);
    return { x: ax - gSensorX, y: ay - gSensorY, z: az - gSensorZ };
  }

  function rotateToWorld(vx, vy, vz, qw, qx, qy, qz) {
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    return {
      x: vx + qw * tx + (qy * tz - qz * ty),
      y: vy + qw * ty + (qz * tx - qx * tz),
      z: vz + qw * tz + (qx * ty - qy * tx)
    };
  }

  /**
   * Integrate one swing segment [from .. to] with drift correction.
   * Assumes velocity = 0 at both endpoints (enforced by ZUPT at anchors).
   * Writes positions[from+1 .. to] and returns final position.
   */
  function integrateSegment(worldAccels, positions, from, to) {
    const len = to - from;
    const startPos = positions[from];
    if (len <= 0) {
      positions[to] = { x: startPos.x, y: startPos.y, z: startPos.z };
      return positions[to];
    }

    // Integrate accel → velocity
    const vel = new Array(len + 1);
    vel[0] = { x: 0, y: 0, z: 0 };
    for (let i = 0; i < len; i++) {
      const a = worldAccels[from + i];
      const p = vel[i];
      vel[i + 1] = {
        x: p.x + a.x * G_TO_MS2 * DT,
        y: p.y + a.y * G_TO_MS2 * DT,
        z: p.z + a.z * G_TO_MS2 * DT
      };
    }

    // Drift correction: end velocity must be 0 (next ZUPT). Subtract linear ramp.
    const vEnd = vel[len];
    for (let i = 0; i <= len; i++) {
      const frac = i / len;
      vel[i].x -= vEnd.x * frac;
      vel[i].y -= vEnd.y * frac;
      vel[i].z -= vEnd.z * frac;
    }

    // Integrate corrected velocity → position
    for (let i = 1; i <= len; i++) {
      const prev = positions[from + i - 1];
      positions[from + i] = {
        x: prev.x + vel[i].x * DT,
        y: prev.y + vel[i].y * DT,
        z: prev.z + vel[i].z * DT
      };
    }
    return positions[to];
  }

  /**
   * Pre-compute the entire 3D path from all packets.
   *
   * Pipeline:
   *   Pass 1: AHRS → orientation → gravity-removed world-frame accel + gyro magnitude
   *   Pass 2: Stance detection (ω ≈ 0 AND |a| ≈ 1 g), morphological closing,
   *           min-run filtering → stance[] bitmap
   *   Pass 3: Segment integration — between two consecutive stance samples that
   *           are separated (swing phase), integrate accel → vel → pos with end-point
   *           ZUPT drift correction. Adjacent stance samples = foot not moving,
   *           position stays locked.
   *
   * If no stance regions are found (e.g. continuous waving, not foot-mounted),
   * fall back to fixed 1-s segments.
   */
  function precomputePath(packets) {
    console.log('[INS] precomputePath called, packets:', packets ? packets.length : 'null');
    if (!packets || !packets.length) {
      precomputedPath = null;
      return;
    }

    try {
    // ── Pass 1: AHRS + feature extraction ──
    const pathAhrs = new MadgwickAHRS({ sampleInterval: 10, beta: 0.4 });
    let pathMag = null;
    const worldAccels = [];
    const gyroMags = [];
    const accelMags = [];

    for (let pi = 0; pi < packets.length; pi++) {
      const pkt = packets[pi];
      if (!pkt.json || !pkt.json.IMU) continue;
      const json = pkt.json;
      if (json.mag) pathMag = json.mag;

      for (let si = 0; si < json.IMU.length; si++) {
        const s = json.IMU[si];
        if (!s || !s.a) continue;
        const a = s.a;
        const g = s.g;

        const gxDeg = g ? g.x : 0;
        const gyDeg = g ? g.y : 0;
        const gzDeg = g ? g.z : 0;
        const gx = gxDeg * (Math.PI / 180);
        const gy = gyDeg * (Math.PI / 180);
        const gz = gzDeg * (Math.PI / 180);

        if (pathMag) {
          pathAhrs.update(gx, gy, gz, a.x, a.y, a.z, pathMag.x, pathMag.y, pathMag.z);
        } else {
          pathAhrs.update(gx, gy, gz, a.x, a.y, a.z);
        }

        const q = pathAhrs.getQuaternion();
        const lin = removeGravity(a.x, a.y, a.z, q.w, q.x, q.y, q.z);

        if (Math.abs(lin.x) < DEAD_ZONE) lin.x = 0;
        if (Math.abs(lin.y) < DEAD_ZONE) lin.y = 0;
        if (Math.abs(lin.z) < DEAD_ZONE) lin.z = 0;

        worldAccels.push(rotateToWorld(lin.x, lin.y, lin.z, q.w, q.x, q.y, q.z));
        gyroMags.push(Math.sqrt(gxDeg*gxDeg + gyDeg*gyDeg + gzDeg*gzDeg));
        accelMags.push(Math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z));
      }
    }

    const n = worldAccels.length;
    if (n === 0) { precomputedPath = null; return; }

    // ── Pass 2: Stance detection ──
    const stance = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const gyroOk  = gyroMags[i] < ZUPT_GYRO_THR_DEG;
      const accelOk = Math.abs(accelMags[i] - 1.0) < ZUPT_ACCEL_MAG_THR;
      stance[i] = (gyroOk && accelOk) ? 1 : 0;
    }
    // Morphological closing: fill short non-stance holes inside stance runs.
    for (let i = 0; i < n; i++) {
      if (stance[i]) continue;
      let j = i;
      while (j < n && !stance[j]) j++;
      const gapLen = j - i;
      const leftIsStance  = i > 0     && stance[i - 1];
      const rightIsStance = j < n     && stance[j];
      if (leftIsStance && rightIsStance && gapLen <= ZUPT_FILL_HOLE_MAX) {
        for (let k = i; k < j; k++) stance[k] = 1;
      }
      i = j - 1;
    }
    // Morphological opening: remove stance runs shorter than MIN_STANCE_LEN (noise blips).
    for (let i = 0; i < n; i++) {
      if (!stance[i]) continue;
      let j = i;
      while (j < n && stance[j]) j++;
      const runLen = j - i;
      if (runLen < ZUPT_MIN_STANCE_LEN) {
        for (let k = i; k < j; k++) stance[k] = 0;
      }
      i = j - 1;
    }

    // Count stance samples for diagnostics / fallback decision
    let stanceCount = 0;
    for (let i = 0; i < n; i++) if (stance[i]) stanceCount++;

    const positions = new Array(n);
    positions[0] = { x: 0, y: 0, z: 0 };

    if (stanceCount < ZUPT_MIN_STANCE_LEN * 2) {
      // ── Fallback: fixed 1-s segments, v=0 at each boundary ──
      let prev = 0;
      for (let i = SEGMENT_LENGTH; i < n; i += SEGMENT_LENGTH) {
        integrateSegment(worldAccels, positions, prev, i);
        prev = i;
      }
      if (prev < n - 1) integrateSegment(worldAccels, positions, prev, n - 1);
      console.log(`[INS] No stance regions detected (stanceCount=${stanceCount}) — fell back to fixed 1-s ZUPT`);
    } else {
      // ── ZUPT integration ──
      // Walk through samples: stance samples keep position fixed; gaps between
      // stance samples are swing phases that get integrated with endpoint ZUPT.
      let lastAnchor = -1;
      for (let i = 0; i < n; i++) {
        if (stance[i]) {
          if (lastAnchor < 0) {
            // Pre-stance prefix: integrate from 0 up to this stance sample
            // (assume v=0 at sample 0 as initial condition).
            if (i > 0) integrateSegment(worldAccels, positions, 0, i);
            else positions[0] = { x: 0, y: 0, z: 0 };
          } else if (i === lastAnchor + 1) {
            // Adjacent stance sample: foot not moving → position stays
            positions[i] = { x: positions[lastAnchor].x, y: positions[lastAnchor].y, z: positions[lastAnchor].z };
          } else {
            // Swing phase between two stance anchors
            integrateSegment(worldAccels, positions, lastAnchor, i);
          }
          lastAnchor = i;
        }
      }
      // Post-stance suffix: integrate from last anchor to end
      if (lastAnchor < 0) {
        // (shouldn't happen given stanceCount check above, but be safe)
        integrateSegment(worldAccels, positions, 0, n - 1);
      } else if (lastAnchor < n - 1) {
        integrateSegment(worldAccels, positions, lastAnchor, n - 1);
      }
    }

    // Safety fill
    for (let i = 0; i < n; i++) {
      if (!positions[i]) positions[i] = { x: 0, y: 0, z: 0 };
    }

    precomputedPath = positions;
    sampleIndex = 0;

    // Stats
    let maxDist = 0, minZ = Infinity, maxZ = -Infinity;
    for (const p of positions) {
      const d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      if (d > maxDist) maxDist = d;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    }
    console.log(`[INS] Path: ${n} samples, stance=${stanceCount} (${(100*stanceCount/n).toFixed(1)}%), maxDist=${maxDist.toFixed(2)}m (${(maxDist*SCALE).toFixed(1)} scene), vertRange=${(maxZ-minZ).toFixed(3)}m (${((maxZ-minZ)*SCALE).toFixed(1)} scene Y)`);

    } catch (err) {
      console.error('[INS] precomputePath FAILED:', err);
      precomputedPath = null;
    }
  }

  function updateSample(sample, mag, deltaTimeSec) {
    if (!boardGroup || !sample || !sample.a) return;

    const a = sample.a;
    const g = sample.g;
    if (mag) lastMag = mag;

    const gx = g ? g.x * (Math.PI / 180) : 0;
    const gy = g ? g.y * (Math.PI / 180) : 0;
    const gz = g ? g.z * (Math.PI / 180) : 0;

    if (lastMag) {
      ahrs.update(gx, gy, gz, a.x, a.y, a.z, lastMag.x, lastMag.y, lastMag.z, deltaTimeSec);
    } else {
      ahrs.update(gx, gy, gz, a.x, a.y, a.z, undefined, deltaTimeSec);
    }

    if (!hasTarget) {
      const q = ahrs.getQuaternion();
      displayQuat.set(q.x, q.y, q.z, q.w);
      hasTarget = true;
    }

    if (precomputedPath && sampleIndex < precomputedPath.length) {
      const p = precomputedPath[sampleIndex];
      const s = nedToScene(p.x, p.y, p.z);
      currentPos.set(s.x, s.y, s.z);

      trailAddCounter++;
      if (trailAddCounter >= TRAIL_INTERVAL) {
        trailAddCounter = 0;
        addTrailPoint(s.x, s.y, s.z);
      }

      sampleIndex++;
    }
  }

  function update(pkt) {
    if (!pkt || !pkt.json) return;
    const json = pkt.json;
    if (!json.IMU || !json.IMU.length) return;
    const last = json.IMU[json.IMU.length - 1];
    updateSample(last, json.mag);
  }

  function reset() {
    ahrs = new MadgwickAHRS({ sampleInterval: 10, beta: 0.4 });
    lastMag = null;
    hasTarget = false;
    currentPos.set(0, 0, 0);
    displayPos.set(0, 0, 0);
    trailAddCounter = 0;

    precomputedPath = null;
    sampleIndex = 0;

    if (trailGeom) {
      trailCount = 0;
      trailGeom.setDrawRange(0, 0);
    }
  }

  return { init, update, updateSample, onResize, reset, precomputePath };
})();
