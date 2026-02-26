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
  const DEAD_ZONE = 0.02;           // very small — drift correction handles the rest
  const SEGMENT_LENGTH = 100;        // samples per segment (~1s) — drift correction interval

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
   * Pre-compute the entire 3D path from all packets.
   *
   * 1. Run AHRS on all samples → orientation quaternion per sample
   * 2. Remove gravity (quaternion-based), rotate linear accel to world frame
   * 3. Split into fixed-length segments (~1s each)
   * 4. Per segment: integrate accel→vel→pos, correct velocity drift retroactively
   * 5. Chain segments together for continuous path
   */
  function precomputePath(packets) {
    console.log('[INS] precomputePath called, packets:', packets ? packets.length : 'null');
    if (!packets || !packets.length) {
      precomputedPath = null;
      return;
    }

    try {
    // ── Pass 1: Run AHRS, compute world-frame linear acceleration ──
    const pathAhrs = new MadgwickAHRS({ sampleInterval: 10, beta: 0.4 });
    let pathMag = null;
    const worldAccels = [];

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

        const gx = g ? g.x * (Math.PI / 180) : 0;
        const gy = g ? g.y * (Math.PI / 180) : 0;
        const gz = g ? g.z * (Math.PI / 180) : 0;

        if (pathMag) {
          pathAhrs.update(gx, gy, gz, a.x, a.y, a.z, pathMag.x, pathMag.y, pathMag.z);
        } else {
          pathAhrs.update(gx, gy, gz, a.x, a.y, a.z);
        }

        const q = pathAhrs.getQuaternion();
        const lin = removeGravity(a.x, a.y, a.z, q.w, q.x, q.y, q.z);

        // Small dead zone to suppress noise at rest
        if (Math.abs(lin.x) < DEAD_ZONE) lin.x = 0;
        if (Math.abs(lin.y) < DEAD_ZONE) lin.y = 0;
        if (Math.abs(lin.z) < DEAD_ZONE) lin.z = 0;

        worldAccels.push(rotateToWorld(lin.x, lin.y, lin.z, q.w, q.x, q.y, q.z));
      }
    }

    const n = worldAccels.length;
    if (n === 0) { precomputedPath = null; return; }

    // ── Pass 2: Build segment boundaries at regular intervals ──
    const anchors = [0];
    for (let i = SEGMENT_LENGTH; i < n; i += SEGMENT_LENGTH) {
      anchors.push(i);
    }
    if (anchors[anchors.length - 1] !== n - 1) {
      anchors.push(n - 1);
    }

    // ── Pass 3: Per-segment integration with velocity drift correction ──
    const positions = new Array(n);
    let curPos = { x: 0, y: 0, z: 0 };

    for (let seg = 0; seg < anchors.length - 1; seg++) {
      const start = anchors[seg];
      const end = anchors[seg + 1];
      const segLen = end - start;

      if (segLen === 0) {
        positions[start] = { x: curPos.x, y: curPos.y, z: curPos.z };
        continue;
      }

      // Integrate accel → velocity
      const velArr = [{ x: 0, y: 0, z: 0 }];
      for (let i = start; i < end; i++) {
        const prev = velArr[velArr.length - 1];
        const acc = worldAccels[i];
        velArr.push({
          x: prev.x + acc.x * G_TO_MS2 * DT,
          y: prev.y + acc.y * G_TO_MS2 * DT,
          z: prev.z + acc.z * G_TO_MS2 * DT
        });
      }

      // Retroactive velocity drift correction:
      // Assume velocity at segment end should be same as start (≈0 relative).
      // Linearly subtract the accumulated drift.
      const endVel = velArr[segLen];
      for (let i = 0; i <= segLen; i++) {
        const frac = i / segLen;
        velArr[i].x -= endVel.x * frac;
        velArr[i].y -= endVel.y * frac;
        velArr[i].z -= endVel.z * frac;
      }

      // Integrate corrected velocity → position
      positions[start] = { x: curPos.x, y: curPos.y, z: curPos.z };
      for (let i = 1; i <= segLen; i++) {
        const prev = positions[start + i - 1];
        positions[start + i] = {
          x: prev.x + velArr[i].x * DT,
          y: prev.y + velArr[i].y * DT,
          z: prev.z + velArr[i].z * DT
        };
      }

      curPos = positions[end];
    }

    // Fill any gaps
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
    console.log(`[INS] Path: ${n} samples, ${anchors.length} segments, maxDist=${maxDist.toFixed(2)}m (${(maxDist*SCALE).toFixed(1)} scene), vertRange=${(maxZ-minZ).toFixed(3)}m (${((maxZ-minZ)*SCALE).toFixed(1)} scene Y)`);

    } catch (err) {
      console.error('[INS] precomputePath FAILED:', err);
      precomputedPath = null;
    }
  }

  function updateSample(sample, mag) {
    if (!boardGroup || !sample || !sample.a) return;

    const a = sample.a;
    const g = sample.g;
    if (mag) lastMag = mag;

    const gx = g ? g.x * (Math.PI / 180) : 0;
    const gy = g ? g.y * (Math.PI / 180) : 0;
    const gz = g ? g.z * (Math.PI / 180) : 0;

    if (lastMag) {
      ahrs.update(gx, gy, gz, a.x, a.y, a.z, lastMag.x, lastMag.y, lastMag.z);
    } else {
      ahrs.update(gx, gy, gz, a.x, a.y, a.z);
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
