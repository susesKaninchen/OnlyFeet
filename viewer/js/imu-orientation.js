/**
 * imu-orientation.js – 3D board visualization from IMU data
 *
 * Rotation:    Madgwick AHRS filter fusing accel + gyro + mag → quaternion
 * Translation: Double integration of linear accel with ZUPT drift correction
 *              Board moves through 3D space, red trail line shows path
 */
const ImuOrientation = (() => {
  const container = document.getElementById('imu-orient-container');
  let scene, camera, renderer, controls;
  let boardGroup;

  // --- AHRS filter ---
  let ahrs = null;
  let lastMag = null;

  // --- Display smoothing ---
  const displayQuat = new THREE.Quaternion();
  let hasTarget = false;
  const SLERP_ALPHA = 0.3;

  // --- Gravity estimation (low-pass) ---
  const LP_ALPHA = 0.95;
  const gravEst = { x: 0, y: 0, z: 0 };
  let gravInitialized = false;

  // --- Double integration for position ---
  const DT = 0.01;              // 100Hz → 10ms
  const G_TO_MS2 = 9.81;       // convert g to m/s²
  const SCALE = 0.5;            // m → scene units
  const DEAD_ZONE = 0.08;       // g, ignore noise
  const VEL_DECAY = 0.98;       // slight velocity damping to limit drift
  const vel = { x: 0, y: 0, z: 0 };
  const pos = { x: 0, y: 0, z: 0 };
  const displayPos = new THREE.Vector3();
  const POS_LERP = 0.3;

  // --- ZUPT: zero-velocity update ---
  const ZUPT_ACCEL_THRESH = 0.12;  // g — max linAccel magnitude to count as "still"
  const ZUPT_GYRO_THRESH = 15;     // deg/s — max gyro magnitude to count as "still"
  let zuptCount = 0;
  const ZUPT_MIN_SAMPLES = 8;      // consecutive still samples before ZUPT fires

  // --- Trail line ---
  const MAX_TRAIL_POINTS = 10000;
  let trailPositions;   // Float32Array
  let trailLine;
  let trailGeom;
  let trailCount = 0;
  let trailAddCounter = 0;          // only add point every N samples
  const TRAIL_INTERVAL = 3;         // add trail point every 3 samples (30ms)

  function init() {
    ahrs = new MadgwickAHRS({ sampleInterval: 10, beta: 0.4 });

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a1a);

    camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 500);
    camera.position.set(3, 3, 5);
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

    // Grid + gravity arrow
    scene.add(new THREE.GridHelper(10, 10, 0x333355, 0x222244));
    scene.add(new THREE.ArrowHelper(
      new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 2, 0),
      2, 0xffff00, 0.3, 0.15
    ));

    // Board group
    boardGroup = new THREE.Group();
    scene.add(boardGroup);

    // Elongated box (~2x4x2 — lang in grüne Y-Achse)
    boardGroup.add(new THREE.Mesh(
      new THREE.BoxGeometry(2, 4, 2),
      new THREE.MeshPhongMaterial({ color: 0x3a7bd5 })
    ));

    const axisLen = 1.5, headLen = 0.2, headW = 0.1;
    boardGroup.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), axisLen, 0xff0000, headLen, headW));
    boardGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), axisLen, 0x00ff00, headLen, headW));
    boardGroup.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), axisLen, 0x4444ff, headLen, headW));

    // Trail line
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
      // Smooth rotation
      const q = ahrs.getQuaternion();
      const targetQuat = new THREE.Quaternion(q.x, q.y, q.z, q.w);
      displayQuat.slerp(targetQuat, SLERP_ALPHA);
      boardGroup.quaternion.copy(displayQuat);

      // Smooth position — lerp toward integrated pos
      const targetPos = new THREE.Vector3(
        pos.x * SCALE,
        pos.z * SCALE,    // NED Z (down) → scene Y (up), inverted below
        pos.y * SCALE     // NED Y (east) → scene Z
      );
      // NED: X=north→scene-Z, Y=east→scene+X, Z=down→scene-Y
      targetPos.set(
        pos.y * SCALE,
        -pos.z * SCALE,
        pos.x * SCALE
      );

      displayPos.lerp(targetPos, POS_LERP);
      boardGroup.position.copy(displayPos);

      // Camera follows board loosely
      controls.target.lerp(displayPos, 0.05);
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

  /**
   * Per-sample update (called at ~100Hz).
   */
  function updateSample(sample, mag) {
    if (!boardGroup || !sample || !sample.a) return;

    const a = sample.a;
    const g = sample.g;
    if (mag) lastMag = mag;

    // Gyro: deg/s → rad/s for AHRS
    const gx = g ? g.x * (Math.PI / 180) : 0;
    const gy = g ? g.y * (Math.PI / 180) : 0;
    const gz = g ? g.z * (Math.PI / 180) : 0;

    // Feed AHRS filter
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

    // --- Gravity estimation via low-pass ---
    if (!gravInitialized) {
      gravEst.x = a.x; gravEst.y = a.y; gravEst.z = a.z;
      gravInitialized = true;
    }
    gravEst.x = LP_ALPHA * gravEst.x + (1 - LP_ALPHA) * a.x;
    gravEst.y = LP_ALPHA * gravEst.y + (1 - LP_ALPHA) * a.y;
    gravEst.z = LP_ALPHA * gravEst.z + (1 - LP_ALPHA) * a.z;

    // Linear acceleration in sensor frame (g)
    let linX = a.x - gravEst.x;
    let linY = a.y - gravEst.y;
    let linZ = a.z - gravEst.z;

    // Dead zone
    if (Math.abs(linX) < DEAD_ZONE) linX = 0;
    if (Math.abs(linY) < DEAD_ZONE) linY = 0;
    if (Math.abs(linZ) < DEAD_ZONE) linZ = 0;

    const linMag = Math.sqrt(linX * linX + linY * linY + linZ * linZ);
    const gyroMag = g ? Math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z) : 0;

    // --- ZUPT detection ---
    const isStill = (linMag < ZUPT_ACCEL_THRESH) && (gyroMag < ZUPT_GYRO_THRESH);
    if (isStill) {
      zuptCount++;
      if (zuptCount >= ZUPT_MIN_SAMPLES) {
        // Reset velocity to zero — corrects drift
        vel.x = 0; vel.y = 0; vel.z = 0;
      }
    } else {
      zuptCount = 0;
    }

    // Rotate linear accel to world frame (NED) using AHRS quaternion
    const q = ahrs.getQuaternion();
    const orientQuat = new THREE.Quaternion(q.x, q.y, q.z, q.w);
    const linWorld = new THREE.Vector3(linX, linY, linZ).applyQuaternion(orientQuat);

    // Convert g → m/s² and integrate
    const ax_ms2 = linWorld.x * G_TO_MS2;
    const ay_ms2 = linWorld.y * G_TO_MS2;
    const az_ms2 = linWorld.z * G_TO_MS2;

    // Integrate: velocity += accel * dt
    vel.x = vel.x * VEL_DECAY + ax_ms2 * DT;
    vel.y = vel.y * VEL_DECAY + ay_ms2 * DT;
    vel.z = vel.z * VEL_DECAY + az_ms2 * DT;

    // Integrate: position += velocity * dt
    pos.x += vel.x * DT;
    pos.y += vel.y * DT;
    pos.z += vel.z * DT;

    // Add trail point periodically
    trailAddCounter++;
    if (trailAddCounter >= TRAIL_INTERVAL) {
      trailAddCounter = 0;
      // Same NED→scene mapping as animate()
      addTrailPoint(
        pos.y * SCALE,
        -pos.z * SCALE,
        pos.x * SCALE
      );
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
    gravEst.x = 0; gravEst.y = 0; gravEst.z = 0;
    gravInitialized = false;
    vel.x = 0; vel.y = 0; vel.z = 0;
    pos.x = 0; pos.y = 0; pos.z = 0;
    displayPos.set(0, 0, 0);
    zuptCount = 0;
    trailAddCounter = 0;

    // Clear trail
    if (trailGeom) {
      trailCount = 0;
      trailGeom.setDrawRange(0, 0);
    }
  }

  return { init, update, updateSample, onResize, reset };
})();
