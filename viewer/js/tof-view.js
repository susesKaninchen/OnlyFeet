/**
 * tof-view.js – Three.js 3D heightmap for 8×8 ToF data
 */
const TofView = (() => {
  const container = document.getElementById('tof-container');
  let scene, camera, renderer, controls;
  let boxes = [];  // 8x8 = 64 box meshes
  const GRID = 8;
  const SPACING = 1.2;
  const MAX_DIST = 2000; // mm, for scaling height
  const MAX_HEIGHT = 8;

  function init() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a1a);

    camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 100);
    const cx = GRID * SPACING / 2;
    const cz = GRID * SPACING / 2;
    camera.position.set(cx - 10, 18, cz);
    camera.lookAt(cx, 0, cz);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(GRID * SPACING / 2, 0, GRID * SPACING / 2);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.update();

    // Ambient + directional light
    scene.add(new THREE.AmbientLight(0x404060, 1.5));
    const dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.position.set(5, 15, 10);
    scene.add(dirLight);

    // Create 8x8 grid of boxes
    const geo = new THREE.BoxGeometry(1, 1, 1);
    for (let row = 0; row < GRID; row++) {
      for (let col = 0; col < GRID; col++) {
        const mat = new THREE.MeshPhongMaterial({ color: 0x4444aa });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.x = col * SPACING;
        mesh.position.z = row * SPACING;
        mesh.position.y = 0;
        scene.add(mesh);
        boxes.push(mesh);
      }
    }

    // Grid helper
    const gridHelper = new THREE.GridHelper(GRID * SPACING, GRID, 0x333355, 0x222244);
    gridHelper.position.set(GRID * SPACING / 2 - SPACING / 2, -0.01, GRID * SPACING / 2 - SPACING / 2);
    scene.add(gridHelper);

    animate();
    window.addEventListener('resize', onResize);
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
    controls.update();
    renderer.render(scene, camera);
  }

  /**
   * Update the heightmap with a single ToF frame.
   * frame: { t, d: [64], s: [64] }
   */
  function updateFrame(frame) {
    if (!frame || !frame.d) return;
    const d = frame.d;
    const s = frame.s;

    for (let i = 0; i < 64 && i < boxes.length; i++) {
      const dist = d[i];
      const status = s[i];
      const valid = (status === 5 || status === 9);

      // Height: invert distance (closer = taller)
      let height;
      if (valid) {
        height = Math.max(0.1, MAX_HEIGHT * (1 - Math.min(dist, MAX_DIST) / MAX_DIST));
      } else {
        height = 0.1; // minimal for invalid
      }

      const mesh = boxes[i];
      mesh.scale.y = height;
      mesh.position.y = height / 2;

      // Color: near = warm (red), far = cool (blue)
      if (valid) {
        const t = Math.min(dist, MAX_DIST) / MAX_DIST; // 0=near, 1=far
        const hue = t * 0.65; // 0 (red) → 0.65 (blue)
        mesh.material.color.setHSL(hue, 0.9, 0.5);
        mesh.material.opacity = 1;
        mesh.material.transparent = false;
      } else {
        mesh.material.color.setHSL(0, 0, 0.2);
        mesh.material.opacity = 0.3;
        mesh.material.transparent = true;
      }
    }
  }

  return { init, updateFrame, onResize };
})();
