// depth-lab.js — 깊이 카메라 실험실
// A three.js room is rendered from the MentorPi depth camera pose into an RGB target and a float
// target (view-space depth + object id). Sensor models (structured light / stereo / ToF) are then
// applied on the CPU so students can see ranges, noise, disparity quantisation and typical holes.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const FX640 = 473.4506985179141, FY640 = 474.2169451085363;       // MentorPi camera_info.yaml
const VFOV = 2 * Math.atan(240 / FY640) * 180 / Math.PI;           // ≈ 53.7°
const START = { x: -0.6, y: 0.0, yaw: 0.0 };

// ------------------------------------------------------------------ sensor model descriptions
const MODELS = {
  ideal: { name: '이상적 깊이', range: [0.0, 50], desc: '렌더링한 참값 깊이(z-depth)입니다. 잡음·결측이 없고 유리판까지 그대로 보입니다. 다른 모델과 비교하는 기준으로 쓰세요.' },
  sl: { name: '구조광', range: [0.2, 4.0], sd: 0.08, desc: '적외선 점무늬를 투사하고 IR 카메라가 본 점들의 <b>옆 이동량(시차 d)</b>으로 Z = f·B/d 를 구합니다(HP60C형: 0.2~4 m). 오차는 거리 제곱에 비례(σ<sub>Z</sub> = Z²·σ<sub>d</sub>/(f·B))하고, 시차가 1/8 픽셀 단위로 양자화되어 멀수록 계단이 생깁니다. 투사기와 카메라가 떨어져 있어 물체 <b>한쪽 가장자리에 그림자(결측)</b>가 생기고, 빛을 흡수하는 <b>검은 물체</b>와 <b>유리</b>는 잘 안 잡힙니다.' },
  stereo: { name: '스테레오', range: [0.3, 10], sd: 0.25, desc: '두 카메라 영상에서 같은 점을 찾아 시차로 거리를 구합니다(패시브). 무늬가 있는 곳은 잘 되지만 <b>무늬 없는 흰 벽</b>은 짝을 찾지 못해 결측이 되고, 멀수록 오차가 빠르게 커집니다. 한쪽 카메라에만 보이는 가림(occlusion) 영역도 결측입니다.' },
  tof: { name: 'ToF', range: [0.1, 5.0], desc: '빛이 갔다 오는 시간(위상차)으로 거리를 직접 잽니다. 오차가 거리에 거의 비례하지 않고 작지만, 물체 경계에서 앞·뒤 거리가 섞인 <b>날아다니는 점(flying pixel)</b>이 생기고, 어두운 물체는 신호가 약해 잡음·결측이 늘어납니다.' },
};

// ------------------------------------------------------------------ colour maps
function makeLUT(kind) {
  const lut = new Uint8Array(256 * 3);
  for (let i = 0; i < 256; i++) {
    const t = i / 255;
    let r, g, b;
    if (kind === 'turbo') {
      r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
      g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
      b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
    } else if (kind === 'jet') {
      r = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3)));
      g = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2)));
      b = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1)));
    } else { r = g = b = 1 - t; }
    lut[i * 3] = Math.round(255 * Math.min(1, Math.max(0, r)));
    lut[i * 3 + 1] = Math.round(255 * Math.min(1, Math.max(0, g)));
    lut[i * 3 + 2] = Math.round(255 * Math.min(1, Math.max(0, b)));
  }
  return lut;
}
const LUTS = { turbo: makeLUT('turbo'), jet: makeLUT('jet'), gray: makeLUT('gray') };

// ------------------------------------------------------------------ deterministic noise
let seed = 12345;
const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return (seed + 0.5) / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(rnd())) * Math.cos(2 * Math.PI * rnd());

// ------------------------------------------------------------------ scene
const scene = new THREE.Scene();
const objects = [];            // meshes that are rendered into the sensor
const ID = { floor: 1, wall: 2, poster: 3, box: 4, cyl: 5, ball: 6, chair: 7, person: 8, ramp: 9, dark: 10, glass: 11 };
const DARK = new Set([ID.dark]), GLASS = new Set([ID.glass]);

function canvasTexture(w, h, draw, repeat = null) {
  const c = document.createElement('canvas'); c.width = w; c.height = h;
  draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  if (repeat) { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(...repeat); }
  return t;
}
function add(mesh, id) {
  mesh.userData.id = id;
  mesh.userData.colorMat = mesh.material;
  mesh.userData.depthMat = new THREE.ShaderMaterial({
    uniforms: { uId: { value: id } },
    vertexShader: 'varying float vZ; void main(){ vec4 mv = modelViewMatrix * vec4(position,1.0); vZ = -mv.z; gl_Position = projectionMatrix * mv; }',
    fragmentShader: 'uniform float uId; varying float vZ; void main(){ gl_FragColor = vec4(vZ, uId, 0.0, 1.0); }',
    side: THREE.DoubleSide,
  });
  if (!mesh.parent) scene.add(mesh);
  objects.push(mesh);
  return mesh;
}
function buildScene() {
  const floorTex = canvasTexture(256, 256, (c, w, h) => {
    c.fillStyle = '#d9d2c3'; c.fillRect(0, 0, w, h);
    c.fillStyle = '#cbc3b1'; c.fillRect(0, 0, w / 2, h / 2); c.fillRect(w / 2, h / 2, w / 2, h / 2);
    for (let i = 0; i < 2500; i++) { const g = 150 + Math.floor(rnd() * 90); c.fillStyle = `rgb(${g},${g - 8},${g - 20})`; c.fillRect(rnd() * w, rnd() * h, 2, 2); }
  }, [5, 4]);
  const floor = add(new THREE.Mesh(new THREE.PlaneGeometry(5, 4), new THREE.MeshStandardMaterial({ map: floorTex, roughness: 0.95 })), ID.floor);
  floor.position.set(1.5, 0, 0);
  const wallMat = new THREE.MeshStandardMaterial({ color: 0xeef1f5, roughness: 0.9, side: THREE.DoubleSide });
  const wall = (x, y, w, rot) => { const m = add(new THREE.Mesh(new THREE.PlaneGeometry(w, 1.2), wallMat), ID.wall); m.position.set(x, y, 0.6); m.rotation.set(Math.PI / 2, rot, 0); };
  wall(4, 0, 4, -Math.PI / 2); wall(-1, 0, 4, Math.PI / 2); wall(1.5, 2, 5, Math.PI); wall(1.5, -2, 5, 0);
  const poster = add(new THREE.Mesh(new THREE.PlaneGeometry(1.4, 0.8), new THREE.MeshStandardMaterial({ map: canvasTexture(256, 160, (c, w, h) => {
    c.fillStyle = '#fef3c7'; c.fillRect(0, 0, w, h);
    for (let i = 0; i < 140; i++) { c.fillStyle = `hsl(${Math.floor(rnd() * 360)},70%,50%)`; c.beginPath(); c.arc(rnd() * w, rnd() * h, 3 + rnd() * 10, 0, 7); c.fill(); }
    c.fillStyle = '#111'; c.font = 'bold 26px sans-serif'; c.fillText('MentorPi', 70, 90);
  }) })), ID.poster);
  poster.position.set(3.98, 0.6, 0.65); poster.rotation.set(Math.PI / 2, -Math.PI / 2, 0);
  const std = (c) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.7 });
  const box = add(new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.4, 0.4), std(0x8b5e34)), ID.box); box.position.set(1.4, 0.55, 0.2);
  const cyl = add(new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.5, 32), std(0x64748b)), ID.cyl); cyl.rotation.x = Math.PI / 2; cyl.position.set(1.1, -0.55, 0.25);
  const ball = add(new THREE.Mesh(new THREE.SphereGeometry(0.12, 32, 20), std(0xdc2626)), ID.ball); ball.position.set(0.75, 0.15, 0.12);
  const chair = new THREE.Group();
  const cm = std(0x9a6b3f);
  const seat = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.42, 0.04), cm); seat.position.set(0, 0, 0.45); chair.add(seat);
  const back = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.42, 0.45), cm); back.position.set(0.19, 0, 0.7); chair.add(back);
  for (const [dx, dy] of [[-0.18, -0.18], [-0.18, 0.18], [0.18, -0.18], [0.18, 0.18]]) { const l = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.035, 0.45), cm); l.position.set(dx, dy, 0.225); chair.add(l); }
  chair.position.set(2.3, -0.1, 0);
  scene.add(chair);
  chair.traverse((o) => { if (o.isMesh) add(o, ID.chair); });
  const body = add(new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.2, 1.3, 24), std(0x2563eb)), ID.person); body.rotation.x = Math.PI / 2; body.position.set(3.0, 1.0, 0.65);
  const head = add(new THREE.Mesh(new THREE.SphereGeometry(0.12, 24, 16), std(0xf1c7a1)), ID.person); head.position.set(3.0, 1.0, 1.42);
  const ramp = add(new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.6, 0.03), std(0x16a34a)), ID.ramp); ramp.position.set(2.2, -1.25, 0.18); ramp.rotation.y = -0.4;
  const dark = add(new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.35, 0.35), new THREE.MeshStandardMaterial({ color: 0x080808, roughness: 1 })), ID.dark); dark.position.set(1.7, -1.05, 0.175);
  const glass = add(new THREE.Mesh(new THREE.PlaneGeometry(0.7, 0.7), new THREE.MeshStandardMaterial({ color: 0x9fd8ff, transparent: true, opacity: 0.22, roughness: 0.1, side: THREE.DoubleSide })), ID.glass);
  glass.position.set(1.9, 0.05, 0.35); glass.rotation.set(Math.PI / 2, Math.PI / 2, 0);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4); sun.position.set(1, -2, 4); scene.add(sun);
}

// ------------------------------------------------------------------ renderers & cameras
const cvWorld = $('cvWorld');
const renderer = new THREE.WebGLRenderer({ canvas: cvWorld, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const worldCam = new THREE.PerspectiveCamera(50, 1, 0.02, 40);
worldCam.up.set(0, 0, 1); worldCam.position.set(-1.6, -2.6, 2.4);
const worldCtl = new OrbitControls(worldCam, cvWorld); worldCtl.target.set(1.2, 0, 0.3); worldCtl.enableDamping = true;
const sensorCam = new THREE.PerspectiveCamera(VFOV, 4 / 3, 0.05, 20);
sensorCam.up.set(0, 0, 1);

// robot marker + frustum
const robot = new THREE.Group();
const chassis = new THREE.Mesh(new THREE.BoxGeometry(0.21, 0.18, 0.09), new THREE.MeshStandardMaterial({ color: 0x15803d }));
chassis.position.set(0, 0, 0.07); robot.add(chassis);
const camBox = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.09, 0.03), new THREE.MeshStandardMaterial({ color: 0x111827 }));
robot.add(camBox);
const frustum = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x4f46e5 }));
scene.add(robot, frustum);
const pickMarker = new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff00ff }));
pickMarker.visible = false; scene.add(pickMarker);

// point cloud viewer
const cvPc = $('cvPc');
const pcRenderer = new THREE.WebGLRenderer({ canvas: cvPc, antialias: true });
pcRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const pcScene = new THREE.Scene(); pcScene.background = new THREE.Color(0x0b0f14);
const pcCam = new THREE.PerspectiveCamera(50, 1, 0.02, 40); pcCam.up.set(0, 0, 1); pcCam.position.set(-1.8, -1.8, 1.6);
const pcCtl = new OrbitControls(pcCam, cvPc); pcCtl.target.set(1.2, 0, 0.3); pcCtl.enableDamping = true;
const grid = new THREE.GridHelper(6, 12, 0x334155, 0x1e293b); grid.rotation.x = Math.PI / 2; grid.position.set(1.5, 0, 0); pcScene.add(grid);
const pcGeom = new THREE.BufferGeometry();
const pcPoints = new THREE.Points(pcGeom, new THREE.PointsMaterial({ size: 0.012, vertexColors: true }));
pcScene.add(pcPoints);
const pcMarker = pickMarker.clone(); pcMarker.visible = false; pcScene.add(pcMarker);
const camAxes = new THREE.AxesHelper(0.15); pcScene.add(camAxes);

// ------------------------------------------------------------------ state
const st = { x: START.x, y: START.y, yaw: START.yaw, h: 0.121, pitch: 0, dirty: true, lastNoise: 0, view4: 'pc', pick: null };
let W = 320, H = 240, rtColor = null, rtDepth = null, bufC = null, bufD = null;
let Zt = null, Zs = null, ids = null, rgb = null, shadow = null, pattern = null, PW = 0;
function allocate() {
  W = Number($('res').value); H = Math.round(W * 3 / 4);
  rtColor?.dispose(); rtDepth?.dispose();
  rtColor = new THREE.WebGLRenderTarget(W, H); rtColor.texture.colorSpace = THREE.SRGBColorSpace;
  rtDepth = new THREE.WebGLRenderTarget(W, H, { type: THREE.FloatType });
  bufC = new Uint8Array(W * H * 4); bufD = new Float32Array(W * H * 4);
  Zt = new Float32Array(W * H); Zs = new Float32Array(W * H); ids = new Uint8Array(W * H); rgb = new Uint8Array(W * H * 3); shadow = new Uint8Array(W * H);
  PW = W * 3; pattern = new Uint8Array(PW * H);
  let s = 999; const r = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  for (let i = 0; i < pattern.length; i++) pattern[i] = r() < 0.09 ? 1 : 0;
  for (const cv of [$('cvRgb'), $('cvDepth'), $('cvIr')]) { cv.width = W; cv.height = H; }
  $('capRgbHdr').textContent = `rgb0/image · ${W}×${H}`;
  st.dirty = true;
}
const fx = () => FX640 * W / 640;
const fy = () => FY640 * H / 480;

function placeCamera() {
  const c = Math.cos(st.yaw), s = Math.sin(st.yaw), p = st.pitch * Math.PI / 180;
  const camX = st.x + 0.061 * c, camY = st.y + 0.061 * s;
  sensorCam.position.set(camX, camY, st.h);
  sensorCam.lookAt(camX + Math.cos(p) * c, camY + Math.cos(p) * s, st.h + Math.sin(p));
  sensorCam.updateMatrixWorld();
  robot.position.set(st.x, st.y, 0); robot.rotation.set(0, 0, st.yaw);
  camBox.position.set(0.061, 0, st.h); chassis.visible = st.h < 0.3;
  // frustum lines to 0.8 m
  const d = 0.8, hw = d * (W / 2) / fx(), hh = d * (H / 2) / fy();
  const corners = [[-hw, hh], [hw, hh], [hw, -hh], [-hw, -hh]].map(([a, b]) => new THREE.Vector3(a, b, -d).applyMatrix4(sensorCam.matrixWorld));
  const o = sensorCam.position;
  const pts = [];
  for (let i = 0; i < 4; i++) { pts.push(o, corners[i], corners[i], corners[(i + 1) % 4]); }
  frustum.geometry.setFromPoints(pts);
}

// ------------------------------------------------------------------ sensor capture
function capture() {
  placeCamera();
  sensorCam.aspect = W / H; sensorCam.updateProjectionMatrix();
  const bg = scene.background; scene.background = null;
  const vis = [robot.visible, frustum.visible, pickMarker.visible];
  robot.visible = frustum.visible = pickMarker.visible = false;
  renderer.setClearColor(0x000000, 1);
  renderer.setRenderTarget(rtColor); renderer.clear(); renderer.render(scene, sensorCam);
  renderer.readRenderTargetPixels(rtColor, 0, 0, W, H, bufC);
  for (const m of objects) m.material = m.userData.depthMat;
  renderer.setRenderTarget(rtDepth); renderer.setClearColor(0x000000, 0); renderer.clear(); renderer.render(scene, sensorCam);
  renderer.readRenderTargetPixels(rtDepth, 0, 0, W, H, bufD);
  for (const m of objects) m.material = m.userData.colorMat;
  renderer.setRenderTarget(null);
  scene.background = bg; [robot.visible, frustum.visible, pickMarker.visible] = vis;
  for (let v = 0; v < H; v++) {
    const src = (H - 1 - v) * W;                 // GL rows are bottom-up
    for (let u = 0; u < W; u++) {
      const i = v * W + u, j = src + u;
      Zt[i] = bufD[j * 4]; ids[i] = Math.round(bufD[j * 4 + 1]);
      rgb[i * 3] = bufC[j * 4]; rgb[i * 3 + 1] = bufC[j * 4 + 1]; rgb[i * 3 + 2] = bufC[j * 4 + 2];
    }
  }
}

function applyModel() {
  const model = $('model').value, M = MODELS[model], noise = $('noise').checked;
  const B = Number($('base').value), f = fx(), f640 = FX640;
  seed = 777 + Math.floor(performance.now() / 250);
  shadow.fill(0);
  // projector / second-camera shadow beside foreground left edges (triangulation sensors)
  if (noise && (model === 'sl' || model === 'stereo')) {
    for (let v = 0; v < H; v++) for (let u = 1; u < W; u++) {
      const i = v * W + u, zf = Zt[i], zb = Zt[i - 1];
      if (zf > 0 && zb > zf + 0.05) {
        const k = Math.min(80, Math.round(f * B * (1 / zf - 1 / zb)));
        for (let q = 1; q <= k && u - q >= 0; q++) shadow[i - q] = 1;
      }
    }
  }
  // texture (for stereo matching): local gradient energy on grey image
  let tex = null;
  if (noise && model === 'stereo') {
    tex = new Float32Array(W * H);
    for (let v = 1; v < H - 1; v++) for (let u = 1; u < W - 1; u++) {
      const g = (k) => rgb[k * 3] * 0.3 + rgb[k * 3 + 1] * 0.59 + rgb[k * 3 + 2] * 0.11;
      const i = v * W + u;
      tex[i] = Math.abs(g(i + 1) - g(i - 1)) + Math.abs(g(i + W) - g(i - W));
    }
    const box = new Float32Array(W * H), r = 3;             // 7×7 box mean
    for (let v = r; v < H - r; v++) for (let u = r; u < W - r; u++) {
      let s = 0; for (let a = -r; a <= r; a++) for (let b = -r; b <= r; b++) s += tex[(v + a) * W + u + b];
      box[v * W + u] = s / 49;
    }
    tex = box;
  }
  for (let i = 0; i < W * H; i++) {
    let z = Zt[i];
    const id = ids[i];
    if (!(z > 0)) { Zs[i] = 0; continue; }
    if (model === 'ideal' || !noise) {
      Zs[i] = (model === 'ideal' || (z >= M.range[0] && z <= M.range[1])) ? z : 0;
      continue;
    }
    if (GLASS.has(id)) { Zs[i] = 0; continue; }
    if (model === 'sl') {
      if (shadow[i] || (DARK.has(id) && rnd() < 0.85) || rnd() < 0.005) { Zs[i] = 0; continue; }
      const d = f640 * B / z + gauss() * M.sd;                 // disparity at native resolution
      const dq = Math.round(d * 8) / 8;                        // 1/8-pixel sub-pixel quantisation
      z = f640 * B / Math.max(dq, 1e-3);
    } else if (model === 'stereo') {
      if (shadow[i] || (tex && tex[i] < 2.5) || (DARK.has(id) && rnd() < 0.5)) { Zs[i] = 0; continue; }
      const d = f640 * B / z + gauss() * M.sd * (DARK.has(id) ? 3 : 1);
      z = f640 * B / Math.max(d, 1e-3);
    } else if (model === 'tof') {
      if (DARK.has(id) && rnd() < 0.6) { Zs[i] = 0; continue; }
      const u = i % W;
      const nb = [i - 1, i + 1, i - W, i + W].filter((k) => k >= 0 && k < W * H && Math.abs((k % W) - u) <= 1);
      let far = z;
      for (const k of nb) if (Zt[k] > far + 0.1) far = Zt[k];
      if (far > z && rnd() < 0.7) { const t = rnd(); z = z * (1 - t) + far * t; }     // flying pixel
      z += gauss() * (0.005 + 0.001 * z) * (DARK.has(id) ? 4 : 1);
    }
    Zs[i] = (z >= M.range[0] && z <= M.range[1]) ? z : 0;
  }
}

// ------------------------------------------------------------------ drawing
function drawRgb() {
  const ctx = $('cvRgb').getContext('2d'), img = ctx.createImageData(W, H);
  for (let i = 0; i < W * H; i++) { img.data[i * 4] = rgb[i * 3]; img.data[i * 4 + 1] = rgb[i * 3 + 1]; img.data[i * 4 + 2] = rgb[i * 3 + 2]; img.data[i * 4 + 3] = 255; }
  ctx.putImageData(img, 0, 0);
  drawPickCross(ctx);
}
function depthColor(z, lut, a, b) {
  const t = Math.max(0, Math.min(1, (z - a) / (b - a)));
  const k = Math.round(t * 255) * 3;
  return [lut[k], lut[k + 1], lut[k + 2]];
}
function drawDepth() {
  const ctx = $('cvDepth').getContext('2d'), img = ctx.createImageData(W, H);
  const lut = LUTS[$('cmap').value], a = Number($('dmin').value), b = Math.max(a + 0.1, Number($('dmax').value));
  let valid = 0;
  for (let i = 0; i < W * H; i++) {
    const z = Zs[i], o = i * 4;
    if (z > 0) { const c = depthColor(z, lut, a, b); img.data[o] = c[0]; img.data[o + 1] = c[1]; img.data[o + 2] = c[2]; valid++; }
    img.data[o + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  drawPickCross(ctx);
  // colour bar
  const cb = $('cbar'), cctx = cb.getContext('2d');
  for (let y = 0; y < cb.height; y++) { const k = Math.round((1 - y / (cb.height - 1)) * 255) * 3; cctx.fillStyle = `rgb(${lut[k]},${lut[k + 1]},${lut[k + 2]})`; cctx.fillRect(0, y, cb.width, 1); }
  $('cbarMax').textContent = b.toFixed(1) + ' m'; $('cbarMin').textContent = a.toFixed(1) + ' m';
  $('capDepth').textContent = `16UC1 · 0 = 무효(검정) · 유효 ${(100 * valid / (W * H)).toFixed(0)}%`;
  return valid;
}
function drawIr() {
  const ctx = $('cvIr').getContext('2d'), img = ctx.createImageData(W, H);
  const B = Number($('base').value), f = fx();
  for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
    const i = v * W + u, z = Zt[i], o = i * 4;
    const amb = (rgb[i * 3] + rgb[i * 3 + 1] + rgb[i * 3 + 2]) / 3 * 0.12;
    let val = amb;
    if (z > 0 && !shadow[i] && !GLASS.has(ids[i])) {
      const d = Math.round(f * B / z);                         // speckle shifts by the disparity
      const dot = pattern[v * PW + ((u + d) % PW + PW) % PW];
      const gain = (DARK.has(ids[i]) ? 0.12 : 1) * Math.min(1, 1.6 / (0.4 + z * z));
      val += dot * 235 * gain;
    }
    img.data[o] = img.data[o + 1] = img.data[o + 2] = Math.min(255, val); img.data[o + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  $('cap4').textContent = `IR 카메라 영상 (구조광 점무늬) · 점의 옆 이동 = 시차 d = f·B/Z (B=${B.toFixed(3)} m)`;
}
function drawErr() {
  const ctx = $('cvIr').getContext('2d'), img = ctx.createImageData(W, H), lut = LUTS.turbo;
  let s = 0, n = 0, mx = 0;
  for (let i = 0; i < W * H; i++) {
    const o = i * 4; img.data[o + 3] = 255;
    if (Zs[i] > 0 && Zt[i] > 0) {
      const e = Math.abs(Zs[i] - Zt[i]); s += e * e; n++; mx = Math.max(mx, e);
      const c = depthColor(e, lut, 0, 0.05); img.data[o] = c[0]; img.data[o + 1] = c[1]; img.data[o + 2] = c[2];
    } else if (Zt[i] > 0) { img.data[o] = 90; img.data[o + 1] = 0; img.data[o + 2] = 90; }
  }
  ctx.putImageData(img, 0, 0);
  $('cap4').textContent = `|측정 − 참값| (0~5 cm 색상) · 보라 = 결측 · RMS ${n ? (Math.sqrt(s / n) * 1000).toFixed(1) : '—'} mm, 최대 ${(mx * 1000).toFixed(0)} mm`;
}
function updatePointCloud() {
  const step = W >= 640 ? 2 : 1, lut = LUTS[$('cmap').value];
  const a = Number($('dmin').value), b = Math.max(a + 0.1, Number($('dmax').value));
  const pos = [], col = [], m = sensorCam.matrixWorld, v3 = new THREE.Vector3();
  const f = fx(), g = fy(), cx = W / 2, cy = H / 2, byDepth = $('pcColor').value === 'depth';
  for (let v = 0; v < H; v += step) for (let u = 0; u < W; u += step) {
    const i = v * W + u, z = Zs[i];
    if (!(z > 0)) continue;
    // optical frame (x right, y down, z forward) -> three camera frame (x right, y up, z back)
    const X = (u + 0.5 - cx) / f * z, Y = (v + 0.5 - cy) / g * z;
    v3.set(X, -Y, -z).applyMatrix4(m);
    pos.push(v3.x, v3.y, v3.z);
    if (byDepth) { const c = depthColor(z, lut, a, b); col.push(c[0] / 255, c[1] / 255, c[2] / 255); }
    else col.push(rgb[i * 3] / 255, rgb[i * 3 + 1] / 255, rgb[i * 3 + 2] / 255);
  }
  pcGeom.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  pcGeom.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  pcGeom.computeBoundingSphere();
  camAxes.position.copy(sensorCam.position); camAxes.quaternion.copy(sensorCam.quaternion);
  if (st.view4 === 'pc') $('cap4').textContent = `PointCloud2 · ${(pos.length / 3).toLocaleString()}점 (${step > 1 ? '2픽셀 간격 표시' : '전체'}) · 월드 좌표`;
}
function drawHist() {
  const cv = $('hist'), ctx = cv.getContext('2d'), bins = new Array(50).fill(0);
  let n = 0;
  for (let i = 0; i < W * H; i++) if (Zs[i] > 0) { bins[Math.min(49, Math.floor(Zs[i] / 0.1))]++; n++; }
  const mx = Math.max(1, ...bins), cs = getComputedStyle(document.documentElement);
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = cs.getPropertyValue('--accent') || '#4f46e5';
  bins.forEach((c, k) => { const hgt = (cv.height - 14) * c / mx; ctx.fillRect(k * cv.width / 50 + 1, cv.height - 12 - hgt, cv.width / 50 - 2, hgt); });
  ctx.fillStyle = cs.getPropertyValue('--text-faint') || '#888'; ctx.font = '10px sans-serif';
  for (let m = 0; m <= 5; m++) ctx.fillText(m + 'm', m * cv.width / 5 - (m === 5 ? 18 : 0), cv.height - 1);
  const zs = []; for (let i = 0; i < W * H; i += 7) if (Zs[i] > 0) zs.push(Zs[i]);
  zs.sort((p, q) => p - q);
  $('stats').textContent = `유효 ${(100 * n / (W * H)).toFixed(1)}% · 최소 ${zs.length ? zs[0].toFixed(2) : '—'} m · 중앙값 ${zs.length ? zs[zs.length >> 1].toFixed(2) : '—'} m · fx=${fx().toFixed(1)} px`;
}
function drawPickCross(ctx) {
  if (!st.pick) return;
  const [u, v] = st.pick;
  ctx.strokeStyle = '#ff00ff'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(u - 6, v + 0.5); ctx.lineTo(u + 7, v + 0.5); ctx.moveTo(u + 0.5, v - 6); ctx.lineTo(u + 0.5, v + 7); ctx.stroke();
}
function updatePick() {
  if (!st.pick) return;
  const [u, v] = st.pick, i = v * W + u, z = Zs[i], zt = Zt[i];
  const f = fx(), g = fy(), cx = W / 2, cy = H / 2;
  if (!(z > 0)) {
    $('pick').textContent = `(u, v) = (${u}, ${v})\n측정 깊이 = 0 (무효 픽셀)\n참값 Z = ${zt > 0 ? zt.toFixed(3) + ' m' : '—'}\n→ ${reason(i)}`;
    pickMarker.visible = pcMarker.visible = false; return;
  }
  const X = (u + 0.5 - cx) / f * z, Y = (v + 0.5 - cy) / g * z;
  const w = new THREE.Vector3(X, -Y, -z).applyMatrix4(sensorCam.matrixWorld);
  pickMarker.position.copy(w); pcMarker.position.copy(w); pickMarker.visible = pcMarker.visible = true;
  $('pick').textContent =
    `(u, v) = (${u}, ${v})   depth = ${Math.round(z * 1000)} mm\n` +
    `참값 Z = ${zt.toFixed(3)} m   오차 = ${((z - zt) * 1000).toFixed(1)} mm\n` +
    `X = (u+0.5−cx)/fx·Z = ${X.toFixed(3)} m\nY = (v+0.5−cy)/fy·Z = ${Y.toFixed(3)} m\n` +
    `광학 좌표 (X, Y, Z) = (${X.toFixed(3)}, ${Y.toFixed(3)}, ${z.toFixed(3)})\n` +
    `월드 좌표 (x, y, z) = (${w.x.toFixed(2)}, ${w.y.toFixed(2)}, ${w.z.toFixed(2)})`;
}
function reason(i) {
  const model = $('model').value, M = MODELS[model], z = Zt[i], id = ids[i];
  if (!(z > 0)) return '장면 밖(배경)';
  if (!$('noise').checked || model === 'ideal') return z < M.range[0] ? '최소 거리보다 가까움' : '최대 거리보다 멂';
  if (z < M.range[0]) return `최소 측정 거리 ${M.range[0]} m보다 가까움`;
  if (z > M.range[1]) return `최대 측정 거리 ${M.range[1]} m보다 멂`;
  if (GLASS.has(id)) return '유리: 빛이 통과·반사되어 측정 불가';
  if (shadow[i]) return model === 'sl' ? '투사기 그림자(점무늬가 닿지 않는 영역)' : '가림(occlusion): 한쪽 카메라에만 보임';
  if (DARK.has(id)) return '검은 물체: 반사광이 약함';
  if (model === 'stereo') return '무늬 없는 면: 짝 찾기(매칭) 실패';
  return '무작위 결측';
}
$('cvDepth').addEventListener('click', (e) => {
  const cv = e.currentTarget, r = cv.getBoundingClientRect();
  const s = Math.min(r.width / W, r.height / H), ox = (r.width - W * s) / 2, oy = (r.height - H * s) / 2;
  const u = Math.floor((e.clientX - r.left - ox) / s), v = Math.floor((e.clientY - r.top - oy) / s);
  if (u < 0 || v < 0 || u >= W || v >= H) return;
  st.pick = [u, v]; drawDepth(); drawRgb(); updatePick();
});

// ------------------------------------------------------------------ UI
function syncOut() {
  $('oB').textContent = Number($('base').value).toFixed(3);
  $('oMin').textContent = Number($('dmin').value).toFixed(2);
  $('oMax').textContent = Number($('dmax').value).toFixed(2);
  $('oH').textContent = Number($('camH').value).toFixed(3);
  $('oP').textContent = $('camP').value;
  $('modelDesc').innerHTML = MODELS[$('model').value].desc;
}
for (const id of ['model', 'base', 'noise', 'cmap', 'dmin', 'dmax', 'pcColor', 'camH', 'camP']) {
  $(id).addEventListener('input', () => {
    st.h = Number($('camH').value); st.pitch = Number($('camP').value);
    if (id === 'model') { const r = MODELS[$('model').value].range; if (r[1] < 20) { $('dmax').value = Math.min(8, r[1]); } }
    syncOut(); st.dirty = true;
  });
}
$('res').addEventListener('change', () => { allocate(); st.pick = null; });
$('btnReset').addEventListener('click', () => {
  Object.assign(st, { x: START.x, y: START.y, yaw: START.yaw, h: 0.121, pitch: 0, pick: null, dirty: true });
  $('camH').value = 0.121; $('camP').value = 0; syncOut();
  pickMarker.visible = pcMarker.visible = false; $('pick').textContent = '깊이 영상에서 한 점을 클릭하세요.';
});
document.querySelectorAll('[data-v4]').forEach((b) => b.addEventListener('click', () => {
  st.view4 = b.dataset.v4;
  document.querySelectorAll('[data-v4]').forEach((x) => x.classList.toggle('on', x === b));
  $('cvPc').hidden = st.view4 !== 'pc'; $('cvIr').hidden = st.view4 === 'pc';
  st.dirty = true;
}));
const keys = new Set();
window.addEventListener('keydown', (e) => { if (!e.target.closest('input, select')) { const k = e.key.toLowerCase(); if ('wasd'.includes(k) && k.length === 1) { keys.add(k); e.preventDefault(); } } });
window.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => keys.clear());

function resize() {
  for (const [r, cam, cv] of [[renderer, worldCam, cvWorld], [pcRenderer, pcCam, cvPc]]) {
    const b = cv.parentElement.getBoundingClientRect();
    r.setSize(Math.max(50, b.width), Math.max(50, b.height), false);
    cam.aspect = b.width / Math.max(1, b.height); cam.updateProjectionMatrix();
  }
}
new ResizeObserver(resize).observe(document.querySelector('.dl-grid'));

// ------------------------------------------------------------------ loop
let last = 0;
function frame(ts) {
  const dt = last ? Math.min(0.05, (ts - last) / 1000) : 0; last = ts;
  if (keys.size) {
    const v = (keys.has('w') ? 0.5 : 0) - (keys.has('s') ? 0.5 : 0), w = (keys.has('a') ? 1.0 : 0) - (keys.has('d') ? 1.0 : 0);
    st.yaw += w * dt;
    st.x = Math.max(-0.8, Math.min(3.8, st.x + v * dt * Math.cos(st.yaw)));
    st.y = Math.max(-1.8, Math.min(1.8, st.y + v * dt * Math.sin(st.yaw)));
    st.dirty = true;
  }
  const noisy = $('noise').checked && $('model').value !== 'ideal';
  if (st.dirty || (noisy && ts - st.lastNoise > 300)) {
    capture(); applyModel();
    drawRgb(); drawDepth(); drawHist();
    if (st.view4 === 'pc') updatePointCloud(); else if (st.view4 === 'ir') drawIr(); else drawErr();
    if (st.view4 !== 'pc') updatePointCloud();
    updatePick();
    st.dirty = false; st.lastNoise = ts;
  }
  $('capWorld').textContent = `카메라 (${st.x.toFixed(2)}, ${st.y.toFixed(2)}) yaw ${(st.yaw * 180 / Math.PI).toFixed(0)}° · 높이 ${st.h.toFixed(2)} m · 기울기 ${st.pitch}°`;
  worldCtl.update(); pcCtl.update();
  renderer.render(scene, worldCam);
  if (st.view4 === 'pc') pcRenderer.render(pcScene, pcCam);
  requestAnimationFrame(frame);
}

// test hook
window.__depthlab = {
  state: () => {
    let valid = 0; for (let i = 0; i < W * H; i++) if (Zs[i] > 0) valid++;
    const c = (H >> 1) * W + (W >> 1);
    return { W, H, model: $('model').value, valid: valid / (W * H), centerTrue: Zt[c], centerMeas: Zs[c], centerId: ids[c] };
  },
  set: (o) => { Object.assign(st, o); st.dirty = true; },
  refresh: () => { capture(); applyModel(); drawRgb(); drawDepth(); drawHist(); updatePointCloud(); },
};

buildScene();
scene.background = new THREE.Color(0xdfe6ee);
allocate(); syncOut(); resize();
$('status').textContent = `HP60C 모델 · fx ${FX640.toFixed(1)} px · 수직 화각 ${VFOV.toFixed(1)}°`;
requestAnimationFrame(frame);
