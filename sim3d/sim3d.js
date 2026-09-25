// sim3d.js — URDF-based 3D simulator. The robot is the real MentorPi URDF (assets/urdf/mentorpi);
// kinematics, lidar, lidar-app behaviours, line follower and A*/Pure Pursuit are the same modules
// the 2D simulator uses (sim/*.js), so both simulators behave identically.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import URDFLoader from 'urdf-loader';
import { objColor } from '../assets/js/world2d.js';
import { World, Robot, makeRng, scan, rayAngle, N_RAYS, APP_LIMIT, PHYS_LIMIT, wrap } from '../sim/physics.js';
import { LidarApp, LineFollower } from '../sim/behaviors.js';
import { Grid } from '../sim/grid.js';
import { Costmap, astar, purePursuit, NAV_DEFAULTS } from '../sim/nav.js';

const $ = (id) => document.getElementById(id);
const DT = 0.02;
const LIDAR_Z = 0.07 + 0.092501;         // base_footprint -> base_link -> lidar_frame
const CAM_POS = [0.061376, 0, 0.07 + 0.051154];
const LS = 'studymentorpi.sim3d.v1';
const lsGet = () => { try { return JSON.parse(localStorage.getItem(LS) || '{}'); } catch (e) { return {}; } };
const lsSet = (o) => { try { localStorage.setItem(LS, JSON.stringify({ ...lsGet(), ...o })); } catch (e) { /* ignore */ } };

let WORLDS, world, robot, rng, ranges = new Array(N_RAYS).fill(Infinity);
let simT = 0, scanAcc = 0, ctrlAcc = 0, mode = 'manual';
const keys = new Set();
const lidarApp = new LidarApp(), lineF = new LineFollower();
let lineMask = null, costmap = null, nav = null, trail = [];

// ------------------------------------------------------------------ three.js
const host = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
host.appendChild(renderer.domElement);
const pipRenderer = new THREE.WebGLRenderer({ canvas: $('pip'), antialias: true });
pipRenderer.setSize(320, 240, false);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xdfe6ee);
const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 60);
camera.up.set(0, 0, 1);
const pipCam = new THREE.PerspectiveCamera(62, 4 / 3, 0.02, 20);
pipCam.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x667788, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
sun.position.set(2, -3, 6); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: 0.5, far: 20 });
scene.add(sun);
const worldGroup = new THREE.Group(); scene.add(worldGroup);
const robotGroup = new THREE.Group(); scene.add(robotGroup);

// lidar visualisation
const rayGeom = new THREE.BufferGeometry();
rayGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(N_RAYS * 2 * 3), 3));
const rayLines = new THREE.LineSegments(rayGeom, new THREE.LineBasicMaterial({ color: 0xef4444, transparent: true, opacity: 0.18 }));
const hitGeom = new THREE.BufferGeometry();
hitGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(N_RAYS * 3), 3));
const hitPts = new THREE.Points(hitGeom, new THREE.PointsMaterial({ color: 0xdc2626, size: 0.025 }));
scene.add(rayLines, hitPts);
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3000 * 3), 3));
const trailLine = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0x4f46e5 }));
scene.add(trailLine);
const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x7b4fd1 }));
const goalMark = new THREE.Mesh(new THREE.TorusGeometry(0.08, 0.012, 8, 32), new THREE.MeshBasicMaterial({ color: 0x4f46e5 }));
goalMark.visible = false;
scene.add(pathLine, goalMark);
const moverMeshes = [];

// ------------------------------------------------------------------ URDF robot
const urdfLoader = new URDFLoader();
const stl = new STLLoader();
urdfLoader.loadMeshCb = (path, manager, material, done) => {
  stl.load(path, (g) => { g.computeVertexNormals(); done(new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0x9ca3af }))); }, undefined, (e) => done(null, e));
};
let urdf = null, wheelJ = {}, steerBase = {}, wheelAngle = {};
const urdfCache = {};
function loadURDF(variant) {
  return new Promise((resolve, reject) => {
    if (urdfCache[variant]) return resolve(urdfCache[variant]);
    const mgr = new THREE.LoadingManager();
    urdfLoader.manager = mgr;
    let model = null;
    mgr.onLoad = () => {
      model.traverse((o) => {
        if (!o.isMesh) return;
        let p = o, link = null;
        while (p) { if (p.isURDFLink) { link = p.name; break; } p = p.parent; }
        const c = link && link.startsWith('wheel') ? 0x374151 : link === 'lidar_frame' ? 0x111111 : link === 'depth_cam' ? 0x1e293b : 0x1f2937;
        o.material = new THREE.MeshStandardMaterial({ color: c, roughness: 0.55, metalness: 0.15 });
        o.castShadow = true;
      });
      urdfCache[variant] = model;
      resolve(model);
    };
    urdfLoader.load(`../assets/urdf/mentorpi/mentorpi_${variant}.urdf`, (m) => { model = m; }, undefined, reject);
  });
}
async function setRobotModel(variant) {
  $('subTitle').textContent = 'URDF 불러오는 중…';
  const m = await loadURDF(variant);
  robotGroup.clear();
  robotGroup.add(m);
  urdf = m; wheelJ = {}; steerBase = {}; wheelAngle = {};
  for (const [n, j] of Object.entries(m.joints)) if (/^wheel_/.test(n)) { wheelJ[n] = j; wheelAngle[n] = 0; }
  for (const n of ['wheel_lf_Joint', 'wheel_rf_Joint']) if (m.joints[n]) steerBase[n] = m.joints[n].origQuaternion ? m.joints[n].origQuaternion.clone() : m.joints[n].quaternion.clone();
  $('subTitle').textContent = `${world.title} · mentorpi_${variant}.urdf (링크 ${Object.keys(m.links).length}, 조인트 ${Object.keys(m.joints).length})`;
}

// ------------------------------------------------------------------ world meshes
function trackPoints(t) {
  const pts = [], h = t.half, r = t.r;
  for (let x = -h; x <= h + 1e-9; x += 0.05) pts.push([x, -r]);
  for (let a = -Math.PI / 2; a <= Math.PI / 2 + 1e-9; a += Math.PI / 40) pts.push([h + r * Math.cos(a), r * Math.sin(a)]);
  for (let x = h; x >= -h - 1e-9; x -= 0.05) pts.push([x, r - t.wiggle * Math.sin(Math.PI * (x + h) / (2 * h)) ** 2]);
  for (let a = Math.PI / 2; a <= 3 * Math.PI / 2 + 1e-9; a += Math.PI / 40) pts.push([-h + r * Math.cos(a), r * Math.sin(a)]);
  return pts;
}
function buildLineMask(track) {
  const res = 0.01, [x0, y0, x1, y1] = world.bounds;
  const w = Math.ceil((x1 - x0) / res), h = Math.ceil((y1 - y0) / res);
  const data = new Uint8Array(w * h), half = track.width / 2, P = track.points;
  for (let k = 0; k < P.length; k++) {
    const [ax, ay] = P[k], [bx, by] = P[(k + 1) % P.length];
    const ex = bx - ax, ey = by - ay, L2 = ex * ex + ey * ey || 1;
    const i0 = Math.floor((Math.min(ax, bx) - half - x0) / res), i1 = Math.ceil((Math.max(ax, bx) + half - x0) / res);
    const j0 = Math.floor((Math.min(ay, by) - half - y0) / res), j1 = Math.ceil((Math.max(ay, by) + half - y0) / res);
    for (let j = Math.max(0, j0); j <= Math.min(h - 1, j1); j++) for (let i = Math.max(0, i0); i <= Math.min(w - 1, i1); i++) {
      const px = x0 + (i + 0.5) * res, py = y0 + (j + 0.5) * res;
      const t = Math.max(0, Math.min(1, ((px - ax) * ex + (py - ay) * ey) / L2));
      if ((ax + t * ex - px) ** 2 + (ay + t * ey - py) ** 2 <= half * half) data[j * w + i] = 1;
    }
  }
  return { res, x0, y0, w, h, data };
}
const isLine = (x, y) => {
  if (!lineMask) return false;
  const i = Math.floor((x - lineMask.x0) / lineMask.res), j = Math.floor((y - lineMask.y0) / lineMask.res);
  return i >= 0 && j >= 0 && i < lineMask.w && j < lineMask.h && lineMask.data[j * lineMask.w + i] === 1;
};

function floorTexture() {
  const [x0, y0, x1, y1] = world.bounds, ppm = 256;
  const cv = document.createElement('canvas');
  cv.width = Math.round((x1 - x0) * ppm); cv.height = Math.round((y1 - y0) * ppm);
  const c = cv.getContext('2d');
  c.fillStyle = '#e9e4da'; c.fillRect(0, 0, cv.width, cv.height);
  c.strokeStyle = 'rgba(0,0,0,.08)'; c.lineWidth = 2;
  for (let x = 0; x <= cv.width; x += ppm / 2) { c.beginPath(); c.moveTo(x, 0); c.lineTo(x, cv.height); c.stroke(); }
  for (let y = 0; y <= cv.height; y += ppm / 2) { c.beginPath(); c.moveTo(0, y); c.lineTo(cv.width, y); c.stroke(); }
  if (world.track) {
    c.strokeStyle = '#111'; c.lineWidth = world.track.width * ppm; c.lineJoin = 'round';
    c.beginPath();
    world.track.points.forEach(([x, y], i) => { const px = (x - x0) * ppm, py = (y1 - y) * ppm; i ? c.lineTo(px, py) : c.moveTo(px, py); });
    c.closePath(); c.stroke();
  }
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}
function buildWorldMeshes() {
  worldGroup.clear(); moverMeshes.length = 0;
  const [x0, y0, x1, y1] = world.bounds;
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), new THREE.MeshStandardMaterial({ map: floorTexture(), roughness: 0.95 }));
  floor.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0); floor.receiveShadow = true; floor.name = 'floor';
  worldGroup.add(floor);
  const wallMat = new THREE.MeshStandardMaterial({ color: 0xcbd5e1, roughness: 0.8 });
  const boxMat = new THREE.MeshStandardMaterial({ color: 0x8b5e34, roughness: 0.7 });
  const cylMat = new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.6 });
  const H = 0.3, T = 0.04;
  const seg = (ax, ay, bx, by, mat, h = H) => {
    const L = Math.hypot(bx - ax, by - ay);
    const m = new THREE.Mesh(new THREE.BoxGeometry(L + T, T, h), mat);
    m.position.set((ax + bx) / 2, (ay + by) / 2, h / 2); m.rotation.z = Math.atan2(by - ay, bx - ax);
    m.castShadow = m.receiveShadow = true; worldGroup.add(m);
  };
  seg(x0, y0, x1, y0, wallMat); seg(x1, y0, x1, y1, wallMat); seg(x1, y1, x0, y1, wallMat); seg(x0, y1, x0, y0, wallMat);
  for (const [a, b, c, d] of world.walls) seg(a, b, c, d, wallMat);
  for (const [cx, cy, w, h] of world.boxes) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, 0.25), boxMat);
    m.position.set(cx, cy, 0.125); m.castShadow = m.receiveShadow = true; worldGroup.add(m);
  }
  const objs = (world.objects || []);
  for (const [cx, cy, r] of world.cylinders) {
    const o = objs.find((q) => Math.abs(q[1] - cx) < 1e-6 && Math.abs(q[2] - cy) < 1e-6);
    if (!o) { addCylinder(cx, cy, r, cylMat); continue; }
    const col = new THREE.Color(objColor(o[0]));
    const m = /ball|공/.test(o[0])
      ? new THREE.Mesh(new THREE.SphereGeometry(r, 24, 16), new THREE.MeshStandardMaterial({ color: col }))
      : new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.3, 24), new THREE.MeshStandardMaterial({ color: col }));
    if (m.geometry.type === 'CylinderGeometry') { m.rotation.x = Math.PI / 2; m.position.set(cx, cy, 0.15); } else m.position.set(cx, cy, r);
    m.castShadow = true; worldGroup.add(m);
  }
}
function addCylinder(cx, cy, r, mat) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.3, 32), mat || new THREE.MeshStandardMaterial({ color: 0x64748b }));
  m.rotation.x = Math.PI / 2; m.position.set(cx, cy, 0.15); m.castShadow = true; worldGroup.add(m);
  return m;
}

// ------------------------------------------------------------------ setup
async function reset() {
  const name = $('selWorld').value, chassis = $('selChassis').value;
  lsSet({ world: name, chassis });
  world = new World(WORLDS[name], name);
  if (WORLDS[name].track) { world.track = { width: WORLDS[name].track.width, points: trackPoints(WORLDS[name].track), closed: true }; lineMask = buildLineMask(world.track); }
  else { world.track = null; lineMask = null; }
  rng = makeRng(7);
  robot = new Robot(world, chassis, world.start, $('chkNoise').checked, rng);
  simT = 0; scanAcc = 0; ctrlAcc = 0; trail = []; costmap = null;
  stopAll();
  ranges = scan(world, [robot.x, robot.y, robot.yaw], false, rng);
  buildWorldMeshes();
  const [x0, y0, x1, y1] = world.bounds;
  controls.target.set(robot.x, robot.y, 0.1);
  camera.position.set(robot.x - 1.2, robot.y - 1.6, 1.3);
  sun.target.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0); scene.add(sun.target);
  await setRobotModel(chassis);
}
function stopAll() {
  mode = 'manual'; lidarApp.reset(); lineF.reset(); nav = null;
  robot && robot.setCmd(0, 0, 0);
  document.querySelectorAll('[data-lmode]').forEach((b) => b.classList.remove('on'));
  goalMark.visible = false; pathLine.geometry.setFromPoints([]);
}
function info(t) { $('info').textContent = t; }

// ------------------------------------------------------------------ UI
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
  document.querySelectorAll('.tab-body').forEach((x) => x.classList.toggle('active', x.dataset.body === b.dataset.tab));
  if (b.dataset.tab === 'line' && !lineMask) { $('selWorld').value = 'track'; reset(); info('라인 트랙 월드로 바꿨습니다.'); }
}));
$('selWorld').addEventListener('change', reset);
$('selChassis').addEventListener('change', reset);
$('btnReset').addEventListener('click', reset);
$('chkNoise').addEventListener('change', () => { robot.noise = $('chkNoise').checked; });
const bindRange = (id, out, fn) => { const r = $(id); const upd = () => { $(out).textContent = Number(r.value).toFixed(2); fn && fn(Number(r.value)); }; r.addEventListener('input', upd); upd(); };
bindRange('rV', 'oV'); bindRange('rW', 'oW');
bindRange('rTh', 'oTh', (v) => { lidarApp.threshold = v; });
bindRange('rSp', 'oSp', (v) => { lidarApp.speed = v; });
document.querySelectorAll('[data-lmode]').forEach((b) => b.addEventListener('click', () => {
  stopAll(); mode = b.dataset.lmode; b.classList.add('on');
}));
$('btnStop').addEventListener('click', () => { stopAll(); info('정지'); });
$('btnMover').addEventListener('click', () => {
  const th = robot.yaw;
  world.movers.push([robot.x + 0.45 * Math.cos(th), robot.y + 0.45 * Math.sin(th), 0.08 * Math.cos(th + 1.2), 0.08 * Math.sin(th + 1.2), 0.08]);
  moverMeshes.push(addCylinder(0, 0, 0.08, new THREE.MeshStandardMaterial({ color: 0x7b4fd1 })));
  info('로봇 앞에 움직이는 목표(보라)를 놓았습니다.');
});
$('btnLineGo').addEventListener('click', async () => { if (!lineMask) { $('selWorld').value = 'track'; await reset(); } stopAll(); mode = 'line'; lineF.reset(); $('selCam').value = 'follow'; });
$('btnLineStop').addEventListener('click', stopAll);
$('btnLineObs').addEventListener('click', () => {
  if (!world.track) return;
  const pts = world.track.points; let best = 0, bd = Infinity;
  pts.forEach(([x, y], k) => { const d = Math.hypot(x - robot.x, y - robot.y); if (d > 1.2 && d < bd) { bd = d; best = k; } });
  world.cylinders.push([pts[best][0], pts[best][1], 0.07]);
  addCylinder(pts[best][0], pts[best][1], 0.07);
  info('트랙 위에 장애물을 놓았습니다 — 0.4 m 앞에서 멈추는지 보세요.');
});
$('btnNavCancel').addEventListener('click', () => { stopAll(); info('내비게이션 취소'); });

window.addEventListener('keydown', (e) => {
  if (e.target.closest('input, select, textarea')) return;
  const k = e.key.toLowerCase();
  if ('wsadqe'.includes(k) && k.length === 1) { keys.add(k); if (mode !== 'manual') stopAll(); e.preventDefault(); }
  if (k === ' ') { stopAll(); keys.clear(); e.preventDefault(); }
  document.querySelectorAll('.keys kbd').forEach((el) => el.classList.toggle('on', keys.has(el.dataset.k)));
});
window.addEventListener('keyup', (e) => { keys.delete(e.key.toLowerCase()); document.querySelectorAll('.keys kbd').forEach((el) => el.classList.toggle('on', keys.has(el.dataset.k))); });
window.addEventListener('blur', () => keys.clear());

// click on the floor → navigation goal
const raycaster = new THREE.Raycaster();
let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 5) return;     // it was a drag (orbit)
  if (!$('chkGoalClick').checked || !document.querySelector('[data-body="nav"]').classList.contains('active')) return;
  const r = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
  const floor = worldGroup.getObjectByName('floor');
  const hit = raycaster.intersectObject(floor)[0];
  if (hit) startNav([hit.point.x, hit.point.y, null]);
});
function startNav(goal) {
  stopAll();
  if (!costmap) costmap = new Costmap(new Grid(world.bounds, 0.05).fromWorld(world), NAV_DEFAULTS);
  const r = astar(costmap, [robot.x, robot.y, robot.yaw], goal);
  if (!r.path) { info('경로 계획 실패: ' + r.reason); return; }
  nav = { goal, path: r.path, progress: 0, t0: simT };
  mode = 'nav';
  pathLine.geometry.setFromPoints(r.path.map(([x, y]) => new THREE.Vector3(x, y, 0.01)));
  goalMark.position.set(goal[0], goal[1], 0.01); goalMark.visible = true;
  info(`A*: ${r.expanded}칸 탐색 → 경로 ${r.path.length}점. Pure Pursuit로 주행합니다.`);
}

// ------------------------------------------------------------------ control @10 Hz
function control() {
  if (mode === 'manual') {
    const vmax = +$('rV').value, wmax = +$('rW').value;
    let vx = 0, vy = 0, wz = 0;
    if (keys.has('w')) vx += vmax; if (keys.has('s')) vx -= vmax;
    if (keys.has('a')) wz += wmax; if (keys.has('d')) wz -= wmax;
    if (keys.has('q')) vy += vmax; if (keys.has('e')) vy -= vmax;
    robot.setCmd(vx, vy, wz, $('chkUnlimited').checked ? PHYS_LIMIT : APP_LIMIT);
  } else if (['avoid', 'follow', 'guard'].includes(mode)) {
    const c = lidarApp.step(mode, ranges, simT, robot.chassis);
    if (c) robot.setCmd(c[0], c[1], c[2], PHYS_LIMIT);
    info(lidarApp.info);
  } else if (mode === 'line') {
    const c = lineF.step([robot.x, robot.y, robot.yaw], isLine, ranges, robot.chassis);
    robot.setCmd(c[0], c[1], c[2], PHYS_LIMIT);
    $('lineStat').textContent = lineF.info; info('[라인] ' + lineF.info);
  } else if (mode === 'nav' && nav) {
    const pose = [robot.x, robot.y, robot.yaw];
    const d = Math.hypot(nav.goal[0] - pose[0], nav.goal[1] - pose[1]);
    if (d < NAV_DEFAULTS.xyTol) { robot.setCmd(0, 0, 0); info(`목표 도착 (${(simT - nav.t0).toFixed(1)} s)`); const g = nav; stopAll(); goalMark.position.set(g.goal[0], g.goal[1], 0.01); return; }
    const res = purePursuit(nav, pose, NAV_DEFAULTS, robot.chassis);
    robot.setCmd(res.cmd[0], res.cmd[1], res.cmd[2], PHYS_LIMIT);
    $('navStat').textContent = `남은 거리 ${d.toFixed(2)} m · v=${res.cmd[0].toFixed(2)} ω=${res.cmd[2].toFixed(2)}`;
  }
}

function advance(seconds) {
  let budget = seconds;
  while (budget > 1e-9) {
    const h = Math.min(DT, budget); budget -= h;
    robot.step(h); world.stepMovers(h); simT += h; scanAcc += h; ctrlAcc += h;
    if (scanAcc >= 0.1 - 1e-9) {
      scanAcc -= 0.1;
      ranges = scan(world, [robot.x, robot.y, robot.yaw], robot.noise, rng);
      trail.push([robot.x, robot.y]); if (trail.length > 3000) trail.shift();
    }
    if (ctrlAcc >= 0.1 - 1e-9) { ctrlAcc -= 0.1; control(); }
  }
}

// ------------------------------------------------------------------ sync 3D
function syncRobot(dt) {
  robotGroup.position.set(robot.x, robot.y, 0);
  robotGroup.rotation.set(0, 0, robot.yaw);
  if (!urdf) return;
  const w = robot.wheels(), rad = robot.p.wheel_diameter / 2;
  // wheel surface speeds: mecanum lin = [LF, LB, RF, RB]; ackermann lin = [vl, vr]
  const s = robot.chassis === 'mecanum'
    ? { wheel_lf_Joint: w.lin[0], wheel_lb_Joint: w.lin[1], wheel_rf_Joint: w.lin[2], wheel_rb_Joint: w.lin[3] }
    : { wheel_lf_Joint: w.lin[0], wheel_lb_Joint: w.lin[0], wheel_rf_Joint: w.lin[1], wheel_rb_Joint: w.lin[1] };
  for (const [n, j] of Object.entries(wheelJ)) {
    wheelAngle[n] += (s[n] || 0) / rad * dt;
    j.setJointValue(wheelAngle[n]);
    if (robot.chassis === 'ackermann' && steerBase[n]) {
      const qs = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), robot.steer);
      const qr = new THREE.Quaternion().setFromAxisAngle(j.axis, wheelAngle[n]);
      j.quaternion.copy(steerBase[n]).multiply(qs).multiply(qr);
    }
  }
}
function syncLidar() {
  const show = $('chkRays').checked;
  rayLines.visible = hitPts.visible = show;
  if (!show) return;
  const rp = rayGeom.attributes.position.array, hp = hitGeom.attributes.position.array;
  const lx = robot.x - 0.012 * Math.cos(robot.yaw), ly = robot.y - 0.012 * Math.sin(robot.yaw);
  let n = 0;
  for (let i = 0; i < N_RAYS; i++) {
    const r = ranges[i];
    const a = robot.yaw + rayAngle(i), rr = isFinite(r) ? r : 0;
    rp.set([lx, ly, LIDAR_Z, lx + rr * Math.cos(a), ly + rr * Math.sin(a), LIDAR_Z], i * 6);
    if (isFinite(r)) { hp.set([lx + r * Math.cos(a), ly + r * Math.sin(a), LIDAR_Z], n * 3); n++; }
  }
  rayGeom.attributes.position.needsUpdate = true;
  hitGeom.attributes.position.needsUpdate = true;
  hitGeom.setDrawRange(0, n);
}
function syncTrail() {
  trailLine.visible = $('chkTrail').checked;
  const a = trailGeom.attributes.position.array;
  trail.forEach(([x, y], i) => a.set([x, y, 0.005], i * 3));
  trailGeom.setDrawRange(0, trail.length);
  trailGeom.attributes.position.needsUpdate = true;
  world.movers.forEach((m, i) => { if (moverMeshes[i]) moverMeshes[i].position.set(m[0], m[1], 0.15); });
}
function syncCameras() {
  const c = Math.cos(robot.yaw), s = Math.sin(robot.yaw);
  const view = $('selCam').value;
  controls.enabled = view === 'orbit' || view === 'top';
  if (view === 'follow') {
    camera.position.lerp(new THREE.Vector3(robot.x - 0.7 * c, robot.y - 0.7 * s, 0.45), 0.12);
    camera.lookAt(robot.x + 0.3 * c, robot.y + 0.3 * s, 0.05);
  } else if (view === 'fpv') {
    camera.position.set(robot.x + CAM_POS[0] * c, robot.y + CAM_POS[0] * s, CAM_POS[2]);
    camera.lookAt(robot.x + c, robot.y + s, 0.05);
  } else if (view === 'top' && !syncCameras.topSet) {
    const [x0, y0, x1, y1] = world.bounds;
    camera.position.set((x0 + x1) / 2, (y0 + y1) / 2 - 0.01, Math.max(x1 - x0, y1 - y0) * 1.1);
    controls.target.set((x0 + x1) / 2, (y0 + y1) / 2, 0);
    syncCameras.topSet = true;
  }
  if (view !== 'top') syncCameras.topSet = false;
  // robot camera (depth_cam), pitched down for line following
  const pitchDown = mode === 'line' || lineMask ? 0.5 : 0.15;
  pipCam.position.set(robot.x + CAM_POS[0] * c, robot.y + CAM_POS[0] * s, CAM_POS[2]);
  pipCam.lookAt(robot.x + (CAM_POS[0] + Math.cos(pitchDown)) * c, robot.y + (CAM_POS[0] + Math.cos(pitchDown)) * s, CAM_POS[2] - Math.sin(pitchDown));
}

function hud() {
  const deg = (r) => (r * 180 / Math.PI).toFixed(1) + '°';
  $('hud').innerHTML = `<div><b>t</b>${simT.toFixed(1)} s · ${mode}</div>` +
    `<div><b>실제</b>(${robot.x.toFixed(2)}, ${robot.y.toFixed(2)}, ${deg(robot.yaw)})</div>` +
    `<div><b>odom</b>(${robot.ox.toFixed(2)}, ${robot.oy.toFixed(2)}, ${deg(robot.oyaw)})</div>` +
    `<div><b>cmd</b>(${robot.cmd.map((v) => v.toFixed(2)).join(', ')})</div>` +
    `<div><b>충돌</b>${robot.collisions} · ${robot.distance.toFixed(2)} m</div>`;
  const w = robot.wheels();
  $('wheels').innerHTML = robot.chassis === 'mecanum'
    ? ['M1 왼앞', 'M2 왼뒤', 'M3 오앞', 'M4 오뒤'].map((n, i) => `<div><span>${n}</span>${w.rps[i] >= 0 ? '+' : ''}${w.rps[i].toFixed(2)} rps</div>`).join('')
    : `<div><span>조향각 δ</span>${w.steerDeg.toFixed(1)}°</div><div><span>서보 펄스</span>${w.servo} µs</div><div><span>M2 왼뒤</span>${w.rps[1].toFixed(2)} rps</div><div><span>M4 오뒤</span>${w.rps[3].toFixed(2)} rps</div>`;
}

function resize() {
  const r = host.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(1, r.height); camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(host);

let last = 0, frameNo = 0;
function frame(ts) {
  const dt = last ? Math.min(0.05, (ts - last) / 1000) : 0; last = ts;
  if (REPLAY) { if (rp.res && world) replayTick(dt); }
  else if (world && robot) {
    advance(dt * Number($('selSpeed').value));
    render(dt * Number($('selSpeed').value));
  }
  requestAnimationFrame(frame);
}
function render(dt = 0) {
  syncRobot(dt); syncLidar(); syncTrail(); syncCameras(); hud();
  if ($('selCam').value !== 'follow' && $('selCam').value !== 'fpv') controls.update();
  renderer.render(scene, camera);
  if (++frameNo % 2 === 0) {
    rayLines.visible = hitPts.visible = false; trailLine.visible = false;
    pipRenderer.render(scene, pipCam);
    rayLines.visible = hitPts.visible = $('chkRays').checked; trailLine.visible = $('chkTrail').checked;
  }
}

// test/automation hook
window.__sim3d = {
  advance: (s) => { advance(s); render(0.02); },
  state: () => ({ t: simT, mode, pose: [robot.x, robot.y, robot.yaw], collisions: robot.collisions, distance: robot.distance, urdf: urdf ? Object.keys(urdf.joints).length : 0, info: $('info').textContent }),
  startNav, setMode: (m) => { stopAll(); mode = m; }, reset,
};


// ------------------------------------------------------------------ embed / replay mode
// ?embed=1       : live simulator without the site header (used inside the lesson side panel)
// ?embed=replay  : plays a finished Playground / lesson run (postMessage {type:'mp-replay', result})
const EMBED = new URLSearchParams(location.search).get('embed');
const REPLAY = EMBED === 'replay';
if (EMBED) document.body.classList.add('embed');
if (REPLAY) document.body.classList.add('embed-replay');
const rp = { res: null, i: 0, t: 0, playing: false };
function replayBar() {
  const bar = document.createElement('div');
  bar.className = 'rp-bar';
  bar.innerHTML = `<button type="button" class="s-btn" id="rpPlay">&#9654;</button><span id="rpT">대기 중</span>
    <input type="range" id="rpSeek" min="0" max="0" value="0" aria-label="재생 위치">
    <select id="rpSpeed" aria-label="재생 속도"><option value="1">1×</option><option value="2" selected>2×</option><option value="4">4×</option></select>
    <select id="rpCam" aria-label="시점"><option value="follow">따라가기</option><option value="orbit">자유 궤도</option><option value="top">위에서</option><option value="fpv">1인칭</option></select>`;
  $('stage').appendChild(bar);
  $('rpPlay').addEventListener('click', () => {
    if (!rp.res) return;
    if (rp.playing) { rp.playing = false; $('rpPlay').innerHTML = '&#9654;'; return; }
    if (rp.i >= rp.res.frames.length - 1) { rp.i = 0; rp.t = 0; trail = []; }
    rp.playing = true; $('rpPlay').innerHTML = '&#10074;&#10074;';
  });
  $('rpSeek').addEventListener('input', (e) => { rp.playing = false; $('rpPlay').innerHTML = '&#9654;'; setReplayFrame(Number(e.target.value)); });
  $('rpCam').addEventListener('change', () => { $('selCam').value = $('rpCam').value; });
  $('info').textContent = '코드를 실행하면 결과가 3D로 재생됩니다.';
}
async function loadReplay(res) {
  rp.res = res; rp.i = 0; rp.t = 0; rp.playing = true;
  const spec = { title: res.world.title, bounds: res.world.bounds, walls: res.world.walls || [], boxes: res.world.boxes || [],
    cylinders: res.world.cylinders || [], start: res.start };
  world = new World(spec, res.world.name || 'replay');
  world.objects = res.world.objects || [];
  const wspec = WORLDS && WORLDS[res.world.name];
  if (wspec && wspec.track) { world.track = { width: wspec.track.width, points: trackPoints(wspec.track), closed: true }; }
  rng = makeRng(7);
  robot = new Robot(world, res.chassis, res.start, false, rng);
  trail = []; simT = 0; mode = 'replay';
  buildWorldMeshes();
  const [x0, y0, x1, y1] = world.bounds;
  sun.target.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0); scene.add(sun.target);
  controls.target.set(res.start[0], res.start[1], 0.1);
  camera.position.set(res.start[0] - 1.2, res.start[1] - 1.6, 1.3);
  $('selCam').value = $('rpCam').value;
  await setRobotModel(res.chassis);
  $('rpSeek').max = Math.max(0, res.frames.length - 1);
  $('rpPlay').innerHTML = '&#10074;&#10074;';
  $('info').textContent = `${res.world.title} · ${res.chassis} · ${res.frames.length} 프레임`;
  setReplayFrame(0);
}
function setReplayFrame(i) {
  const fr = rp.res.frames;
  rp.i = Math.max(0, Math.min(i, fr.length - 1));
  const f = fr[rp.i];
  rp.t = f.t - fr[0].t;
  [robot.x, robot.y, robot.yaw] = f.gt;
  [robot.ox, robot.oy, robot.oyaw] = f.od;
  robot.cmd = f.cmd.slice();
  robot.v = f.cmd.slice();
  robot.steer = f.steer || 0;
  simT = f.t;
  if (f.scan && f.scan.length) {
    const n = f.scan.length, k = Math.round(N_RAYS / n);
    ranges = new Array(N_RAYS).fill(Infinity);
    f.scan.forEach((r, j) => { ranges[j * k] = r == null ? Infinity : r; });
  }
  // movers (walkers / targets) — create meshes on demand
  world.movers = (f.movers || []).map(([x, y, r]) => [x, y, 0, 0, r]);
  while (moverMeshes.length < world.movers.length) moverMeshes.push(addCylinder(0, 0, world.movers[moverMeshes.length][4], new THREE.MeshStandardMaterial({ color: 0x7b4fd1 })));
  trail = fr.slice(0, rp.i + 1).map((q) => [q.gt[0], q.gt[1]]).slice(-3000);
  $('rpSeek').value = rp.i;
  $('rpT').textContent = `${f.t.toFixed(1)} / ${fr[fr.length - 1].t.toFixed(1)} s`;
}
function replayTick(dt) {
  const fr = rp.res.frames;
  if (rp.playing) {
    rp.t += dt * Number($('rpSpeed').value);
    let i = rp.i;
    while (i < fr.length - 1 && fr[i + 1].t - fr[0].t <= rp.t) i++;
    if (i !== rp.i) setReplayFrame(i);
    if (rp.i >= fr.length - 1) { rp.playing = false; $('rpPlay').innerHTML = '&#8634;'; }
  }
  render(rp.playing ? dt * Number($('rpSpeed').value) : 0);
}
window.addEventListener('message', (ev) => {
  const m = ev.data || {};
  if (m.type === 'mp-replay' && m.result) loadReplay(m.result).catch((e) => { $('info').textContent = '재생 실패: ' + e.message; });
});

(async () => {
  WORLDS = await (await fetch('../python/mentorpi_sim/worlds.json')).json();
  delete WORLDS._comment;
  $('selWorld').innerHTML = Object.entries(WORLDS).map(([k, w]) => `<option value="${k}">${w.title}</option>`).join('');
  const saved = lsGet(), q = new URLSearchParams(location.search);
  $('selWorld').value = q.get('world') || saved.world || 'room';
  $('selChassis').value = q.get('chassis') || saved.chassis || 'mecanum';
  resize();
  if (REPLAY) {
    replayBar();
    requestAnimationFrame(frame);
    if (window.parent !== window) window.parent.postMessage({ type: 'mp3d-ready' }, '*');
    return;
  }
  await reset();
  requestAnimationFrame(frame);
})().catch((e) => { $('subTitle').textContent = '로드 실패: ' + e.message; console.error(e); });
