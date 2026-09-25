// urdf-viewer.js — MentorPi URDF viewer (tools/urdf-viewer.html)
// three.js 0.186.0 + urdf-loader 0.13.1 (jsdelivr importmap). Meshes: decimated from mentorpi_description.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import URDFLoader from 'urdf-loader';

const $ = (id) => document.getElementById(id);
const BASE = '../assets/urdf/mentorpi/';
const GEOM = {
  mecanum: { wheelbase: 0.1368, track: 0.1446, d: 0.065 },
  ackermann: { wheelbase: 0.145, track: 0.133, d: 0.067 },
};
const COLORS = { base_link: 0x1f2937, wheel: 0x374151, lidar_frame: 0x111111, depth_cam: 0x1e293b, default: 0x9ca3af };
const AXES_FRAMES = ['base_footprint', 'lidar_frame', 'depth_cam', 'imu_link'];

function status(t, cls) { const s = $('viewerStatus'); s.textContent = t; s.className = 'vstat' + (cls ? ' ' + cls : ''); }

// ------------------------------------------------------------------ three setup
const host = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
host.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.005, 20);
camera.up.set(0, 0, 1);
camera.position.set(0.32, -0.34, 0.28);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.07);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(0.6, -0.8, 1.2); scene.add(sun);
const grid = new THREE.GridHelper(1.0, 20, 0x888888, 0xcccccc);
grid.rotation.x = Math.PI / 2; scene.add(grid);
const world = new THREE.Group(); scene.add(world);

function resize() {
  const r = host.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(1, r.height); camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(host);

// ------------------------------------------------------------------ model
let robot = null, variant = 'mecanum', wheels = {}, steerBase = {}, axesHelpers = [];
const loader = new URDFLoader();
const stl = new STLLoader();
loader.loadMeshCb = (path, manager, material, done) => {   // urdf-loader 0.13 signature
  stl.load(path, (geom) => {
    geom.computeVertexNormals();
    done(new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: COLORS.default, roughness: 0.6, metalness: 0.1 })));
  }, undefined, (err) => { console.warn(err); done(null, err); });
};

function colorize() {
  robot.traverse((o) => {
    if (!o.isMesh) return;
    let p = o; let link = null;
    while (p) { if (p.isURDFLink) { link = p.name; break; } p = p.parent; }
    const c = link && link.startsWith('wheel') ? COLORS.wheel : COLORS[link] ?? COLORS.default;
    o.material = new THREE.MeshStandardMaterial({ color: c, roughness: 0.55, metalness: 0.15, transparent: true, opacity: 1 });
    o.userData.link = link;
  });
  applyGhost();
}
function applyGhost() {
  if (!robot) return;
  robot.traverse((o) => { if (o.isMesh) o.material.opacity = $('chkGhost').checked && o.userData.link === 'base_link' ? 0.25 : 1; });
}

function load(v) {
  variant = v;
  status('모델 불러오는 중…');
  if (robot) { world.remove(robot); robot = null; }
  axesHelpers.forEach((a) => a.parent && a.parent.remove(a)); axesHelpers = [];
  loader.packages = '';
  loader.manager.onLoad = () => { colorize(); status(`${v === 'mecanum' ? '메카넘' : '애커만'} · 링크 ${Object.keys(robot.links).length} · 조인트 ${Object.keys(robot.joints).length}`, 'ok'); updateTF(); };
  loader.load(BASE + `mentorpi_${v}.urdf`, (r) => {
    robot = r;
    world.add(robot);
    window.__urdf = { robot, scene, camera, THREE };
    wheels = {}; steerBase = {};
    for (const [name, j] of Object.entries(robot.joints)) if (/^wheel_/.test(name)) wheels[name] = { joint: j, angle: 0 };
    for (const n of ['wheel_lf_Joint', 'wheel_rf_Joint']) if (robot.joints[n]) steerBase[n] = robot.joints[n].quaternion.clone();
    for (const f of AXES_FRAMES) { const l = robot.links[f]; if (l) { const a = new THREE.AxesHelper(0.05); l.add(a); axesHelpers.push(a); } }
    applyAxes();
    $('tree').textContent = tree(robot);
    updateTF();
  }, undefined, (e) => status('불러오기 실패: ' + e, 'bad'));
}
function applyAxes() { axesHelpers.forEach((a) => { a.visible = $('chkAxes').checked; }); }

function tree(r) {
  const out = [];
  const walk = (link, depth) => {
    out.push('  '.repeat(depth) + (depth ? '└ ' : '') + link.name);
    for (const c of link.children) {
      if (!c.isURDFJoint) continue;
      const child = c.children.find((x) => x.isURDFLink);
      if (!child) continue;
      out.push('  '.repeat(depth + 1) + `(${c.jointType}) ${c.name}`);
      walk(child, depth + 1);
    }
  };
  const roots = Object.values(r.links).filter((l) => !l.parent || !l.parent.isURDFJoint);
  const root = roots.find((l) => l.name === 'base_footprint') || roots[0] || r;
  walk(root, 0);
  return out.join('\n');
}

function updateTF() {
  if (!robot || !robot.links.base_footprint) return;
  robot.updateMatrixWorld(true);
  const inv = new THREE.Matrix4().copy(robot.links.base_footprint.matrixWorld).invert();
  const rows = [];
  for (const [name, l] of Object.entries(robot.links)) {
    const m = new THREE.Matrix4().multiplyMatrices(inv, l.matrixWorld);
    const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3();
    m.decompose(p, q, s);
    const yaw = new THREE.Euler().setFromQuaternion(q, 'ZYX').z * 180 / Math.PI;
    rows.push(`<tr class="${AXES_FRAMES.includes(name) ? 'hl' : ''}"><td>${name}</td><td>${p.x.toFixed(4)}</td><td>${p.y.toFixed(4)}</td><td>${p.z.toFixed(4)}</td><td>${yaw.toFixed(1)}</td></tr>`);
  }
  $('tfBody').innerHTML = rows.join('');
}

// ------------------------------------------------------------------ kinematics → wheel spin
function wheelRates() {
  const vx = Number($('vx').value), vy = variant === 'mecanum' ? Number($('vy').value) : 0, wz = Number($('wz').value);
  const g = GEOM[variant], circ = Math.PI * g.d;
  if (variant === 'mecanum') {
    const k = (g.wheelbase + g.track) / 2;
    const m1 = vx - vy - wz * k, m2 = vx + vy - wz * k, m3 = vx + vy + wz * k, m4 = vx - vy + wz * k;
    // wheel surface speed (+ = rolling forward) per link; motor rps uses MentorPi sign convention [-m1,-m2,m3,m4]
    return { lf: m1, lb: m2, rf: m3, rb: m4, rps: [-m1 / circ, -m2 / circ, m3 / circ, m4 / circ], steer: 0 };
  }
  let steer = 0;
  if (Math.abs(vx) > 1e-6 && Math.abs(wz) > 1e-6) steer = Math.max(-29, Math.min(29, Math.atan(g.wheelbase * wz / vx) * 180 / Math.PI));
  const wzEff = Math.abs(vx) > 1e-6 ? vx * Math.tan(steer * Math.PI / 180) / g.wheelbase : 0;
  const vl = vx - wzEff * g.track / 2, vr = vx + wzEff * g.track / 2;
  return { lf: vl, lb: vl, rf: vr, rb: vr, rps: [0, vl / circ, 0, -vr / circ], steer, servo: Math.round(1500 + 2000 * (-steer) / 180) };
}
function showRates(r) {
  const cell = (k, v) => `<div><span>${k}</span><b>${v}</b></div>`;
  let html = variant === 'mecanum'
    ? ['M1 왼앞', 'M2 왼뒤', 'M3 오앞', 'M4 오뒤'].map((n, i) => cell(n, (r.rps[i] >= 0 ? '+' : '') + r.rps[i].toFixed(2) + ' rps')).join('')
    : cell('조향각 δ', r.steer.toFixed(1) + '°') + cell('서보 펄스', r.servo + ' µs') + cell('M2 왼뒤', r.rps[1].toFixed(2) + ' rps') + cell('M4 오뒤', r.rps[3].toFixed(2) + ' rps');
  $('wheelKv').innerHTML = html;
}
['vx', 'vy', 'wz'].forEach((id) => $(id).addEventListener('input', () => {
  $('o' + id[0].toUpperCase() + id.slice(1)).textContent = Number($(id).value).toFixed(2);
}));
$('btnZero').addEventListener('click', () => { ['vx', 'vy', 'wz'].forEach((id) => { $(id).value = 0; $(id).dispatchEvent(new Event('input')); }); });
$('variant').addEventListener('change', () => { load($('variant').value); $('vy').disabled = $('variant').value !== 'mecanum'; });
$('chkAxes').addEventListener('change', applyAxes);
$('chkGhost').addEventListener('change', applyGhost);
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
  document.querySelectorAll('.tab-body').forEach((x) => x.classList.toggle('active', x.dataset.body === b.dataset.tab));
  updateTF();
}));

// hover → link name
const ray = new THREE.Raycaster(), mouse = new THREE.Vector2();
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!robot) return;
  const r = renderer.domElement.getBoundingClientRect();
  mouse.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObject(robot, true)[0];
  $('hoverLink').textContent = hit ? hit.object.userData.link || '—' : '—';
});

let last = performance.now();
function frame(now) {
  const dt = Math.min(0.05, (now - last) / 1000); last = now;
  if (robot) {
    const r = wheelRates();
    showRates(r);
    const rad = GEOM[variant].d / 2;
    for (const [name, w] of Object.entries(wheels)) {
      const side = name.includes('_l') ? (name.includes('lf') ? 'lf' : 'lb') : (name.includes('rf') ? 'rf' : 'rb');
      w.angle += (r[side] / rad) * dt;           // axis +y: positive angle = rolling forward
      w.joint.setJointValue(w.angle);
      if (variant === 'ackermann' && steerBase[name]) {
        const qs = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), r.steer * Math.PI / 180);
        const qr = new THREE.Quaternion().setFromAxisAngle(w.joint.axis, w.angle);
        w.joint.quaternion.copy(steerBase[name]).multiply(qs).multiply(qr);
      }
    }
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
resize();
load(new URLSearchParams(location.search).get('variant') === 'ackermann' ? 'ackermann' : 'mecanum');
if (new URLSearchParams(location.search).get('variant') === 'ackermann') $('variant').value = 'ackermann';
requestAnimationFrame(frame);
