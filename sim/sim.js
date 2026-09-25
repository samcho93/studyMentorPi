// sim.js — MentorPi 2D simulator: manual drive, lidar app modes, occupancy-grid SLAM,
// A* + Pure Pursuit/DWA navigation with AMCL, line following on a track.
import { View, themeColors, drawWorld, drawRobot, drawScan, drawPath, drawMovers, drawGoal, drawGrid } from '../assets/js/world2d.js';
import { World, Robot, makeRng, scan, rayAngle, N_RAYS, APP_LIMIT, PHYS_LIMIT, wrap } from './physics.js';
import { LidarApp, LineFollower, CAM, camPixelToWorld } from './behaviors.js';
import { Grid } from './grid.js';
import { Costmap, astar, purePursuit, dwa, NAV_DEFAULTS } from './nav.js';
import { AMCL } from './amcl.js';

const $ = (id) => document.getElementById(id);
const DT = 0.02;
const LS = 'studymentorpi.sim.v1';
const lsGet = () => { try { return JSON.parse(localStorage.getItem(LS) || '{}'); } catch (e) { return {}; } };
const lsSet = (o) => { try { localStorage.setItem(LS, JSON.stringify({ ...lsGet(), ...o })); } catch (e) { /* ignore */ } };

let WORLDS = null;
let world, robot, rng, C = themeColors();
let simT = 0, scanAcc = 0, ranges = new Array(N_RAYS).fill(Infinity), paused = false;
let trail = [], odoTrail = [];
let mode = 'manual';             // manual | avoid | follow | guard | nav | line | explore
const keys = new Set();
let joy = [0, 0];
const lidarApp = new LidarApp();
const lineF = new LineFollower();
let lineMask = null;             // {res,x0,y0,w,h,data}
let slam = null, slamOn = false;
let costmap = null, nav = null, amcl = null, amclEst = null;
let goalDrag = null, clickMode = null;   // 'goal' | 'pose' | 'place'
let waypointMode = false, waypoints = [];
let dwaViz = null;

const view = new View($('cv'));

// ------------------------------------------------------------------ setup
async function boot() {
  WORLDS = await (await fetch('../python/mentorpi_sim/worlds.json')).json();
  delete WORLDS._comment;
  const sel = $('selWorld');
  sel.innerHTML = Object.entries(WORLDS).map(([k, w]) => `<option value="${k}">${w.title}</option>`).join('');
  const saved = lsGet();
  const q = new URLSearchParams(location.search);
  sel.value = q.get('world') || saved.world || 'room';
  $('selChassis').value = q.get('chassis') || saved.chassis || 'mecanum';
  const tab = q.get('tab');
  reset();
  if (tab) showTab(tab);
  if (q.get('mode')) startMode(q.get('mode'));
  requestAnimationFrame(loop);
}

function trackPoints(t) {
  const pts = [], h = t.half, r = t.r;
  for (let x = -h; x <= h + 1e-9; x += 0.05) pts.push([x, -r]);
  for (let a = -Math.PI / 2; a <= Math.PI / 2 + 1e-9; a += Math.PI / 40) pts.push([h + r * Math.cos(a), r * Math.sin(a)]);
  for (let x = h; x >= -h - 1e-9; x -= 0.05) pts.push([x, r - t.wiggle * Math.sin(Math.PI * (x + h) / (2 * h)) ** 2 * 1.0 * (Math.sin(Math.PI * (x + h) / h) > -2 ? 1 : 1)]);
  for (let a = Math.PI / 2; a <= 3 * Math.PI / 2 + 1e-9; a += Math.PI / 40) pts.push([-h + r * Math.cos(a), r * Math.sin(a)]);
  return pts;
}
function buildLineMask(track) {
  const res = 0.01, [x0, y0, x1, y1] = world.bounds;
  const w = Math.ceil((x1 - x0) / res), h = Math.ceil((y1 - y0) / res);
  const data = new Uint8Array(w * h), half = track.width / 2, P = track.points;
  for (let k = 0; k < P.length - 1 + (track.closed ? 1 : 0); k++) {
    const [ax, ay] = P[k], [bx, by] = P[(k + 1) % P.length];
    const i0 = Math.floor((Math.min(ax, bx) - half - x0) / res), i1 = Math.ceil((Math.max(ax, bx) + half - x0) / res);
    const j0 = Math.floor((Math.min(ay, by) - half - y0) / res), j1 = Math.ceil((Math.max(ay, by) + half - y0) / res);
    const ex = bx - ax, ey = by - ay, L2 = ex * ex + ey * ey || 1;
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

function reset() {
  const name = $('selWorld').value;
  const chassis = $('selChassis').value;
  lsSet({ world: name, chassis });
  world = new World(WORLDS[name], name);
  if (WORLDS[name].track) {
    world.track = { width: WORLDS[name].track.width, points: trackPoints(WORLDS[name].track), closed: true };
    lineMask = buildLineMask(world.track);
  } else { world.track = null; lineMask = null; }
  rng = makeRng(7);
  robot = new Robot(world, chassis, world.start, $('chkNoise').checked, rng);
  simT = 0; scanAcc = 0; trail = []; odoTrail = [];
  ranges = scan(world, [robot.x, robot.y, robot.yaw], false, rng);
  stopAll();
  slam = new Grid(world.bounds, Number($('selRes').value));
  costmap = null; amcl = null; amclEst = null; nav = null; waypoints = [];
  view.resize(); view.fit(world.bounds);
  $('subTitle').textContent = `${world.title} · ${chassis === 'mecanum' ? '메카넘' : '애커만'}`;
  $('camBox').hidden = !lineMask;
  if ($('selLoc').value === 'amcl') ensureAmcl();
}

function stopAll() {
  mode = 'manual';
  lidarApp.reset(); lineF.reset();
  nav = null; dwaViz = null;
  robot && robot.setCmd(0, 0, 0);
  document.querySelectorAll('[data-lmode]').forEach((b) => b.classList.remove('on'));
  $('btnExplore').classList.remove('on');
}

// ------------------------------------------------------------------ tabs / UI
function showTab(name) {
  document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tab-body').forEach((b) => b.classList.toggle('active', b.dataset.body === name));
  clickMode = name === 'nav' ? 'goal' : clickMode === 'place' && name === 'lidar' ? 'place' : null;
  if (name === 'line' && !lineMask) info('라인 트레이싱은 “라인 트랙” 월드에서 동작합니다 — 월드를 바꿉니다.'), $('selWorld').value = 'track', reset();
  if (name === 'nav') ensureCostmap();
}
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
$('selWorld').addEventListener('change', reset);
$('selChassis').addEventListener('change', reset);
$('chkNoise').addEventListener('change', () => { robot.noise = $('chkNoise').checked; });
$('btnReset').addEventListener('click', reset);
$('btnPause').addEventListener('click', () => { paused = !paused; $('btnPause').innerHTML = paused ? '&#9654; 계속' : '&#10074;&#10074; 일시정지'; });
const bindRange = (id, out, fmt, fn) => { const r = $(id); const upd = () => { $(out).textContent = fmt(Number(r.value)); fn && fn(Number(r.value)); }; r.addEventListener('input', upd); upd(); };
bindRange('rV', 'oV', (v) => v.toFixed(2));
bindRange('rW', 'oW', (v) => v.toFixed(2));
bindRange('rTh', 'oTh', (v) => v.toFixed(2), (v) => { lidarApp.threshold = v; });
bindRange('rSa', 'oSa', (v) => v.toFixed(0), (v) => { lidarApp.scanAngle = v * Math.PI / 180; });
bindRange('rSp', 'oSp', (v) => v.toFixed(2), (v) => { lidarApp.speed = v; });
bindRange('rMr', 'oMr', (v) => v.toFixed(1));
bindRange('rInf', 'oInf', (v) => v.toFixed(2), () => { costmap = null; if (document.querySelector('[data-body="nav"]').classList.contains('active')) ensureCostmap(); });
bindRange('rKp', 'oKp', (v) => v.toFixed(2), (v) => { lineF.pid.kp = v; });
bindRange('rKd', 'oKd', (v) => v.toFixed(2), (v) => { lineF.pid.kd = v; });
bindRange('rLs', 'oLs', (v) => v.toFixed(2), (v) => { lineF.speed = v; });
function info(t) { $('info').textContent = t; }

document.querySelectorAll('[data-lmode]').forEach((b) => b.addEventListener('click', () => startMode(b.dataset.lmode)));
function startMode(m) {
  stopAll();
  mode = m;
  if (['avoid', 'follow', 'guard'].includes(m)) {
    document.querySelector(`[data-lmode="${m}"]`)?.classList.add('on');
    if (m !== 'avoid' && !world.movers.length && !world.cylinders.length) info('추종/경비할 물체가 없습니다 — “움직이는 목표 추가”를 누르세요.');
  }
  if (m === 'line') lineF.reset();
  if (m === 'explore') $('btnExplore').classList.add('on');
}
$('btnLidarStop').addEventListener('click', () => { stopAll(); info('정지'); });
$('btnMover').addEventListener('click', () => {
  const [x, y, th] = [robot.x, robot.y, robot.yaw];
  world.movers.push([x + 0.45 * Math.cos(th), y + 0.45 * Math.sin(th), 0.08 * Math.cos(th + 1.2), 0.08 * Math.sin(th + 1.2), 0.08]);
  info('로봇 앞 0.45 m에 움직이는 목표(보라)를 놓았습니다.');
});
$('btnPlace').addEventListener('click', () => { clickMode = clickMode === 'place' ? null : 'place'; $('btnPlace').classList.toggle('on', clickMode === 'place'); info(clickMode === 'place' ? '지도를 클릭하면 원통 장애물(r=0.1 m)을 놓습니다.' : '장애물 놓기 끝'); });
$('btnOdoReset').addEventListener('click', () => { robot.ox = robot.oy = robot.oyaw = 0; robot.start = [robot.x, robot.y, robot.yaw]; odoTrail = []; });

// SLAM tab
$('chkSlam').addEventListener('change', () => { slamOn = $('chkSlam').checked; });
$('selRes').addEventListener('change', () => { slam = new Grid(world.bounds, Number($('selRes').value)); });
$('btnSlamClear').addEventListener('click', () => slam.clear());
$('btnSlamSave').addEventListener('click', () => {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([slam.toPGM()], { type: 'image/x-portable-graymap' }));
  a.download = 'map.pgm'; a.click();
  const yaml = `image: map.pgm\nresolution: ${slam.res}\norigin: [${slam.x0.toFixed(3)}, ${slam.y0.toFixed(3)}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n`;
  const b = document.createElement('a');
  b.href = URL.createObjectURL(new Blob([yaml], { type: 'text/yaml' })); b.download = 'map.yaml'; b.click();
});
$('btnExplore').addEventListener('click', () => { if (mode === 'explore') stopAll(); else { startMode('explore'); if (!slamOn) { $('chkSlam').checked = slamOn = true; } } });

// NAV tab
$('selNavMap').addEventListener('change', () => { costmap = null; ensureCostmap(); });
$('selLoc').addEventListener('change', () => { if ($('selLoc').value === 'amcl') ensureAmcl(); });
$('btnNavCancel').addEventListener('click', () => { stopAll(); info('내비게이션 취소'); });
$('btnWaypoints').addEventListener('click', () => {
  waypointMode = !waypointMode; waypoints = [];
  $('btnWaypoints').textContent = '다중 지점 모드: ' + (waypointMode ? '켬 (클릭으로 추가)' : '끔');
  $('btnWpGo').disabled = true;
});
$('btnWpGo').addEventListener('click', () => { if (waypoints.length) startNav(waypoints.shift()); });
$('btnPoseEst').addEventListener('click', () => { $('selLoc').value = 'amcl'; ensureAmcl(); clickMode = 'pose'; info('지도를 클릭·드래그해 로봇의 대략적 위치와 방향을 알려 주세요.'); });
$('btnGlobal').addEventListener('click', () => { $('selLoc').value = 'amcl'; ensureAmcl(); amcl.initGlobal(world); info('파티클을 지도 전체에 뿌렸습니다 — 로봇을 움직이면 수렴합니다.'); });
$('btnKidnap').addEventListener('click', () => {
  for (let k = 0; k < 200; k++) {
    const [x0, y0, x1, y1] = world.bounds;
    const x = x0 + 0.3 + (x1 - x0 - 0.6) * rng.uniform(), y = y0 + 0.3 + (y1 - y0 - 0.6) * rng.uniform();
    if (!world.collides(x, y, robot.p.radius + 0.05)) { robot.x = x; robot.y = y; robot.yaw = wrap(robot.yaw + 2); break; }
  }
  info('로봇을 다른 곳으로 옮겼습니다. AMCL이 스스로 복구할 수 있을까요? (안 되면 전역 초기화)');
});

// LINE tab
$('btnLineGo').addEventListener('click', () => { if (!lineMask) { $('selWorld').value = 'track'; reset(); } startMode('line'); });
$('btnLineStop').addEventListener('click', () => { stopAll(); });
$('btnLineObs').addEventListener('click', () => {
  if (!world.track) return;
  const pts = world.track.points;
  let best = 0, bd = Infinity;
  pts.forEach(([x, y], k) => { const d = Math.hypot(x - robot.x, y - robot.y); if (d > 1.2 && d < bd) { bd = d; best = k; } });
  world.cylinders.push([pts[best][0], pts[best][1], 0.07]);
  info('트랙 위에 장애물을 놓았습니다 — 0.4 m 앞에서 멈추는지 보세요.');
});

// ------------------------------------------------------------------ input
const KEYMAP = { w: 1, s: 1, a: 1, d: 1, q: 1, e: 1 };
window.addEventListener('keydown', (e) => {
  if (e.target.closest('input, select, textarea')) return;
  const k = e.key.toLowerCase();
  if (KEYMAP[k]) { keys.add(k); if (mode !== 'manual') stopAll(); e.preventDefault(); }
  if (k === ' ') { stopAll(); keys.clear(); e.preventDefault(); }
  document.querySelectorAll('.keys kbd').forEach((el) => el.classList.toggle('on', keys.has(el.dataset.k)));
});
window.addEventListener('keyup', (e) => {
  keys.delete(e.key.toLowerCase());
  document.querySelectorAll('.keys kbd').forEach((el) => el.classList.toggle('on', keys.has(el.dataset.k)));
});
window.addEventListener('blur', () => keys.clear());
(() => {
  const el = $('joy'), knob = $('joyKnob');
  let active = false;
  const set = (ev) => {
    const r = el.getBoundingClientRect();
    let dx = (ev.clientX - r.left - r.width / 2) / (r.width / 2), dy = (ev.clientY - r.top - r.height / 2) / (r.height / 2);
    const m = Math.hypot(dx, dy); if (m > 1) { dx /= m; dy /= m; }
    joy = [-dy, -dx];
    knob.style.transform = `translate(${dx * 32}px, ${dy * 32}px)`;
  };
  el.addEventListener('pointerdown', (ev) => { active = true; el.setPointerCapture(ev.pointerId); if (mode !== 'manual') stopAll(); set(ev); });
  el.addEventListener('pointermove', (ev) => active && set(ev));
  const end = () => { active = false; joy = [0, 0]; knob.style.transform = ''; };
  el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
})();

$('cv').addEventListener('pointerdown', (ev) => {
  const p = view.eventToWorld(ev);
  if (clickMode === 'place') { world.cylinders.push([p[0], p[1], 0.1]); costmap = null; return; }
  if (clickMode === 'goal' || clickMode === 'pose') { goalDrag = { start: p, now: p, kind: clickMode }; $('cv').setPointerCapture(ev.pointerId); }
});
$('cv').addEventListener('pointermove', (ev) => { if (goalDrag) goalDrag.now = view.eventToWorld(ev); });
$('cv').addEventListener('pointerup', () => {
  if (!goalDrag) return;
  const { start, now, kind } = goalDrag;
  goalDrag = null;
  const yaw = Math.hypot(now[0] - start[0], now[1] - start[1]) > 0.08 ? Math.atan2(now[1] - start[1], now[0] - start[0]) : null;
  if (kind === 'pose') {
    ensureAmcl(); amcl.init([start[0], start[1], yaw ?? robot.yaw], [0.2, 0.2, 0.3]);
    clickMode = 'goal'; info('초기 위치를 설정했습니다. 이제 목표를 클릭하세요.');
    return;
  }
  const g = [start[0], start[1], yaw];
  if (waypointMode) { waypoints.push(g); $('btnWpGo').disabled = false; info(`지점 ${waypoints.length}개 — “순회 시작”을 누르세요.`); return; }
  startNav(g);
});

// ------------------------------------------------------------------ nav helpers
function ensureCostmap() {
  if (costmap) return costmap;
  const src = $('selNavMap').value;
  const p = { ...NAV_DEFAULTS, inflation: Number($('rInf').value) };
  let grid;
  if (src === 'slam' && slam.stats().occ > 20) grid = slam;
  else { grid = new Grid(world.bounds, 0.05).fromWorld(world); if (src === 'slam') info('SLAM 지도가 비어 있어 알려진 지도를 씁니다 — SLAM 탭에서 먼저 지도를 만드세요.'); }
  costmap = new Costmap(grid, p);
  return costmap;
}
function ensureAmcl() {
  if (amcl) return amcl;
  const g = new Grid(world.bounds, 0.05).fromWorld(world);
  amcl = new AMCL(g, makeRng(11));
  amcl.init([robot.x, robot.y, robot.yaw], [0.25, 0.25, 0.3]);
  return amcl;
}
function locPose() {
  const src = $('selLoc').value;
  if (src === 'odom') return robot.odomWorld();
  if (src === 'amcl' && amclEst) return amclEst.pose;
  return [robot.x, robot.y, robot.yaw];
}
function startNav(goal) {
  stopAll();
  const cm = ensureCostmap();
  const pose = locPose();
  const r = astar(cm, pose, goal);
  if (!r.path) { info('경로 계획 실패: ' + r.reason); return; }
  nav = { goal, path: r.path, progress: 0, expanded: r.expanded, t0: simT, replans: 0 };
  mode = 'nav';
  info(`A*: ${r.expanded}칸 탐색, 경로 ${pathLen(r.path).toFixed(2)} m`);
}
const pathLen = (p) => p.reduce((s, q, i) => (i ? s + Math.hypot(q[0] - p[i - 1][0], q[1] - p[i - 1][1]) : 0), 0);

// ------------------------------------------------------------------ control @10 Hz
function control() {
  const lim = $('chkUnlimited').checked ? PHYS_LIMIT : APP_LIMIT;
  if (mode === 'manual') {
    const vmax = Number($('rV').value), wmax = Number($('rW').value);
    let vx = 0, vy = 0, wz = 0;
    if (keys.has('w')) vx += vmax; if (keys.has('s')) vx -= vmax;
    if (keys.has('a')) wz += wmax; if (keys.has('d')) wz -= wmax;
    if (keys.has('q')) vy += vmax; if (keys.has('e')) vy -= vmax;
    if (joy[0] || joy[1]) { vx = joy[0] * vmax; wz = joy[1] * wmax; }
    robot.setCmd(vx, vy, wz, lim);
    return;
  }
  if (['avoid', 'follow', 'guard'].includes(mode)) {
    const c = lidarApp.step(mode, ranges, simT, robot.chassis);
    if (c) robot.setCmd(c[0], c[1], c[2], PHYS_LIMIT);
    info(`[${{ avoid: '회피', follow: '추종', guard: '경비' }[mode]}] ${lidarApp.info}`);
    return;
  }
  if (mode === 'explore') {
    const c = lidarApp.step('avoid', ranges, simT, robot.chassis);
    if (c) robot.setCmd(c[0], c[1], c[2], PHYS_LIMIT);
    return;
  }
  if (mode === 'line') {
    const c = lineF.step([robot.x, robot.y, robot.yaw], isLine, ranges, robot.chassis);
    robot.setCmd(c[0], c[1], c[2], PHYS_LIMIT);
    $('lineStat').textContent = lineF.info;
    info('[라인] ' + lineF.info);
    return;
  }
  if (mode === 'nav' && nav) {
    const pose = locPose();
    const p = { ...NAV_DEFAULTS, inflation: Number($('rInf').value) };
    const [gx, gy, gyaw] = nav.goal;
    const d = Math.hypot(gx - pose[0], gy - pose[1]);
    if (d < p.xyTol || nav.aligning) {
      nav.aligning = true;
      const e = gyaw == null ? 0 : wrap(gyaw - pose[2]);
      if (Math.abs(e) < p.yawTol || robot.chassis === 'ackermann') {
        robot.setCmd(0, 0, 0);
        info(`목표 도착 (${(simT - nav.t0).toFixed(1)} s, 오차 ${d.toFixed(3)} m)`);
        nav = null; mode = 'manual'; dwaViz = null;
        if (waypoints.length) setTimeout(() => startNav(waypoints.shift()), 300);
        return;
      }
      robot.setCmd(0, 0, Math.sign(e) * Math.min(1.0, 2 * Math.abs(e) + 0.1), PHYS_LIMIT);
      return;
    }
    let res;
    if ($('selCtrl').value === 'dwa') {
      const pts = [];
      for (let i = 0; i < N_RAYS; i += 2) { const r = ranges[i]; if (!isFinite(r)) continue; const a = pose[2] + rayAngle(i); pts.push([pose[0] + r * Math.cos(a), pose[1] + r * Math.sin(a)]); }
      res = dwa(nav, pose, robot.v, pts, p, robot.chassis);
      dwaViz = res;
    } else { res = purePursuit(nav, pose, p, robot.chassis); dwaViz = null; }
    nav.target = res.target;
    robot.setCmd(res.cmd[0], res.cmd[1], res.cmd[2], PHYS_LIMIT);
    $('navStat').textContent = `남은 거리 ${d.toFixed(2)} m · cmd v=${res.cmd[0].toFixed(2)} ω=${res.cmd[2].toFixed(2)}` + (res.recovery ? ' · 회복 동작(제자리 회전)' : '');
  }
}

// ------------------------------------------------------------------ loop
let lastTs = 0, ctrlAcc = 0;
function advance(seconds) {
  let budget = seconds;
  while (budget > 1e-9) {
    const h = Math.min(DT, budget); budget -= h;
    robot.step(h); world.stepMovers(h); simT += h;
    scanAcc += h; ctrlAcc += h;
    if (scanAcc >= 0.1 - 1e-9) {
      scanAcc -= 0.1;
      ranges = scan(world, [robot.x, robot.y, robot.yaw], robot.noise, rng);
      if (slamOn) slam.integrate($('selSlamPose').value === 'odom' ? robot.odomWorld() : [robot.x, robot.y, robot.yaw], ranges, { maxR: Number($('rMr').value) });
      if (amcl && $('selLoc').value === 'amcl') { amcl.update([robot.ox, robot.oy, robot.oyaw], ranges); amclEst = amcl.estimate(); }
      trail.push([robot.x, robot.y]); odoTrail.push(robot.odomWorld());
      if (trail.length > 3000) { trail.shift(); odoTrail.shift(); }
    }
    if (ctrlAcc >= 0.1 - 1e-9) { ctrlAcc -= 0.1; control(); }
  }
}
function loop(ts) {
  const real = lastTs ? Math.min(0.05, (ts - lastTs) / 1000) : 0;
  lastTs = ts;
  if (!paused && world) advance(real * Number($('selSpeed').value));
  render();
  requestAnimationFrame(loop);
}
// test/automation hook (used by the course's own checks; harmless for students)
window.__sim = {
  advance: (sec) => { advance(sec); render(); },
  state: () => ({ t: simT, mode, pose: [robot.x, robot.y, robot.yaw], odom: robot.odomWorld(), collisions: robot.collisions,
    distance: robot.distance, amcl: amclEst, nav: nav ? { goal: nav.goal, len: nav.path.length } : null,
    slam: slam ? slam.stats() : null, info: $('info').textContent }),
  startNav: (g) => startNav(g), startMode: (m) => startMode(m), reset: () => reset(),
};

function render() {
  if (!world) return;
  const tabActive = (n) => document.querySelector(`[data-body="${n}"]`).classList.contains('active');
  drawWorld(view, world, C, { track: world.track });
  if (slamOn || tabActive('slam')) drawGrid(view, slam.asMap());
  if (tabActive('nav') && costmap && $('chkCost').checked) drawGrid(view, costmap.asMap());
  drawMovers(view, world.movers, C);
  if ($('chkTrail').checked) drawPath(view, trail, C.accent, 1.8);
  if ($('chkOdom').checked) {
    drawPath(view, odoTrail, C.warn, 1.4, [5, 4]);
    drawRobot(view, robot.odomWorld(), robot.chassis, robot.p, C, { steer: robot.steer, ghost: true, color: '#b45309' });
  }
  // lidar window & threshold
  if (tabActive('lidar') && $('chkSector').checked) {
    const ctx = view.ctx, [p, q] = view.px(robot.x, robot.y), R = lidarApp.threshold * view.scale, half = lidarApp.scanAngle / 2;
    ctx.fillStyle = 'rgba(79,70,229,.12)'; ctx.strokeStyle = C.accent; ctx.lineWidth = 1.5 * view.dpr;
    ctx.beginPath(); ctx.moveTo(p, q); ctx.arc(p, q, R, -robot.yaw - half, -robot.yaw + half); ctx.closePath(); ctx.fill(); ctx.stroke();
  }
  drawScan(view, [robot.x, robot.y, robot.yaw], ranges, C, { rays: $('chkRays').checked });
  // nav
  if (nav) {
    drawPath(view, nav.path, C.task, 2.4);
    if (nav.target) { const [p, q] = view.px(nav.target[0], nav.target[1]); view.ctx.fillStyle = C.task; view.ctx.beginPath(); view.ctx.arc(p, q, 4 * view.dpr, 0, 7); view.ctx.fill(); }
    drawGoal(view, nav.goal, C);
  }
  for (const w of waypoints) drawGoal(view, w, C);
  if (dwaViz && dwaViz.trajs) {
    for (const t of dwaViz.trajs) drawPath(view, t.tr, t.ok ? 'rgba(31,157,85,.35)' : 'rgba(214,69,65,.25)', 1);
    if (dwaViz.best) drawPath(view, dwaViz.best, C.ok, 3);
  }
  if (goalDrag) drawGoal(view, [goalDrag.start[0], goalDrag.start[1], Math.atan2(goalDrag.now[1] - goalDrag.start[1], goalDrag.now[0] - goalDrag.start[0])], C);
  // AMCL particles
  if (amcl && $('selLoc').value === 'amcl') {
    const ctx = view.ctx; ctx.strokeStyle = C.danger; ctx.lineWidth = 1;
    ctx.beginPath();
    for (const pt of amcl.parts) {
      const [p, q] = view.px(pt.x, pt.y), [a, b] = view.px(pt.x + 0.07 * Math.cos(pt.th), pt.y + 0.07 * Math.sin(pt.th));
      ctx.moveTo(p, q); ctx.lineTo(a, b);
    }
    ctx.stroke();
    if (amclEst) drawRobot(view, amclEst.pose, robot.chassis, robot.p, C, { ghost: true, color: '#7b4fd1' });
  }
  drawRobot(view, [robot.x, robot.y, robot.yaw], robot.chassis, robot.p, C, { steer: robot.steer });
  if (lineMask && mode === 'line') drawCamFootprint();
  hud();
  if (lineMask && (mode === 'line' || document.querySelector('[data-body="line"]').classList.contains('active'))) renderCam();
}

function drawCamFootprint() {
  const pose = [robot.x, robot.y, robot.yaw];
  const c = [[0, 0], [CAM.W, 0], [CAM.W, CAM.H], [0, CAM.H]].map(([u, v]) => camPixelToWorld(pose, u, v));
  drawPath(view, [...c, c[0]], 'rgba(79,70,229,.7)', 1.2, [4, 3]);
}

let camTick = 0;
function renderCam() {
  if (++camTick % 3) return;
  const cv = $('cam'), ctx = cv.getContext('2d'), img = ctx.createImageData(CAM.W, CAM.H), d = img.data;
  const pose = [robot.x, robot.y, robot.yaw];
  for (let v = 0; v < CAM.H; v++) for (let u = 0; u < CAM.W; u++) {
    const [x, y] = camPixelToWorld(pose, u + 0.5, v + 0.5);
    let col = isLine(x, y) ? 25 : 215;
    for (const [cx, cy, r] of world.cylinders) if ((x - cx) ** 2 + (y - cy) ** 2 < r * r) col = 90;
    const o = (v * CAM.W + u) * 4; d[o] = col; d[o + 1] = col; d[o + 2] = col + (col > 100 ? 8 : 0); d[o + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  ctx.strokeStyle = '#4f46e5'; ctx.lineWidth = 1;
  for (const [a, b] of [[0.9, 0.95], [0.8, 0.85], [0.7, 0.75]]) ctx.strokeRect(0.5, a * CAM.H, CAM.W - 1, (b - a) * CAM.H);
  ctx.fillStyle = '#ef4444';
  for (const [v, cx] of lineF.rois || []) ctx.fillRect(cx - 2, v - 2, 4, 4);
  ctx.strokeStyle = '#22c55e'; ctx.beginPath(); ctx.moveTo(CAM.W / 2, CAM.H); ctx.lineTo(CAM.W / 2, CAM.H * 0.6); ctx.stroke();
  if (lineF.center != null && mode === 'line') { ctx.strokeStyle = '#ef4444'; ctx.beginPath(); ctx.moveTo(CAM.W / 2, CAM.H); ctx.lineTo(lineF.center, CAM.H / 2); ctx.stroke(); }
}

function hud() {
  const deg = (r) => (r * 180 / Math.PI).toFixed(1) + '°';
  const od = robot.odomWorld();
  const f = (a) => a.map((v) => v.toFixed(2)).join(', ');
  let h = `<div><b>t</b>${simT.toFixed(1)} s · 모드 ${mode}</div>` +
    `<div class="gt"><b>실제</b>(${robot.x.toFixed(2)}, ${robot.y.toFixed(2)}, ${deg(robot.yaw)})</div>` +
    `<div class="od"><b>odom</b>(${robot.ox.toFixed(2)}, ${robot.oy.toFixed(2)}, ${deg(robot.oyaw)})</div>`;
  if (amclEst && $('selLoc').value === 'amcl') h += `<div class="am"><b>AMCL</b>(${amclEst.pose[0].toFixed(2)}, ${amclEst.pose[1].toFixed(2)}, ${deg(amclEst.pose[2])}) σ=${amclEst.std.toFixed(2)}</div>`;
  h += `<div><b>cmd</b>(${f(robot.cmd)})</div><div><b>충돌</b>${robot.collisions} · 거리 ${robot.distance.toFixed(2)} m</div>`;
  $('hud').innerHTML = h;
  const w = robot.wheels();
  if (robot.chassis === 'mecanum') {
    const names = ['M1 왼앞', 'M2 왼뒤', 'M3 오앞', 'M4 오뒤'];
    $('wheels').innerHTML = w.rps.map((r, i) => `<div><span>${names[i]}</span>${r >= 0 ? '+' : ''}${r.toFixed(2)} rps</div>`).join('');
  } else {
    $('wheels').innerHTML = `<div><span>조향각 δ</span>${w.steerDeg.toFixed(1)}°</div><div><span>서보 펄스</span>${w.servo} µs</div>` +
      `<div><span>M2 왼뒤</span>${w.rps[1].toFixed(2)} rps</div><div><span>M4 오뒤</span>${w.rps[3].toFixed(2)} rps</div>`;
  }
  const e = Math.hypot(od[0] - robot.x, od[1] - robot.y);
  $('odoErr').textContent = `위치 오차 ${e.toFixed(3)} m · 방향 오차 ${deg(wrap(od[2] - robot.yaw))} · 이동 ${robot.distance.toFixed(2)} m`;
  if (slam && (slamOn || document.querySelector('[data-body="slam"]').classList.contains('active'))) {
    if (camTick % 15 === 0 || !slamStatCache) { const s = slam.stats(); slamStatCache = `점유 ${s.occ} · 빈칸 ${s.free} · 미확인 ${s.unknown} 칸 (${slam.w}×${slam.h}, ${slam.res} m)`; }
    $('slamStat').textContent = slamStatCache;
  }
  camTick++;
}
let slamStatCache = '';

new ResizeObserver(() => { view.resize(); if (world) view.fit(world.bounds); }).observe($('stage'));
new MutationObserver(() => { C = themeColors(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
boot().catch((e) => { $('subTitle').textContent = '로드 실패: ' + e.message; console.error(e); });
