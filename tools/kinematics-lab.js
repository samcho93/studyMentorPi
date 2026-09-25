// kinematics-lab.js — mecanum inverse/forward kinematics and Ackermann steering geometry.
// Formulas follow MentorPi controller/mecanum.py and controller/ackermann.py.
import { themeColors } from '../assets/js/world2d.js';

const $ = (id) => document.getElementById(id);
const cv = $('cv'), ctx = cv.getContext('2d');
let C = themeColors(), tab = 'mec', dpr = 1;
const D_MEC = 0.065, D_ACK = 0.067;
const f = (v, n = 3) => (v >= 0 ? '+' : '') + v.toFixed(n);

const PRESETS = [
  ['전진', 0.2, 0, 0], ['후진', -0.2, 0, 0], ['왼쪽 옆', 0, 0.2, 0], ['오른쪽 옆', 0, -0.2, 0],
  ['대각 ↖', 0.14, 0.14, 0], ['대각 ↘', -0.14, -0.14, 0], ['제자리 회전', 0, 0, 1.0], ['선회 (앞+회전)', 0.2, 0, 0.8],
  ['옆으로 돌며 바라보기', 0, 0.2, -0.6],
];
$('mecPresets').innerHTML = PRESETS.map((p, i) => `<button type="button" data-i="${i}">${p[0]}</button>`).join('');
$('mecPresets').addEventListener('click', (e) => {
  const b = e.target.closest('button'); if (!b) return;
  const [, vx, vy, wz] = PRESETS[Number(b.dataset.i)];
  $('vx').value = vx; $('vy').value = vy; $('wz').value = wz;
  update();
});

function resize() {
  const r = cv.getBoundingClientRect();
  dpr = Math.min(window.devicePixelRatio || 1, 2);
  cv.width = Math.max(100, r.width * dpr); cv.height = Math.max(100, r.height * dpr);
  draw();
}

// --------------------------------------------------------------- math
function mecanum() {
  const vx = +$('vx').value, vy = +$('vy').value, wz = +$('wz').value, a = +$('ma').value, b = +$('mb').value;
  const k = (a + b) / 2;
  const m = [vx - vy - wz * k, vx + vy - wz * k, vx + vy + wz * k, vx - vy + wz * k];   // LF, LB, RF, RB surface speeds
  const circ = Math.PI * D_MEC;
  const rps = [-m[0] / circ, -m[1] / circ, m[2] / circ, m[3] / circ];
  // forward kinematics (least squares inverse of J)
  const fk = [(m[0] + m[1] + m[2] + m[3]) / 4, (-m[0] + m[1] + m[2] - m[3]) / 4, (-m[0] - m[1] + m[2] + m[3]) / (4 * k)];
  return { vx, vy, wz, a, b, k, m, rps, fk };
}
function ackermann() {
  const v = +$('av').value, w = +$('aw').value, L = +$('aL').value, T = +$('aT').value;
  const MAX = 29 * Math.PI / 180;
  let delta = 0, clamped = false;
  if (Math.abs(v) > 1e-8 && Math.abs(w) > 1e-8) {
    delta = Math.atan(L * w / v);
    if (Math.abs(delta) > MAX) { delta = Math.sign(delta) * MAX; clamped = true; }
  }
  const weff = Math.abs(v) > 1e-8 ? v * Math.tan(delta) / L : 0;
  const R = Math.abs(delta) > 1e-6 ? L / Math.tan(delta) : Infinity;
  // ideal Ackermann inner/outer front wheel angles for this R
  const din = Math.abs(delta) > 1e-6 ? Math.atan(L / (Math.abs(R) - T / 2)) * Math.sign(delta) : 0;
  const dout = Math.abs(delta) > 1e-6 ? Math.atan(L / (Math.abs(R) + T / 2)) * Math.sign(delta) : 0;
  const vl = v - w * T / 2, vr = v + w * T / 2;                     // as in ackermann.py (uses commanded ω)
  const circ = Math.PI * D_ACK;
  const servo = Math.abs(v) > 1e-8 ? Math.round(1500 + 2000 * (-delta * 180 / Math.PI) / 180) : 1500;
  return { v, w, L, T, delta, clamped, weff, R, din, dout, vl, vr, rps: [0, vl / circ, 0, -vr / circ], servo, Rmin: L / Math.tan(MAX) };
}

// --------------------------------------------------------------- UI text
function update() {
  for (const [id, o, n] of [['vx', 'oVx', 2], ['vy', 'oVy', 2], ['wz', 'oWz', 2], ['av', 'oAv', 2], ['aw', 'oAw', 2]]) $(o).textContent = (+$(id).value).toFixed(n);
  const M = mecanum();
  const k = M.k.toFixed(4);
  $('mecMat').textContent =
    `    ⎡ 1  -1  -k ⎤   v_LF = ${f(M.m[0])}\n` +
    `J = ⎢ 1  +1  -k ⎥   v_LB = ${f(M.m[1])}\n` +
    `    ⎢ 1  +1  +k ⎥   v_RF = ${f(M.m[2])}\n` +
    `    ⎣ 1  -1  +k ⎦   v_RB = ${f(M.m[3])}  m/s\n` +
    `k = (a+b)/2 = ${k} m\nrps = v / (π·${D_MEC} m)`;
  const names = ['M1 왼앞 (LF)', 'M2 왼뒤 (LB)', 'M3 오앞 (RF)', 'M4 오뒤 (RB)'];
  $('mecKv').innerHTML = names.map((n, i) => `<div><span>${n}</span><b>${f(M.rps[i], 2)} rps</b></div>`).join('');
  $('mecFk').textContent =
    `vx = (v1+v2+v3+v4)/4        = ${f(M.fk[0])} m/s\n` +
    `vy = (-v1+v2+v3-v4)/4       = ${f(M.fk[1])} m/s\n` +
    `ωz = (-v1-v2+v3+v4)/(4·k)   = ${f(M.fk[2])} rad/s`;
  const A = ackermann();
  const deg = (r) => (r * 180 / Math.PI).toFixed(1) + '°';
  $('ackKv').innerHTML =
    `<div><span>조향각 δ ${A.clamped ? '(29° 제한!)' : ''}</span><b>${deg(A.delta)}</b></div>` +
    `<div><span>서보 펄스</span><b>${A.servo} µs</b></div>` +
    `<div><span>회전 반경 R</span><b>${isFinite(A.R) ? Math.abs(A.R).toFixed(3) + ' m' : '∞ (직진)'}</b></div>` +
    `<div><span>실제 ωz = v·tanδ/L</span><b>${A.weff.toFixed(3)} rad/s</b></div>` +
    `<div><span>M2 왼뒤</span><b>${A.rps[1].toFixed(2)} rps</b></div>` +
    `<div><span>M4 오뒤</span><b>${A.rps[3].toFixed(2)} rps</b></div>`;
  $('ackMat').textContent =
    `δ = atan(L·ω/v) = atan(${A.L}·${A.w.toFixed(2)}/${A.v.toFixed(2)})\n` +
    `이상적 애커만: 안쪽 δi = ${deg(A.din)}, 바깥 δo = ${deg(A.dout)}\n` +
    `v_l = v − ω·T/2 = ${A.vl.toFixed(3)},  v_r = v + ω·T/2 = ${A.vr.toFixed(3)} m/s\n` +
    `최소 회전 반경 R_min = L/tan29° = ${A.Rmin.toFixed(3)} m`;
  draw();
}
['vx', 'vy', 'wz', 'ma', 'mb', 'av', 'aw', 'aL', 'aT'].forEach((id) => $(id).addEventListener('input', update));
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
  tab = b.dataset.tab;
  document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
  document.querySelectorAll('.tab-body').forEach((x) => x.classList.toggle('active', x.dataset.body === tab));
  draw();
}));

// --------------------------------------------------------------- drawing (robot frame: x up on screen, y left)
function arrow(x0, y0, x1, y1, color, w = 3) {
  const a = Math.atan2(y1 - y0, x1 - x0), L = Math.hypot(x1 - x0, y1 - y0);
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = w * dpr;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  if (L < 4 * dpr) return;
  const h = Math.min(12 * dpr, L * 0.5);
  ctx.beginPath(); ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - h * Math.cos(a - 0.4), y1 - h * Math.sin(a - 0.4));
  ctx.lineTo(x1 - h * Math.cos(a + 0.4), y1 - h * Math.sin(a + 0.4)); ctx.closePath(); ctx.fill();
}
function draw() {
  const W = cv.width, H = cv.height;
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, W, H);
  const S = Math.min(W, H) / 0.75;         // px per metre (≈0.75 m view)
  const cx = W / 2, cy = H * 0.58;
  // robot frame -> screen: x forward = up, y left = left
  const P = (x, y) => [cx - y * S, cy - x * S];
  ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
  for (let g = -1; g <= 1; g += 0.05) {
    const [a] = P(0, g), [, b] = P(g, 0);
    ctx.beginPath(); ctx.moveTo(a, 0); ctx.lineTo(a, H); ctx.moveTo(0, b); ctx.lineTo(W, b); ctx.stroke();
  }
  ctx.font = `${12 * dpr}px sans-serif`; ctx.fillStyle = C.faint;
  ctx.fillText('x (전방) ↑   y (왼쪽) ←   격자 5 cm', 12 * dpr, 20 * dpr);
  if (tab === 'mec') drawMec(P, S); else drawAck(P, S);
}
function predict(vx, vy, wz, P, color) {
  let x = 0, y = 0, th = 0; const pts = [P(0, 0)];
  for (let t = 0; t < 3; t += 0.03) { x += (vx * Math.cos(th) - vy * Math.sin(th)) * 0.03; y += (vx * Math.sin(th) + vy * Math.cos(th)) * 0.03; th += wz * 0.03; pts.push(P(x, y)); }
  ctx.strokeStyle = color; ctx.lineWidth = 2 * dpr; ctx.setLineDash([6 * dpr, 5 * dpr]);
  ctx.beginPath(); pts.forEach(([a, b], i) => (i ? ctx.lineTo(a, b) : ctx.moveTo(a, b))); ctx.stroke(); ctx.setLineDash([]);
}
function drawBody(P, S, len, wid) {
  const [x0, y0] = P(len / 2, wid / 2);
  ctx.fillStyle = 'rgba(21,128,61,.18)'; ctx.strokeStyle = '#15803d'; ctx.lineWidth = 2 * dpr;
  ctx.fillRect(x0, y0, wid * S, len * S); ctx.strokeRect(x0, y0, wid * S, len * S);
}
function wheelRect(P, S, x, y, ang, rollerSign) {
  const [px, py] = P(x, y), wl = D_MEC * S, ww = 0.028 * S;
  ctx.save(); ctx.translate(px, py); ctx.rotate(-ang);
  ctx.fillStyle = '#374151'; ctx.fillRect(-ww / 2, -wl / 2, ww, wl);
  if (rollerSign) {
    ctx.strokeStyle = '#9ca3af'; ctx.lineWidth = 1.5 * dpr; ctx.beginPath();
    for (let i = -2; i <= 2; i++) { ctx.moveTo(-ww / 2, i * wl / 6 - rollerSign * ww / 2); ctx.lineTo(ww / 2, i * wl / 6 + rollerSign * ww / 2); }
    ctx.stroke();
  }
  ctx.restore();
}
function drawMec(P, S) {
  const M = mecanum(), a = M.a / 2, b = M.b / 2 + 0.014;
  predict(M.vx, M.vy, M.wz, P, C.accent);
  drawBody(P, S, M.a + 0.07, M.b - 0.02);
  // roller directions (X layout from top): LF & RB rollers '\', LB & RF '/'  (screen)
  const wheels = [['LF', a, b, M.m[0], 1], ['LB', -a, b, M.m[1], -1], ['RF', a, -b, M.m[2], -1], ['RB', -a, -b, M.m[3], 1]];
  const K = 0.9 * S;                                     // arrow scale: m/s -> px
  for (const [n, x, y, v, rs] of wheels) {
    wheelRect(P, S, x, y, 0, rs);
    const [px, py] = P(x, y);
    arrow(px, py, px, py - v * K, '#e0823d', 3.5);
    ctx.fillStyle = C.text; ctx.font = `600 ${12 * dpr}px sans-serif`;
    ctx.fillText(`${n} ${M.m[['LF', 'LB', 'RF', 'RB'].indexOf(n)].toFixed(2)}`, px + (y > 0 ? -70 : 18) * dpr, py + 4 * dpr);
  }
  const [ox, oy] = P(0, 0);
  arrow(ox, oy, ox - M.vy * K, oy - M.vx * K, C.accent, 4);
  if (Math.abs(M.wz) > 1e-3) {
    ctx.strokeStyle = C.accent; ctx.lineWidth = 2.5 * dpr; ctx.beginPath();
    const r = 0.05 * S, s0 = -Math.PI / 2, s1 = s0 - M.wz * 1.2;
    ctx.arc(ox, oy, r, Math.min(s0, s1), Math.max(s0, s1)); ctx.stroke();
    // ICR: point with zero velocity = (-vy/ω, vx/ω) in robot frame
    const ix = -M.vy / M.wz, iy = M.vx / M.wz;
    if (Math.hypot(ix, iy) < 1.5) {
      const [p, q] = P(ix, iy);
      ctx.fillStyle = C.danger; ctx.beginPath(); ctx.arc(p, q, 5 * dpr, 0, 7); ctx.fill();
      ctx.fillText('ICR', p + 8 * dpr, q - 6 * dpr);
    }
  }
}
function drawAck(P, S) {
  const A = ackermann(), L = A.L, T = A.T;
  predict(A.v, 0, A.weff, P, C.accent);
  // rear axle at x = -L/2 (origin = centre)
  drawBody(P, S, L + 0.08, T - 0.03);
  const xr = -L / 2, xf = L / 2, K = 0.9 * S;
  wheelRect(P, S, xr, T / 2 + 0.01, 0, 0); wheelRect(P, S, xr, -T / 2 - 0.01, 0, 0);
  const inL = A.delta > 0;
  wheelRect(P, S, xf, T / 2 + 0.01, inL ? A.din : A.dout, 0);
  wheelRect(P, S, xf, -T / 2 - 0.01, inL ? A.dout : A.din, 0);
  for (const [y, v] of [[T / 2 + 0.01, A.vl], [-T / 2 - 0.01, A.vr]]) { const [p, q] = P(xr, y); arrow(p, q, p, q - v * K, '#e0823d', 3.5); }
  if (isFinite(A.R) && Math.abs(A.R) < 2) {
    const [ip, iq] = P(xr, A.R);                        // ICR on the rear-axle line
    ctx.strokeStyle = C.danger; ctx.lineWidth = 1.2 * dpr; ctx.setLineDash([4 * dpr, 4 * dpr]);
    ctx.beginPath(); const [a1, b1] = P(xr, 0); ctx.moveTo(a1, b1); ctx.lineTo(ip, iq);
    for (const y of [T / 2, -T / 2]) { const [a2, b2] = P(xf, y); ctx.moveTo(a2, b2); ctx.lineTo(ip, iq); }
    ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = C.danger; ctx.beginPath(); ctx.arc(ip, iq, 5 * dpr, 0, 7); ctx.fill();
    ctx.font = `600 ${12 * dpr}px sans-serif`; ctx.fillText(`ICR  R=${Math.abs(A.R).toFixed(2)} m`, ip + 8 * dpr, iq - 8 * dpr);
    // circle traced by the rear-axle centre
    ctx.strokeStyle = 'rgba(214,69,65,.35)'; ctx.beginPath(); ctx.arc(ip, iq, Math.abs(A.R) * S, 0, 7); ctx.stroke();
  }
  ctx.fillStyle = C.text; ctx.font = `600 ${13 * dpr}px sans-serif`;
  const [tp, tq] = P(xf + 0.06, 0); ctx.fillText(`δ = ${(A.delta * 180 / Math.PI).toFixed(1)}°`, tp - 30 * dpr, tq);
}

new ResizeObserver(resize).observe($('stage'));
new MutationObserver(() => { C = themeColors(); draw(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
const q = new URLSearchParams(location.search).get('tab');
if (q === 'ack') document.querySelector('[data-tab="ack"]').click();
update();
