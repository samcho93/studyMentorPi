// world2d.js — shared top-down renderer for the MentorPi 2D simulator and the Playground replay.
// World units are metres (x right, y up, yaw CCW from +x, REP-103). Canvas is HiDPI-aware.

export function themeColors(el = document.documentElement) {
  const cs = getComputedStyle(el);
  const v = (n, d) => (cs.getPropertyValue(n).trim() || d);
  return {
    bg: v('--bg-elev', '#fff'),
    floor: v('--bg', '#f4f6f9'),
    grid: v('--border-soft', '#e9edf3'),
    wall: v('--text-dim', '#4b5567'),
    obstacle: v('--text-faint', '#6b7587'),
    text: v('--text', '#1d2433'),
    faint: v('--text-faint', '#6b7587'),
    accent: v('--accent', '#4f46e5'),
    ok: v('--ok', '#1f9d55'),
    warn: v('--warn', '#c98a00'),
    danger: v('--danger', '#d64541'),
    task: v('--task', '#7b4fd1'),
  };
}

export class View {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.scale = 100;
    this.ox = 0; this.oy = 0;
    this.bounds = [-3, -3, 3, 3];
  }
  resize() {
    const r = this.canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(50, Math.round(r.width * dpr)), h = Math.max(50, Math.round(r.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    this.dpr = dpr;
    this.fit(this.bounds);
  }
  fit(b, margin = 0.06) {
    this.bounds = b;
    const W = this.canvas.width, H = this.canvas.height;
    const bw = b[2] - b[0], bh = b[3] - b[1];
    this.scale = Math.min(W * (1 - 2 * margin) / bw, H * (1 - 2 * margin) / bh);
    this.ox = W / 2 - this.scale * (b[0] + b[2]) / 2;
    this.oy = H / 2 + this.scale * (b[1] + b[3]) / 2;
  }
  px(x, y) { return [this.ox + x * this.scale, this.oy - y * this.scale]; }
  world(px, py) { return [(px - this.ox) / this.scale, (this.oy - py) / this.scale]; }
  eventToWorld(ev) {
    const r = this.canvas.getBoundingClientRect();
    return this.world((ev.clientX - r.left) * this.dpr, (ev.clientY - r.top) * this.dpr);
  }
}

export function drawWorld(view, world, C, opts = {}) {
  const { ctx } = view;
  const [x0, y0, x1, y1] = world.bounds;
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, view.canvas.width, view.canvas.height);
  // floor + 0.5 m grid
  let [a, b] = view.px(x0, y1), [c, d] = view.px(x1, y0);
  ctx.fillStyle = C.floor;
  ctx.fillRect(a, b, c - a, d - b);
  if (opts.grid !== false) {
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(x0 * 2) / 2; x <= x1 + 1e-9; x += 0.5) { const [p] = view.px(x, 0); ctx.moveTo(p, b); ctx.lineTo(p, d); }
    for (let y = Math.ceil(y0 * 2) / 2; y <= y1 + 1e-9; y += 0.5) { const [, q] = view.px(0, y); ctx.moveTo(a, q); ctx.lineTo(c, q); }
    ctx.stroke();
  }
  if (opts.track) drawTrack(view, opts.track, C);
  if (opts.map) drawGrid(view, opts.map);
  // obstacles
  ctx.fillStyle = C.obstacle; ctx.globalAlpha = 0.85;
  for (const [cx, cy, w, h] of world.boxes || []) {
    const [p, q] = view.px(cx - w / 2, cy + h / 2);
    ctx.fillRect(p, q, w * view.scale, h * view.scale);
  }
  for (const [cx, cy, r] of world.cylinders || []) {
    const [p, q] = view.px(cx, cy);
    ctx.beginPath(); ctx.arc(p, q, r * view.scale, 0, Math.PI * 2); ctx.fill();
  }
  ctx.globalAlpha = 1;
  ctx.strokeStyle = C.wall; ctx.lineWidth = Math.max(2, 0.04 * view.scale); ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.rect(a, b, c - a, d - b);
  for (const [wx0, wy0, wx1, wy1] of world.walls || []) {
    const [p0, q0] = view.px(wx0, wy0), [p1, q1] = view.px(wx1, wy1);
    ctx.moveTo(p0, q0); ctx.lineTo(p1, q1);
  }
  ctx.stroke();
  // scale bar
  ctx.fillStyle = C.faint; ctx.font = `${12 * (view.dpr || 1)}px sans-serif`;
  const [sx, sy] = [a, d + 16 * (view.dpr || 1)];
  if (sy < view.canvas.height - 2) {
    ctx.fillRect(sx, sy - 4, view.scale, 2);
    ctx.fillText('1 m', sx + view.scale + 6, sy);
  }
}

export function drawMovers(view, movers, C) {
  const { ctx } = view;
  ctx.fillStyle = C.task;
  for (const [x, y, r] of movers || []) {
    const [p, q] = view.px(x, y);
    ctx.beginPath(); ctx.arc(p, q, r * view.scale, 0, Math.PI * 2); ctx.fill();
  }
}

export function drawTrack(view, track, C) {
  // track: {width, points:[[x,y],...], closed}
  const { ctx } = view;
  ctx.strokeStyle = '#111'; ctx.lineWidth = track.width * view.scale; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.beginPath();
  track.points.forEach(([x, y], i) => { const [p, q] = view.px(x, y); i ? ctx.lineTo(p, q) : ctx.moveTo(p, q); });
  if (track.closed) ctx.closePath();
  ctx.stroke();
}

export function drawGrid(view, map) {
  // map: {res, x0, y0, w, h, canvas} (canvas prepared by caller, row 0 = y0)
  const { ctx } = view;
  const [p, q] = view.px(map.x0, map.y0 + map.h * map.res);
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(map.canvas, p, q, map.w * map.res * view.scale, map.h * map.res * view.scale);
  ctx.restore();
}

export function drawPath(view, pts, color, width = 2, dash = null) {
  if (!pts || pts.length < 2) return;
  const { ctx } = view;
  ctx.strokeStyle = color; ctx.lineWidth = width * (view.dpr || 1);
  ctx.setLineDash(dash ? dash.map((v) => v * (view.dpr || 1)) : []);
  ctx.beginPath();
  pts.forEach(([x, y], i) => { const [p, q] = view.px(x, y); i ? ctx.lineTo(p, q) : ctx.moveTo(p, q); });
  ctx.stroke();
  ctx.setLineDash([]);
}

export function drawScan(view, pose, ranges, C, { angleMin = -Math.PI, step = null, rays = false, color = null } = {}) {
  if (!ranges || !ranges.length) return;
  const { ctx } = view;
  const n = ranges.length;
  const inc = step || (2 * Math.PI / n);
  const [x, y, yaw] = pose;
  const [px0, py0] = view.px(x, y);
  const col = color || C.danger;
  if (rays) {
    ctx.strokeStyle = col; ctx.globalAlpha = 0.12; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i < n; i += 2) {
      const r = ranges[i]; if (r == null || !isFinite(r)) continue;
      const a = yaw + angleMin + i * inc;
      const [p, q] = view.px(x + r * Math.cos(a), y + r * Math.sin(a));
      ctx.moveTo(px0, py0); ctx.lineTo(p, q);
    }
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  ctx.fillStyle = col;
  const s = Math.max(2, 2.2 * (view.dpr || 1));
  for (let i = 0; i < n; i++) {
    const r = ranges[i]; if (r == null || !isFinite(r)) continue;
    const a = yaw + angleMin + i * inc;
    const [p, q] = view.px(x + r * Math.cos(a), y + r * Math.sin(a));
    ctx.fillRect(p - s / 2, q - s / 2, s, s);
  }
}

// geom: {length, width, wheelbase, track_width, wheel_diameter}
export function drawRobot(view, pose, chassis, geom, C, { steer = 0, ghost = false, color = null } = {}) {
  const { ctx } = view;
  const [x, y, yaw] = pose;
  const [p, q] = view.px(x, y);
  const k = view.scale;
  ctx.save();
  ctx.translate(p, q);
  ctx.rotate(-yaw);
  ctx.globalAlpha = ghost ? 0.35 : 1;
  const L = geom.length * k, W = geom.width * k;
  const wl = geom.wheel_diameter * k, ww = 0.028 * k;
  const hx = geom.wheelbase / 2 * k, hy = (geom.track_width / 2 + 0.012) * k;
  // wheels
  ctx.fillStyle = '#1f2937';
  const wheel = (cx, cy, ang = 0) => {
    ctx.save(); ctx.translate(cx, cy); ctx.rotate(-ang);
    ctx.fillRect(-wl / 2, -ww / 2, wl, ww);
    if (chassis === 'mecanum') {
      ctx.strokeStyle = '#9ca3af'; ctx.lineWidth = Math.max(1, 0.004 * k);
      ctx.beginPath();
      const sgn = (cx > 0) === (cy > 0) ? 1 : -1;   // roller direction pattern (X layout seen from top)
      for (let i = -1; i <= 1; i++) { ctx.moveTo(i * wl / 4 - sgn * ww / 2, -ww / 2); ctx.lineTo(i * wl / 4 + sgn * ww / 2, ww / 2); }
      ctx.stroke();
    }
    ctx.restore();
  };
  if (chassis === 'ackermann') {
    wheel(hx, -hy, steer); wheel(hx, hy, steer); wheel(-hx, -hy); wheel(-hx, hy);
  } else {
    wheel(hx, -hy); wheel(hx, hy); wheel(-hx, -hy); wheel(-hx, hy);
  }
  // body
  ctx.fillStyle = color || (chassis === 'ackermann' ? '#0f766e' : '#15803d');
  ctx.strokeStyle = '#052e16'; ctx.lineWidth = Math.max(1, 0.006 * k);
  const r = 0.025 * k;
  ctx.beginPath();
  ctx.roundRect ? ctx.roundRect(-L / 2, -W / 2 + ww * 0.6, L, W - ww * 1.2, r) : ctx.rect(-L / 2, -W / 2, L, W);
  ctx.fill(); ctx.stroke();
  // lidar puck + heading
  ctx.fillStyle = '#111827';
  ctx.beginPath(); ctx.arc(-0.012 * k, 0, 0.03 * k, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#fbbf24';
  ctx.beginPath(); ctx.moveTo(L / 2 - 0.005 * k, 0); ctx.lineTo(L / 2 - 0.05 * k, -0.03 * k); ctx.lineTo(L / 2 - 0.05 * k, 0.03 * k); ctx.closePath(); ctx.fill();
  ctx.restore();
}

export function drawGoal(view, goal, C) {
  if (!goal) return;
  const { ctx } = view;
  const [p, q] = view.px(goal[0], goal[1]);
  const s = 9 * (view.dpr || 1);
  ctx.strokeStyle = C.accent; ctx.lineWidth = 2.5 * (view.dpr || 1);
  ctx.beginPath(); ctx.arc(p, q, s, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(p - s * 1.5, q); ctx.lineTo(p + s * 1.5, q); ctx.moveTo(p, q - s * 1.5); ctx.lineTo(p, q + s * 1.5); ctx.stroke();
  if (goal.length > 2 && goal[2] != null) {
    const [a, b] = view.px(goal[0] + 0.25 * Math.cos(goal[2]), goal[1] + 0.25 * Math.sin(goal[2]));
    ctx.beginPath(); ctx.moveTo(p, q); ctx.lineTo(a, b); ctx.stroke();
  }
}

// compose odom pose onto the start pose (odom frame origin = start pose)
export function odomToWorld(start, od) {
  const [sx, sy, syaw] = start, [x, y, yaw] = od;
  const c = Math.cos(syaw), s = Math.sin(syaw);
  return [sx + c * x - s * y, sy + s * x + c * y, syaw + yaw];
}
