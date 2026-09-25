// nav.js — tiny Nav2: inflated costmap, A* global planner, Pure Pursuit or DWA local controller.
import { distanceField } from './grid.js';
import { wrap } from './physics.js';

export const NAV_DEFAULTS = {
  robotRadius: 0.13,        // m — inscribed radius (lethal)
  inflation: 0.30,          // m — inflation_radius
  costScaling: 6.0,         // cost_scaling_factor
  maxV: 0.30, maxW: 1.2,    // controller limits (/controller/cmd_vel)
  accV: 0.6, accW: 2.5,
  lookahead: 0.35,
  xyTol: 0.08, yawTol: 0.15,
};

export class Costmap {
  constructor(grid, p = NAV_DEFAULTS) {
    this.grid = grid; this.p = p;
    this.dist = distanceField(grid);
    const n = grid.w * grid.h;
    this.cost = new Uint8Array(n);
    for (let k = 0; k < n; k++) {
      const d = this.dist[k];
      if (d <= p.robotRadius) this.cost[k] = 254;                                    // inscribed/lethal
      else if (d <= p.inflation) this.cost[k] = Math.max(1, Math.round(252 * Math.exp(-p.costScaling * (d - p.robotRadius))));
      else this.cost[k] = 0;
    }
    this.canvas = document.createElement('canvas');
    this.canvas.width = grid.w; this.canvas.height = grid.h;
    const ctx = this.canvas.getContext('2d'), img = ctx.createImageData(grid.w, grid.h);
    for (let j = 0; j < grid.h; j++) for (let i = 0; i < grid.w; i++) {
      const c = this.cost[j * grid.w + i], o = ((grid.h - 1 - j) * grid.w + i) * 4;
      if (!c) continue;
      if (c >= 254) { img.data[o] = 220; img.data[o + 1] = 40; img.data[o + 2] = 200; img.data[o + 3] = 120; }
      else { img.data[o] = 60; img.data[o + 1] = 120; img.data[o + 2] = 255; img.data[o + 3] = Math.round(30 + 110 * c / 252); }
    }
    ctx.putImageData(img, 0, 0);
  }
  asMap() { const g = this.grid; return { res: g.res, x0: g.x0, y0: g.y0, w: g.w, h: g.h, canvas: this.canvas }; }
  costAt(x, y) { const [i, j] = this.grid.cell(x, y); return this.grid.inside(i, j) ? this.cost[j * this.grid.w + i] : 254; }
}

// A* on the costmap (8-connected, octile heuristic). Returns world points or null.
export function astar(cm, start, goal) {
  const g = cm.grid, W = g.w, H = g.h;
  const [si, sj] = g.cell(start[0], start[1]), [gi, gj] = g.cell(goal[0], goal[1]);
  if (!g.inside(gi, gj) || cm.cost[gj * W + gi] >= 254) return { path: null, expanded: 0, reason: '목표가 장애물(또는 팽창 영역) 안입니다' };
  const N = W * H, gs = new Float32Array(N).fill(Infinity), came = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
  const heap = [];
  const push = (k, f) => { heap.push([f, k]); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
  const h = (i, j) => { const dx = Math.abs(i - gi), dy = Math.abs(j - gj); return (dx + dy) + (Math.SQRT2 - 2) * Math.min(dx, dy); };
  const s = sj * W + si;
  gs[s] = 0; push(s, h(si, sj));
  let expanded = 0;
  const nb = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  while (heap.length) {
    const [, k] = pop();
    if (closed[k]) continue;
    closed[k] = 1; expanded++;
    const i = k % W, j = (k - i) / W;
    if (i === gi && j === gj) {
      const cells = []; let c = k;
      while (c !== -1) { cells.push(c); c = came[c]; }
      cells.reverse();
      const pts = cells.map((q) => g.center(q % W, Math.floor(q / W)));
      pts[0] = [start[0], start[1]]; pts[pts.length - 1] = [goal[0], goal[1]];
      return { path: pts, expanded, closed };
    }
    for (const [di, dj, step] of nb) {
      const ni = i + di, nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
      const q = nj * W + ni;
      const c = cm.cost[q];
      if (closed[q] || (c >= 254 && q !== s)) continue;
      const ng = gs[k] + step * (1 + 3 * c / 252);
      if (ng < gs[q]) { gs[q] = ng; came[q] = k; push(q, ng + h(ni, nj)); }
    }
  }
  return { path: null, expanded, reason: '경로 없음 (막혀 있음)' };
}

// ------------------------------------------------------------ local controllers
function lookaheadPoint(path, pose, Ld, progress) {
  let best = progress, bd = Infinity;
  for (let k = progress; k < Math.min(path.length, progress + 60); k++) {
    const d = Math.hypot(path[k][0] - pose[0], path[k][1] - pose[1]);
    if (d < bd) { bd = d; best = k; }
  }
  let k = best;
  while (k < path.length - 1 && Math.hypot(path[k][0] - pose[0], path[k][1] - pose[1]) < Ld) k++;
  return { idx: best, target: path[k], last: k === path.length - 1 };
}

export function purePursuit(state, pose, p, chassis) {
  const { path } = state;
  const la = lookaheadPoint(path, pose, p.lookahead, state.progress);
  state.progress = la.idx;
  const [tx, ty] = la.target;
  const dx = tx - pose[0], dy = ty - pose[1];
  const alpha = wrap(Math.atan2(dy, dx) - pose[2]);
  const dist = Math.hypot(path[path.length - 1][0] - pose[0], path[path.length - 1][1] - pose[1]);
  let v = Math.min(p.maxV, 0.8 * dist + 0.05);
  if (chassis !== 'ackermann' && Math.abs(alpha) > 1.0) return { cmd: [0, 0, Math.sign(alpha) * Math.min(p.maxW, 1.5 * Math.abs(alpha))], target: la.target, alpha };
  v *= Math.max(0.25, Math.cos(alpha));
  const Ld = Math.max(0.1, Math.hypot(dx, dy));
  const wz = Math.max(-p.maxW, Math.min(p.maxW, 2 * v * Math.sin(alpha) / Ld));
  return { cmd: [v, 0, wz], target: la.target, alpha };
}

// DWA over (v, w) with scan points as obstacles. Returns best command + sampled trajectories.
export function dwa(state, pose, vel, scanPts, p, chassis) {
  const { path } = state;
  const la = lookaheadPoint(path, pose, Math.max(p.lookahead, 0.5), state.progress);
  state.progress = la.idx;
  const T = 1.6, dt = 0.1, win = 0.25;
  const v0 = vel[0], w0 = vel[2];
  const vmin = Math.max(chassis === 'ackermann' ? 0.05 : 0, v0 - p.accV * win), vmax = Math.min(p.maxV, v0 + p.accV * win);
  const wmin = Math.max(-p.maxW, w0 - p.accW * win), wmax = Math.min(p.maxW, w0 + p.accW * win);
  const pts = scanPts.filter(([x, y]) => Math.hypot(x - pose[0], y - pose[1]) < 2.0);
  const trajs = [];
  let best = null;
  const [gx, gy] = la.target;
  for (let a = 0; a <= 6; a++) {
    const v = vmin + (vmax - vmin) * a / 6;
    for (let b = 0; b <= 14; b++) {
      let w = wmin + (wmax - wmin) * b / 14;
      if (chassis === 'ackermann') w = Math.max(-v * Math.tan(29 * Math.PI / 180) / 0.145, Math.min(v * Math.tan(29 * Math.PI / 180) / 0.145, w));
      let [x, y, th] = pose;
      const tr = [[x, y]];
      let clear = Infinity;
      for (let t = 0; t < T; t += dt) {
        x += v * Math.cos(th) * dt; y += v * Math.sin(th) * dt; th += w * dt;
        tr.push([x, y]);
        for (const [ox, oy] of pts) { const d = Math.hypot(ox - x, oy - y); if (d < clear) clear = d; }
      }
      const ok = clear > p.robotRadius + 0.02;
      const heading = Math.PI - Math.abs(wrap(Math.atan2(gy - y, gx - x) - th));
      const goalDist = Math.hypot(gx - x, gy - y);
      const score = ok ? (0.8 * heading / Math.PI + 1.0 * Math.min(clear, 0.8) / 0.8 + 0.4 * v / p.maxV - 1.2 * goalDist) : -Infinity;
      trajs.push({ tr, ok, score });
      if (ok && (!best || score > best.score)) best = { v, w, score, tr };
    }
  }
  if (!best) return { cmd: [0, 0, chassis === 'ackermann' ? 0 : 0.8], trajs, target: la.target, recovery: true };
  return { cmd: [best.v, 0, best.w], trajs, best: best.tr, target: la.target };
}
