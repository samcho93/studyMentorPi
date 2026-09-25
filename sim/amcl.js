// amcl.js — particle filter localisation on a known map (likelihood-field model), the idea behind Nav2 AMCL.
import { distanceField } from './grid.js';
import { N_RAYS, rayAngle, wrap } from './physics.js';

export class AMCL {
  constructor(grid, rng, { n = 400, alpha = [0.2, 0.2, 0.2, 0.2], sigmaHit = 0.08, zHit = 0.9, zRand = 0.1, beams = 36, maxR = 5 } = {}) {
    this.grid = grid; this.rng = rng;
    this.field = distanceField(grid);
    Object.assign(this, { n, alpha, sigmaHit, zHit, zRand, beams, maxR });
    this.parts = [];
    this.lastOdom = null;
    this.updates = 0;
  }
  init(pose, spread = [0.3, 0.3, 0.4]) {
    const g = this.rng.gauss;
    this.parts = Array.from({ length: this.n }, () => ({ x: pose[0] + g(spread[0]), y: pose[1] + g(spread[1]), th: wrap(pose[2] + g(spread[2])), w: 1 / this.n }));
    this.lastOdom = null;
  }
  initGlobal(world) {
    const [x0, y0, x1, y1] = world.bounds, u = this.rng.uniform;
    this.parts = [];
    while (this.parts.length < this.n) {
      const x = x0 + (x1 - x0) * u(), y = y0 + (y1 - y0) * u();
      if (this.fieldAt(x, y) < 0.15) continue;
      this.parts.push({ x, y, th: -Math.PI + 2 * Math.PI * u(), w: 1 / this.n });
    }
    this.lastOdom = null;
  }
  fieldAt(x, y) {
    const [i, j] = this.grid.cell(x, y);
    if (!this.grid.inside(i, j)) return 0;
    return this.field[j * this.grid.w + i];
  }
  // sample_motion_model_odometry (Probabilistic Robotics, Table 5.6)
  motion(odom) {
    if (!this.lastOdom) { this.lastOdom = odom.slice(); return false; }
    const [px, py, pt] = this.lastOdom, [x, y, t] = odom;
    const dx = x - px, dy = y - py, trans = Math.hypot(dx, dy);
    const rot1 = trans < 0.01 ? 0 : wrap(Math.atan2(dy, dx) - pt), rot2 = wrap(t - pt - rot1);
    if (trans < 0.01 && Math.abs(wrap(t - pt)) < 0.02) return false;
    const [a1, a2, a3, a4] = this.alpha, g = this.rng.gauss;
    for (const p of this.parts) {
      const r1 = rot1 - g(Math.sqrt(a1 * rot1 * rot1 + a2 * trans * trans));
      const tr = trans - g(Math.sqrt(a3 * trans * trans + a4 * (rot1 * rot1 + rot2 * rot2)));
      const r2 = rot2 - g(Math.sqrt(a1 * rot2 * rot2 + a2 * trans * trans));
      p.x += tr * Math.cos(p.th + r1); p.y += tr * Math.sin(p.th + r1); p.th = wrap(p.th + r1 + r2);
    }
    this.lastOdom = odom.slice();
    return true;
  }
  // likelihood field measurement model
  measure(ranges) {
    const step = Math.floor(N_RAYS / this.beams), s2 = 2 * this.sigmaHit * this.sigmaHit, rnd = this.zRand / this.maxR;
    let total = 0;
    for (const p of this.parts) {
      let logw = 0;
      for (let k = 0; k < N_RAYS; k += step) {
        const r = ranges[k];
        if (!isFinite(r) || r >= this.maxR) continue;
        const a = p.th + rayAngle(k);
        const d = this.fieldAt(p.x + r * Math.cos(a), p.y + r * Math.sin(a));
        logw += Math.log(this.zHit * Math.exp(-d * d / s2) + rnd);
      }
      p.lw = logw;
    }
    const mx = Math.max(...this.parts.map((p) => p.lw));
    for (const p of this.parts) { p.w = Math.exp(p.lw - mx); total += p.w; }
    for (const p of this.parts) p.w /= total;
    this.updates++;
  }
  neff() { let s = 0; for (const p of this.parts) s += p.w * p.w; return 1 / s; }
  // low-variance (systematic) resampling
  resample() {
    const n = this.n, out = [], step = 1 / n;
    let r = this.rng.uniform() * step, c = this.parts[0].w, i = 0;
    for (let m = 0; m < n; m++) {
      const u = r + m * step;
      while (u > c && i < this.parts.length - 1) { i++; c += this.parts[i].w; }
      const p = this.parts[i];
      out.push({ x: p.x, y: p.y, th: p.th, w: step });
    }
    this.parts = out;
  }
  estimate() {
    let x = 0, y = 0, cs = 0, sn = 0;
    for (const p of this.parts) { x += p.w * p.x; y += p.w * p.y; cs += p.w * Math.cos(p.th); sn += p.w * Math.sin(p.th); }
    let vx = 0, vy = 0;
    for (const p of this.parts) { vx += p.w * (p.x - x) ** 2; vy += p.w * (p.y - y) ** 2; }
    return { pose: [x, y, Math.atan2(sn, cs)], std: Math.sqrt(vx + vy) };
  }
  // full update: returns true when a measurement update happened
  update(odom, ranges) {
    const moved = this.motion(odom);
    if (!moved && this.updates > 0) return false;
    this.measure(ranges);
    if (this.neff() < this.n / 2) this.resample();
    return true;
  }
}
