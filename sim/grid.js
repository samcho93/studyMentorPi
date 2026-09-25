// grid.js — occupancy grids: SLAM-style log-odds mapping, static map rasterisation,
// distance transform (for inflation, AMCL likelihood field) and canvas rendering.
import { N_RAYS, rayAngle } from './physics.js';

export class Grid {
  constructor(bounds, res = 0.05, pad = 0.2) {
    this.res = res;
    this.x0 = bounds[0] - pad; this.y0 = bounds[1] - pad;
    this.w = Math.ceil((bounds[2] - bounds[0] + 2 * pad) / res);
    this.h = Math.ceil((bounds[3] - bounds[1] + 2 * pad) / res);
    this.lo = new Float32Array(this.w * this.h);   // log-odds, 0 = unknown
    this.canvas = document.createElement('canvas');
    this.canvas.width = this.w; this.canvas.height = this.h;
    this.dirty = true;
  }
  cell(x, y) { return [Math.floor((x - this.x0) / this.res), Math.floor((y - this.y0) / this.res)]; }
  center(i, j) { return [this.x0 + (i + 0.5) * this.res, this.y0 + (j + 0.5) * this.res]; }
  inside(i, j) { return i >= 0 && j >= 0 && i < this.w && j < this.h; }
  idx(i, j) { return j * this.w + i; }
  prob(k) { return 1 - 1 / (1 + Math.exp(this.lo[k])); }
  clear() { this.lo.fill(0); this.dirty = true; }

  // log-odds update with one scan from pose (inverse sensor model, Bresenham ray)
  integrate(pose, ranges, { lOcc = 0.85, lFree = -0.4, maxR = 5, stride = 2 } = {}) {
    const [x, y, yaw] = pose;
    const [ci, cj] = this.cell(x, y);
    for (let k = 0; k < N_RAYS; k += stride) {
      let r = ranges[k];
      const hit = isFinite(r) && r < maxR;
      if (!hit) r = maxR;
      const a = yaw + rayAngle(k);
      const [ei, ej] = this.cell(x + r * Math.cos(a), y + r * Math.sin(a));
      this.bresenham(ci, cj, ei, ej, (i, j) => {
        if (this.inside(i, j)) { const q = this.idx(i, j); this.lo[q] = Math.max(-6, this.lo[q] + lFree); }
      });
      if (hit && this.inside(ei, ej)) { const q = this.idx(ei, ej); this.lo[q] = Math.min(8, this.lo[q] + lOcc - lFree); }
    }
    this.dirty = true;
  }
  bresenham(x0, y0, x1, y1, fn) {
    let dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (let n = 0; n < 4000; n++) {
      fn(x0, y0);
      if (x0 === x1 && y0 === y1) break;
      const e2 = 2 * err;
      if (e2 >= dy) { err += dy; x0 += sx; }
      if (e2 <= dx) { err += dx; y0 += sy; }
    }
  }
  // fill from world geometry (perfect "known map")
  fromWorld(world) {
    for (let j = 0; j < this.h; j++) for (let i = 0; i < this.w; i++) {
      const [x, y] = this.center(i, j);
      const [bx0, by0, bx1, by1] = world.bounds;
      const outside = x < bx0 || y < by0 || x > bx1 || y > by1;
      this.lo[this.idx(i, j)] = outside || world.collides(x, y, this.res * 0.55) ? 8 : -6;
    }
    this.dirty = true;
    return this;
  }
  // occupancy (0 free, 1 occupied, -1 unknown) with thresholds
  state(k) { const v = this.lo[k]; return v > 0.6 ? 1 : v < -0.6 ? 0 : -1; }
  stats() {
    let occ = 0, free = 0;
    for (let k = 0; k < this.lo.length; k++) { const s = this.state(k); if (s === 1) occ++; else if (s === 0) free++; }
    return { occ, free, unknown: this.lo.length - occ - free };
  }
  render(colors = {}) {
    if (!this.dirty) return this.canvas;
    const ctx = this.canvas.getContext('2d');
    const img = ctx.createImageData(this.w, this.h);
    const d = img.data;
    for (let j = 0; j < this.h; j++) for (let i = 0; i < this.w; i++) {
      const k = this.idx(i, j), p = this.prob(k);
      const o = ((this.h - 1 - j) * this.w + i) * 4;          // flip: row 0 = top = max y
      const g = Math.round(255 * (1 - p));
      if (Math.abs(this.lo[k]) < 0.05) { d[o] = 150; d[o + 1] = 160; d[o + 2] = 175; d[o + 3] = 70; continue; }
      d[o] = g; d[o + 1] = g; d[o + 2] = g; d[o + 3] = colors.alpha ?? 200;
    }
    ctx.putImageData(img, 0, 0);
    this.dirty = false;
    return this.canvas;
  }
  asMap() { return { res: this.res, x0: this.x0, y0: this.y0, w: this.w, h: this.h, canvas: this.render() }; }
  toPGM() {
    // ROS map_server style: 254 free, 0 occupied, 205 unknown (row 0 = top)
    const head = `P5\n# studyMentorPi 2D sim map, resolution ${this.res}, origin [${this.x0.toFixed(3)}, ${this.y0.toFixed(3)}, 0]\n${this.w} ${this.h}\n255\n`;
    const body = new Uint8Array(this.w * this.h);
    for (let j = 0; j < this.h; j++) for (let i = 0; i < this.w; i++) {
      const s = this.state(this.idx(i, j));
      body[(this.h - 1 - j) * this.w + i] = s === 1 ? 0 : s === 0 ? 254 : 205;
    }
    const enc = new TextEncoder().encode(head);
    const out = new Uint8Array(enc.length + body.length); out.set(enc); out.set(body, enc.length);
    return out;
  }
}

// Euclidean-ish distance transform (two-pass chamfer 3-4) in metres to the nearest occupied cell.
export function distanceField(grid, unknownIsObstacle = false) {
  const { w, h } = grid;
  const INF = 1e9;
  const d = new Float32Array(w * h);
  for (let k = 0; k < w * h; k++) {
    const s = grid.state(k);
    d[k] = s === 1 || (unknownIsObstacle && s === -1) ? 0 : INF;
  }
  const a = 1, b = Math.SQRT2;
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
    const k = j * w + i; let v = d[k];
    if (i > 0) v = Math.min(v, d[k - 1] + a);
    if (j > 0) { v = Math.min(v, d[k - w] + a); if (i > 0) v = Math.min(v, d[k - w - 1] + b); if (i < w - 1) v = Math.min(v, d[k - w + 1] + b); }
    d[k] = v;
  }
  for (let j = h - 1; j >= 0; j--) for (let i = w - 1; i >= 0; i--) {
    const k = j * w + i; let v = d[k];
    if (i < w - 1) v = Math.min(v, d[k + 1] + a);
    if (j < h - 1) { v = Math.min(v, d[k + w] + a); if (i < w - 1) v = Math.min(v, d[k + w + 1] + b); if (i > 0) v = Math.min(v, d[k + w - 1] + b); }
    d[k] = v;
  }
  for (let k = 0; k < w * h; k++) d[k] = Math.min(d[k], 1e4) * grid.res;
  return d;
}
