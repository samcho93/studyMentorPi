// physics.js — JS twin of python/mentorpi_sim (world geometry, ray casting, chassis kinematics).
// Parameters mirror MentorPi controller/mecanum.py and ackermann.py.

export const CHASSIS = {
  mecanum: { wheelbase: 0.1368, track_width: 0.1446, wheel_diameter: 0.065, radius: 0.13, length: 0.21, width: 0.20 },
  ackermann: { wheelbase: 0.145, track_width: 0.133, wheel_diameter: 0.067, radius: 0.14, length: 0.23, width: 0.19, max_steer: 29 * Math.PI / 180 },
};
export const APP_LIMIT = [0.2, 0.2, 0.5];      // /cmd_vel (app_cmd_vel_callback)
export const PHYS_LIMIT = [0.6, 0.6, 3.0];     // /controller/cmd_vel
const ACC = [1.2, 1.2, 4.0];

export const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export class World {
  constructor(spec, name) {
    this.name = name;
    this.title = spec.title;
    this.bounds = spec.bounds.slice();
    this.start = spec.start.slice();
    this.walls = spec.walls.map((w) => w.slice());
    this.boxes = spec.boxes.map((b) => b.slice());
    this.cylinders = spec.cylinders.map((c) => c.slice());
    this.movers = [];
    this.track = spec.track || null;
    this.rebuild();
  }
  rebuild() {
    const [x0, y0, x1, y1] = this.bounds;
    const s = [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0], ...this.walls];
    for (const [cx, cy, w, h] of this.boxes) {
      const a = cx - w / 2, b = cy - h / 2, c = cx + w / 2, d = cy + h / 2;
      s.push([a, b, c, b], [c, b, c, d], [c, d, a, d], [a, d, a, b]);
    }
    this.segs = s;
  }
  circles() { return [...this.cylinders, ...this.movers.map((m) => [m[0], m[1], m[4]])]; }
  stepMovers(dt) {
    const [x0, y0, x1, y1] = this.bounds;
    for (const m of this.movers) {
      m[0] += m[2] * dt; m[1] += m[3] * dt;
      if (m[0] - m[4] < x0 || m[0] + m[4] > x1) m[2] = -m[2];
      if (m[1] - m[4] < y0 || m[1] + m[4] > y1) m[3] = -m[3];
      for (const [cx, cy, w, h] of this.boxes) {        // bounce off boxes too
        if (Math.abs(m[0] - cx) < w / 2 + m[4] && Math.abs(m[1] - cy) < h / 2 + m[4]) { m[2] = -m[2]; m[3] = -m[3]; m[0] += m[2] * dt * 2; m[1] += m[3] * dt * 2; }
      }
    }
  }
  ray(x, y, a, maxR = 12) {
    const dx = Math.cos(a), dy = Math.sin(a);
    let best = Infinity;
    for (const [sx0, sy0, sx1, sy1] of this.segs) {
      const ex = sx1 - sx0, ey = sy1 - sy0, qx = sx0 - x, qy = sy0 - y;
      const den = dx * ey - dy * ex;
      if (Math.abs(den) < 1e-12) continue;
      const t = (qx * ey - qy * ex) / den, u = (qx * dy - qy * dx) / den;
      if (t > 1e-6 && u >= 0 && u <= 1 && t < best) best = t;
    }
    for (const [cx, cy, r] of this.circles()) {
      const fx = x - cx, fy = y - cy;
      const b = fx * dx + fy * dy, c = fx * fx + fy * fy - r * r, disc = b * b - c;
      if (disc < 0) continue;
      const t = -b - Math.sqrt(disc);
      if (t > 1e-6 && t < best) best = t;
    }
    return best > maxR ? Infinity : best;
  }
  collides(x, y, rad) {
    for (const [px, py, qx, qy] of this.segs) {
      const ex = qx - px, ey = qy - py, L2 = ex * ex + ey * ey;
      const t = L2 > 0 ? clamp(((x - px) * ex + (y - py) * ey) / L2, 0, 1) : 0;
      const dx = px + t * ex - x, dy = py + t * ey - y;
      if (dx * dx + dy * dy < rad * rad) return true;
    }
    for (const [cx, cy, r] of this.circles()) if ((x - cx) ** 2 + (y - cy) ** 2 < (r + rad) ** 2) return true;
    return false;
  }
}

// deterministic gaussian noise
export function makeRng(seed = 7) {
  let s = seed >>> 0;
  const u = () => { s = (s * 1664525 + 1013904223) >>> 0; return (s + 0.5) / 4294967296; };
  return { uniform: u, gauss: (sd = 1) => sd * Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u()) };
}

export class Robot {
  constructor(world, chassis, start, noise, rng) {
    this.world = world;
    this.setChassis(chassis);
    [this.x, this.y, this.yaw] = start;
    this.start = start.slice();
    this.ox = 0; this.oy = 0; this.oyaw = 0;
    this.cmd = [0, 0, 0];
    this.v = [0, 0, 0];
    this.steer = 0;
    this.noise = noise;
    this.rng = rng;
    this.collisions = 0; this.distance = 0; this.inContact = false; this.lastHit = -10; this.t = 0;
  }
  setChassis(ch) { this.chassis = ch; this.p = CHASSIS[ch]; }
  setCmd(vx, vy, wz, limit = APP_LIMIT) {
    this.cmd = [clamp(vx, -limit[0], limit[0]), this.chassis === 'ackermann' ? 0 : clamp(vy, -limit[1], limit[1]), clamp(wz, -limit[2], limit[2])];
  }
  // wheel speeds [rps] in the order of MentorPi motors 1..4 (mecanum) or steering info (ackermann)
  wheels() {
    const [vx, vy, wz] = this.v;
    if (this.chassis === 'mecanum') {
      const k = (this.p.wheelbase + this.p.track_width) / 2, c = Math.PI * this.p.wheel_diameter;
      const m1 = vx - vy - wz * k, m2 = vx + vy - wz * k, m3 = vx + vy + wz * k, m4 = vx - vy + wz * k;
      return { rps: [-m1 / c, -m2 / c, m3 / c, m4 / c], lin: [m1, m2, m3, m4] };
    }
    const c = Math.PI * this.p.wheel_diameter, T = this.p.track_width;
    const vr = vx + wz * T / 2, vl = vx - wz * T / 2;
    const deg = this.steer * 180 / Math.PI;
    return { rps: [0, vl / c, 0, -vr / c], lin: [vl, vr], steerDeg: deg, servo: Math.round(1500 + 2000 * (-deg) / 180) };
  }
  step(dt) {
    const tgt = this.cmd.slice();
    if (this.chassis === 'ackermann') {
      const L = this.p.wheelbase, v = tgt[0];
      if (Math.abs(v) > 1e-6 && Math.abs(tgt[2]) > 1e-6) this.steer = clamp(Math.atan(L * tgt[2] / v), -this.p.max_steer, this.p.max_steer);
      else if (Math.abs(v) > 1e-6) this.steer = 0;
    }
    for (let i = 0; i < 3; i++) this.v[i] += clamp(tgt[i] - this.v[i], -ACC[i] * dt, ACC[i] * dt);
    if (this.chassis === 'ackermann') { this.v[1] = 0; this.v[2] = this.v[0] * Math.tan(this.steer) / this.p.wheelbase; }
    const [vx, vy, wz] = this.v;
    let c = Math.cos(this.oyaw), s = Math.sin(this.oyaw);
    this.ox += (vx * c - vy * s) * dt; this.oy += (vx * s + vy * c) * dt; this.oyaw = wrap(this.oyaw + wz * dt);
    const g = this.noise ? this.rng.gauss : () => 0;
    const sl = this.noise ? 0.95 : 1, sa = this.noise ? 0.97 : 1;
    const tvx = vx * sl + g(0.003), tvy = vy * sl + g(0.003), twz = wz * sa + g(0.004);
    c = Math.cos(this.yaw); s = Math.sin(this.yaw);
    let nx = this.x + (tvx * c - tvy * s) * dt, ny = this.y + (tvx * s + tvy * c) * dt;
    const nyaw = wrap(this.yaw + twz * dt);
    const col = (a, b) => this.world.collides(a, b, this.p.radius);
    this.t += dt;
    if (col(nx, ny)) {
      if (!col(nx, this.y)) ny = this.y;
      else if (!col(this.x, ny)) nx = this.x;
      else { nx = this.x; ny = this.y; this.v[0] = this.v[1] = 0; }
      if (!this.inContact && this.t - this.lastHit > 1) this.collisions++;
      this.inContact = true; this.lastHit = this.t;
      if (col(nx, ny)) { nx = this.x; ny = this.y; }
    } else this.inContact = false;
    this.distance += Math.hypot(nx - this.x, ny - this.y);
    this.x = nx; this.y = ny; this.yaw = nyaw;
  }
  odomWorld() {
    const [sx, sy, syaw] = this.start, c = Math.cos(syaw), s = Math.sin(syaw);
    return [sx + c * this.ox - s * this.oy, sy + s * this.ox + c * this.oy, wrap(syaw + this.oyaw)];
  }
}

// 360-ray scan, index 0 = -pi (behind), 180 = front, CCW positive (like LaserScan in the Playground)
export const N_RAYS = 360;
export function scan(world, pose, noise, rng) {
  const out = new Array(N_RAYS);
  for (let i = 0; i < N_RAYS; i++) {
    let r = world.ray(pose[0], pose[1], pose[2] - Math.PI + i * 2 * Math.PI / N_RAYS);
    if (noise && isFinite(r)) r += rng.gauss(0.008);
    out[i] = r < 0.05 ? Infinity : r;
  }
  return out;
}
export const rayAngle = (i) => -Math.PI + i * 2 * Math.PI / N_RAYS;
