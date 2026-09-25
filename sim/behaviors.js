// behaviors.js — reactive behaviours ported from the MentorPi app package
//   app/lidar_controller.py : running_mode 1 (avoid) / 2 (follow) / 3 (guard)
//   app/line_following.py   : 3 weighted ROIs + PID on the deflection angle, lidar stop
import { N_RAYS, rayAngle, wrap } from './physics.js';

// Hiwonder sdk/pid.py: error = SetPoint(0) - feedback; output = Kp e + Ki ∫e + Kd de/dt
export class PID {
  constructor(kp, ki, kd) { this.kp = kp; this.ki = ki; this.kd = kd; this.clear(); }
  clear() { this.i = 0; this.last = 0; this.output = 0; this.first = true; }
  update(feedback, dt = 0.1) {
    const e = 0 - feedback;
    this.i += e * dt;
    const d = this.first ? 0 : (e - this.last) / dt;
    this.first = false; this.last = e;
    this.output = this.kp * e + this.ki * this.i + this.kd * d;
    return this.output;
  }
}
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// ranges within ±half (rad) of the front, as [{r, a}] with a>0 on the left
function window(ranges, half) {
  const out = [];
  for (let i = 0; i < N_RAYS; i++) {
    const a = rayAngle(i);
    if (Math.abs(a) <= half + 1e-9 && isFinite(ranges[i]) && ranges[i] > 0) out.push({ r: ranges[i], a });
  }
  return out;
}

export class LidarApp {
  constructor() {
    this.threshold = 0.6;               // m  (lidar_controller.py)
    this.scanAngle = Math.PI / 2;       // 90° window (reset_value)
    this.speed = 0.2;
    this.pidYaw = new PID(1.6, 0, 0.16);
    this.pidDist = new PID(1.7, 0, 0.16);
    this.reset();
  }
  reset() { this.hold = 0; this.lastAct = 0; this.cmd = [0, 0, 0]; this.pidYaw.clear(); this.pidDist.clear(); this.info = ''; }
  // returns [vx, vy, wz] for /controller/cmd_vel, or null = keep the previous command
  step(mode, ranges, t, chassis) {
    const w = window(ranges, this.scanAngle / 2);
    const left = w.filter((p) => p.a >= 0), right = w.filter((p) => p.a < 0);
    const minL = left.length ? Math.min(...left.map((p) => p.r)) : Infinity;
    const minR = right.length ? Math.min(...right.map((p) => p.r)) : Infinity;
    const sp = this.speed, th = this.threshold;
    if (mode === 'avoid') {
      if (t < this.hold) return null;                        // still executing the last turn
      const W = sp * 6.0;
      if (chassis === 'ackermann') {
        // Acker branch: drive forward on a 0.1 m radius, back up when both sides are blocked
        if (minL <= th && minR > th) { this.info = '왼쪽 장애물 → 오른쪽으로'; return [sp, 0, -sp / 0.1]; }
        if (minL <= th && minR <= th) { this.info = '양쪽 막힘 → 후진'; this.hold = t + 1.0; return [-0.3 / 2, 0, 0.3 / 2 / 0.1]; }
        if (minL > th && minR <= th) { this.info = '오른쪽 장애물 → 왼쪽으로'; return [sp, 0, sp / 0.1]; }
        this.info = '직진'; return [sp, 0, 0];
      }
      if (minL <= th && minR > th) {
        let wz = -W; if (this.lastAct !== 0 && this.lastAct !== 1) wz = W;
        this.lastAct = 1; this.hold = t + (Math.PI / 2) / W / 2;
        this.info = `왼쪽 ${minL.toFixed(2)} m ≤ ${th} → 회전`; return [sp / 6, 0, wz];
      }
      if (minL <= th && minR <= th) {
        this.lastAct = 3; this.hold = t + Math.PI / W / 2;
        this.info = '양쪽 막힘 → 180° 회전'; return [sp / 6, 0, W];
      }
      if (minL > th && minR <= th) {
        let wz = W; if (this.lastAct !== 0 && this.lastAct !== 2) wz = -W;
        this.lastAct = 2; this.hold = t + (Math.PI / 2) / W / 2;
        this.info = `오른쪽 ${minR.toFixed(2)} m ≤ ${th} → 회전`; return [sp / 6, 0, wz];
      }
      this.lastAct = 0; this.info = '장애물 없음 → 직진';
      return [sp, 0, 0];
    }
    // follow / guard: the nearest return in the window
    if (!w.length) { this.info = '범위 안에 물체 없음'; return [0, 0, 0]; }
    let best = w[0];
    for (const p of w) if (p.r < best.r) best = p;
    const dist = best.r, ang = best.a;
    let vx = 0, wz = 0;
    if (dist < th && Math.abs(ang) > 5 * Math.PI / 180) wz = clamp(this.pidYaw.update(-ang), -sp * 6, sp * 6);
    else this.pidYaw.clear();
    if (mode === 'follow') {
      if (dist < th && Math.abs(0.2 - dist) > 0.02) vx = clamp(this.pidDist.update(th / 2 - dist), -sp, sp);
      else this.pidDist.clear();
      if (Math.abs(vx) < 0.05) vx = 0;
    }
    if (Math.abs(wz) < 0.008) wz = 0;
    this.info = `가장 가까운 물체 ${dist.toFixed(2)} m @ ${(ang * 180 / Math.PI).toFixed(0)}°` + (dist >= th ? ' (임계값 밖 → 대기)' : '');
    if (chassis === 'ackermann') { wz = vx !== 0 ? wz : 0; }
    return [vx, 0, wz];
  }
}

// ---------------------------------------------------------------- line following
// Virtual downward-looking camera: image W×H covers the floor from NEAR to FAR metres ahead,
// lateral half-width grows linearly with distance (pinhole-like trapezoid).
export const CAM = { W: 160, H: 120, near: 0.10, far: 0.50, halfNear: 0.10, halfFar: 0.30, x: 0.06 };
const ROIS = [[0.9, 0.95, 0.7], [0.8, 0.85, 0.2], [0.7, 0.75, 0.1]];   // (y0, y1, weight) — line_following.py

export function camPixelToWorld(pose, u, v) {
  // u: 0..W (left→right), v: 0..H (top=far → bottom=near)
  const f = v / CAM.H;
  const d = CAM.far + (CAM.near - CAM.far) * f;
  const half = CAM.halfFar + (CAM.halfNear - CAM.halfFar) * f;
  const lat = (0.5 - u / CAM.W) * 2 * half;            // + = left
  const [x, y, yaw] = pose, c = Math.cos(yaw), s = Math.sin(yaw);
  const fx = CAM.x + d;
  return [x + c * fx - s * lat, y + s * fx + c * lat];
}

export class LineFollower {
  constructor() { this.pid = new PID(1.1, 0, 0); this.speed = 0.15; this.stopTh = 0.4; this.stop = false; this.count = 0; this.info = ''; this.rois = []; }
  reset() { this.pid.clear(); this.stop = false; this.count = 0; }
  // isLine(x, y) -> bool ; returns [vx, vy, wz] or [0,0,0]
  step(pose, isLine, ranges, chassis) {
    // lidar stop: scan_angle 45° total (±22.5°), stop_threshold 0.4 m
    const w = window(ranges, Math.PI / 8);
    const minD = w.length ? Math.min(...w.map((p) => p.r)) : Infinity;
    if (minD < this.stopTh) { this.stop = true; this.count = 0; } else if (++this.count > 5) { this.count = 0; this.stop = false; }
    let sum = 0, wsum = 0;
    this.rois = [];
    for (const [a, b, wt] of ROIS) {
      const v = (a + b) / 2 * CAM.H;
      let acc = 0, n = 0;
      for (let u = 0; u < CAM.W; u += 2) {
        const [px, py] = camPixelToWorld(pose, u, v);
        if (isLine(px, py)) { acc += u; n++; }
      }
      if (n) { const cx = acc / n; sum += cx * wt; this.rois.push([v, cx]); }
      wsum += wt;
    }
    if (sum === 0) { this.pid.clear(); this.info = '선을 찾지 못함 → 정지'; return [0, 0, 0]; }
    const center = sum / 1.0;                              // weight_sum = 1.0 in the original
    const defl = -Math.atan((center - CAM.W / 2) / (CAM.H / 2));
    this.center = center;
    if (this.stop) { this.info = `전방 ${minD.toFixed(2)} m < ${this.stopTh} → 정지`; return [0, 0, 0]; }
    this.pid.update(defl);
    const v = this.speed;
    let wz;
    if (chassis === 'ackermann') {
      const lim = 322 / 2000 * Math.PI;                     // radians(322/2000*180)
      const st = clamp(-this.pid.output, -lim, lim);
      wz = st !== 0 ? v / (0.145 / Math.tan(st)) : 0;
    } else wz = clamp(-this.pid.output, -1, 1);
    this.info = `선 중심 x=${center.toFixed(0)}px, 편향각 ${(defl * 180 / Math.PI).toFixed(1)}° → ωz=${wz.toFixed(2)}`;
    return [v, 0, wz];
  }
}

export { wrap };
