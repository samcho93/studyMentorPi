"""Simulated clock, message bus and virtual MentorPi for the ROS 2 Playground.

Design: one global Runtime. Simulated time only advances when user code calls
rclpy.spin / spin_once / time.sleep. Physics runs at 50 Hz; sensors publish on the
same bus that user nodes subscribe to, so user code is ordinary rclpy code.
"""
import copy
import math
import random

import numpy as np

from .world import World, wrap

DT = 0.02                       # physics step [s]

CHASSIS = {
    # values from MentorPi controller/mecanum.py & ackermann.py (odom_publisher_node)
    "mecanum": {"wheelbase": 0.1368, "track_width": 0.1446, "wheel_diameter": 0.065,
                "radius": 0.13, "length": 0.21, "width": 0.20},
    "ackermann": {"wheelbase": 0.145, "track_width": 0.133, "wheel_diameter": 0.067,
                  "radius": 0.14, "length": 0.23, "width": 0.19, "max_steer": math.radians(29)},
}
APP_LIMIT = (0.2, 0.2, 0.5)     # /cmd_vel clamp (app_cmd_vel_callback)
PHYS_LIMIT = (0.6, 0.6, 3.0)    # hardware ceiling for /controller/cmd_vel
ACC = (1.2, 1.2, 4.0)           # accel limits [m/s^2, m/s^2, rad/s^2]


class SimTimeUp(KeyboardInterrupt):
    """Raised when simulated time reaches `duration` inside sleep/spin_once loops.
    Subclasses KeyboardInterrupt so the usual `except KeyboardInterrupt:` + `finally:`
    shutdown pattern of ROS 2 programs runs unchanged."""


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class Robot:
    def __init__(self, rt, chassis, start, noise):
        self.rt = rt
        self.chassis = chassis
        self.p = CHASSIS[chassis]
        self.x, self.y, self.yaw = start           # ground truth
        self.ox, self.oy, self.oyaw = 0.0, 0.0, 0.0  # odometry (odom frame starts at 0)
        self.cmd = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]                   # actual body velocity
        self.steer = 0.0
        self.noise = noise
        # wheel slip: robot really moves a bit less than the encoders report
        self.slip_lin = 0.95 if noise else 1.0
        self.slip_ang = 0.97 if noise else 1.0
        self.collisions = 0
        self.distance = 0.0
        self._in_contact = False
        self._last_hit = -10.0

    def set_cmd(self, twist, limit):
        vx = max(-limit[0], min(limit[0], float(twist.linear.x)))
        vy = max(-limit[1], min(limit[1], float(twist.linear.y)))
        wz = max(-limit[2], min(limit[2], float(twist.angular.z)))
        if self.chassis == "ackermann":
            vy = 0.0
        self.cmd = [vx, vy, wz]

    def step(self, dt):
        tgt = list(self.cmd)
        if self.chassis == "ackermann":
            L = self.p["wheelbase"]
            v = tgt[0]
            if abs(v) > 1e-6 and abs(tgt[2]) > 1e-6:
                d = math.atan(L * tgt[2] / v)
                d = max(-self.p["max_steer"], min(self.p["max_steer"], d))
            else:
                d = 0.0 if abs(v) > 1e-6 else self.steer
            self.steer = d
        for i in range(3):
            dv = tgt[i] - self.v[i]
            lim = ACC[i] * dt
            self.v[i] += max(-lim, min(lim, dv))
        if self.chassis == "ackermann":
            self.v[1] = 0.0
            self.v[2] = self.v[0] * math.tan(self.steer) / self.p["wheelbase"]
        vx, vy, wz = self.v
        # odometry integrates what the encoders report
        c, s = math.cos(self.oyaw), math.sin(self.oyaw)
        self.ox += (vx * c - vy * s) * dt
        self.oy += (vx * s + vy * c) * dt
        self.oyaw = wrap(self.oyaw + wz * dt)
        # ground truth: slip + small random noise
        rn = (lambda sd: random.gauss(0, sd)) if self.noise else (lambda sd: 0.0)
        tvx = vx * self.slip_lin + rn(0.003)
        tvy = vy * self.slip_lin + rn(0.003)
        twz = wz * self.slip_ang + rn(0.004)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        nx = self.x + (tvx * c - tvy * s) * dt
        ny = self.y + (tvx * s + tvy * c) * dt
        nyaw = wrap(self.yaw + twz * dt)
        col = self.rt.world.collides
        rad = self.p["radius"]
        if col(nx, ny, rad):
            # slide along the obstacle if one axis is free, otherwise stop
            if not col(nx, self.y, rad):
                ny = self.y
            elif not col(self.x, ny, rad):
                nx = self.x
            else:
                nx, ny = self.x, self.y
                self.v[0] = self.v[1] = 0.0
            if not self._in_contact and self.rt.t - self._last_hit > 1.0:
                self.collisions += 1
                self.rt.log_event("충돌! (%.2f, %.2f) — 장애물에 닿았습니다" % (self.x, self.y))
            self._in_contact = True
            self._last_hit = self.rt.t
            if col(nx, ny, rad):
                nx, ny = self.x, self.y
        else:
            self._in_contact = False
        self.distance += math.hypot(nx - self.x, ny - self.y)
        self.x, self.y, self.yaw = nx, ny, nyaw


class Runtime:
    def __init__(self):
        self.reset()

    # ---------------------------------------------------------------- setup
    def reset(self, chassis="mecanum", world="room", duration=20.0, start=None, noise=True, seed=None):
        if chassis not in CHASSIS:
            raise ValueError("chassis must be 'mecanum' or 'ackermann'")
        random.seed(seed if seed is not None else 7)
        self.rng = np.random.default_rng(seed if seed is not None else 7)
        self.t = 0.0
        self.duration = float(duration)
        self.world = World(world)
        st = tuple(start) if start is not None else self.world.start
        self.start = st
        self.robot = Robot(self, chassis, st, noise)
        self.noise = noise
        self.subs = {}            # topic -> [Subscription]
        self.internal = {}        # topic -> [callable] (virtual robot inputs)
        self.nodes = []
        self.services = {}
        self.actions = {}
        self.timers = []
        self.soon = []
        self.shutdown_flag = False
        self.inited = False
        self.timeup = False
        self.frames = []
        self.events = []
        self.plots = []
        self.images = []
        self.tf = {}              # child -> (parent, (x,y,z), (qx,qy,qz,qw))
        # fixed frames from mentorpi_description (mecanum.xacro)
        self.static_tf = {
            "base_link": ("base_footprint", (0.0, 0.0, 0.07), (0.0, 0.0, 0.0, 1.0)),
            "lidar_frame": ("base_link", (-0.012242, -0.00008533, 0.092501), (0.0, 0.0, 0.0, 1.0)),
            "imu_link": ("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, math.sin(-1.57 / 2), math.cos(-1.57 / 2))),
            "depth_cam": ("base_link", (0.061376, -0.00013463, 0.051154), (0.0, 0.0, 0.0, 1.0)),
        }
        self._next_odom = 0.0
        self._next_scan = 0.0
        self._next_frame = 0.0
        self._last_scan = None
        self._pubs_seen = set()
        self.configured = False
        self.internal["/cmd_vel"] = [lambda m: self.robot.set_cmd(m, APP_LIMIT)]
        self.internal["/controller/cmd_vel"] = [lambda m: self.robot.set_cmd(m, PHYS_LIMIT)]

    def log_event(self, text):
        self.events.append((round(self.t, 2), text))
        print("[sim %.2f s] %s" % (self.t, text))

    # ---------------------------------------------------------------- bus
    def publish(self, topic, msg):
        self._pubs_seen.add(topic)
        for cb in self.internal.get(topic, []):
            cb(msg)
        for sub in self.subs.get(topic, []):
            sub._enqueue(copy.deepcopy(msg))
        if topic == "/tf" or topic == "/tf_static":
            tfs = getattr(msg, "transforms", None) or [msg]
            for ts in tfs:
                self._store_tf(ts, static=(topic == "/tf_static"))

    def _store_tf(self, ts, static=False):
        tr = ts.transform
        rec = (ts.header.frame_id, (tr.translation.x, tr.translation.y, tr.translation.z),
               (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))
        (self.static_tf if static else self.tf)[ts.child_frame_id] = rec

    # ---------------------------------------------------------------- time
    def ok(self):
        return self.inited and not self.shutdown_flag and not self.timeup

    def _check_time(self):
        if self.t >= self.duration - 1e-9:
            if not self.timeup:
                self.timeup = True
                print("[sim] 시뮬레이션 시간 %.1f s 종료 (sim.setup(duration=...)으로 변경)" % self.duration)
            raise SimTimeUp()

    def advance(self, seconds, run_callbacks=False):
        """Advance simulated time by `seconds` (physics + sensors)."""
        end = self.t + max(0.0, seconds)
        while self.t < end - 1e-9:
            self._check_time()
            if run_callbacks:
                self.run_ready()
            self._physics(min(DT, end - self.t))

    def _physics(self, h):
        self.robot.step(h)
        self.world.step_movers(h)
        self.t = round(self.t + h, 9)
        self._sensors()

    def _sensors(self):
        from . import bridge
        r = self.robot
        if self.t + 1e-9 >= self._next_odom:
            self._next_odom += 0.02
            bridge.publish_odom(self, r)
        if self.t + 1e-9 >= self._next_scan:
            self._next_scan += 0.1
            self._last_scan = bridge.publish_scan(self, r)
            gt = bridge.Pose2D(x=r.x, y=r.y, theta=r.yaw)
            self.publish("/ground_truth", gt)
        if self.t + 1e-9 >= self._next_frame:
            self._next_frame += 0.1
            self._record()

    def _record(self):
        r = self.robot
        scan = self._last_scan
        pts = []
        if scan is not None:
            # every 2nd ray, rounded, None for inf
            pts = [None if not math.isfinite(v) else round(v, 3) for v in scan[::2]]
        self.frames.append({
            "t": round(self.t, 2),
            "gt": [round(r.x, 4), round(r.y, 4), round(r.yaw, 4)],
            "od": [round(r.ox, 4), round(r.oy, 4), round(r.oyaw, 4)],
            "cmd": [round(v, 3) for v in r.cmd],
            "steer": round(r.steer, 3),
            "scan": pts,
            "movers": [[round(m[0], 3), round(m[1], 3), m[4]] for m in self.world.movers],
        })

    # ---------------------------------------------------------------- executor
    def ready_items(self):
        items = []
        for tm in self.timers:
            if not tm.cancelled and tm.next_t <= self.t + 1e-9:
                items.append((tm.next_t, 0, tm))
        for subs in self.subs.values():
            for s in subs:
                if s.queue:
                    items.append((s.queue[0][0], 1, s))
        items.sort(key=lambda it: (it[0], it[1]))
        return items

    def run_one(self):
        if self.soon:
            self.soon.pop(0)()
            return True
        items = self.ready_items()
        if not items:
            return False
        _, kind, obj = items[0]
        if kind == 0:
            obj.next_t += obj.period
            obj.callback()
        else:
            _, m = obj.queue.pop(0)
            obj.callback(m)
        return True

    def run_ready(self, limit=500):
        n = 0
        while n < limit and self.run_one():
            n += 1

    def spin(self):
        while not self.shutdown_flag:
            if self.t >= self.duration - 1e-9:
                if not self.timeup:
                    self.timeup = True
                    print("[sim] 시뮬레이션 시간 %.1f s 종료 (sim.setup(duration=...)으로 변경)" % self.duration)
                return
            self.run_ready()
            self._physics(DT)

    def spin_once(self, timeout_sec=None):
        self._check_time()
        if self.run_one():
            return
        budget = 0.1 if timeout_sec is None else max(0.0, timeout_sec)
        end = self.t + budget
        while self.t < end - 1e-9:
            self._physics(min(DT, end - self.t))
            if self.run_one():
                return
            self._check_time()
        if budget == 0:
            return

    # ---------------------------------------------------------------- export
    def export(self):
        r = self.robot
        return {
            "world": self.world.to_dict(),
            "chassis": r.chassis,
            "start": list(self.start),
            "geom": {k: v for k, v in r.p.items() if isinstance(v, (int, float))},
            "frames": self.frames,
            "events": self.events,
            "plots": self.plots,
            "summary": {
                "t": round(self.t, 2),
                "distance": round(r.distance, 3),
                "collisions": r.collisions,
                "gt": [round(r.x, 3), round(r.y, 3), round(math.degrees(r.yaw), 1)],
                "odom": [round(r.ox, 3), round(r.oy, 3), round(math.degrees(r.oyaw), 1)],
                "used_sim": bool(self.frames),
            },
        }


RT = Runtime()
