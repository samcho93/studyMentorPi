"""mentorpi_sim — browser-side virtual MentorPi for the studyMentorPi ROS 2 Playground.

    import mentorpi_sim as sim          # 시뮬 전용
    sim.setup(chassis="mecanum", world="room", duration=20.0)

See docs/PLAYGROUND_API.md. On a real robot, delete the two lines above; everything
else in a Playground program is ordinary rclpy code.
"""
import base64
import json
import math
import time as _time

from .core import RT, CHASSIS, SimTimeUp  # noqa: F401
from .world import load_worlds

__all__ = ["setup", "add_box", "add_cylinder", "add_wall", "add_mover", "plot", "pose",
           "world_names", "SimTimeUp", "CHASSIS"]

_real_sleep = _time.sleep
_real_time = _time.time
_real_monotonic = _time.monotonic
_EPOCH = 1_750_000_000.0


def setup(chassis="mecanum", world="room", duration=20.0, start=None, noise=True, seed=None):
    """Configure the virtual robot. Call before creating nodes."""
    subs, nodes = dict(RT.subs), list(RT.nodes)
    inited = RT.inited
    RT.reset(chassis=chassis, world=world, duration=duration, start=start, noise=noise, seed=seed)
    RT.subs.update(subs)          # keep subscriptions made before setup()
    RT.nodes.extend(nodes)
    RT.inited = inited
    RT.configured = True


def world_names():
    return list(load_worlds().keys())


def add_box(x, y, w, h):
    RT.world.add_box(x, y, w, h)


def add_cylinder(x, y, r=0.1):
    RT.world.add_cylinder(x, y, r)


def add_wall(x1, y1, x2, y2):
    RT.world.add_wall(x1, y1, x2, y2)


def add_mover(x, y, vx=0.0, vy=0.0, r=0.1):
    """A moving cylinder that bounces off the outer walls (target for lidar following)."""
    RT.world.add_mover(x, y, vx, vy, r)


def pose():
    """Ground-truth pose (x, y, yaw) — simulation only."""
    r = RT.robot
    return (r.x, r.y, r.yaw)


def plot(xs, ys=None, label=None):
    """Add a line to the Playground result chart."""
    if ys is None:
        ys, xs = list(xs), list(range(len(xs)))
    RT.plots.append({"label": label or "series %d" % (len(RT.plots) + 1),
                     "x": [_fin(v) for v in xs], "y": [_fin(v) for v in ys]})


def _fin(v):
    v = float(v)
    return v if math.isfinite(v) else None


# ---------------------------------------------------------------- time patch
def _sim_active():
    return RT.inited or RT.configured


def _sleep(seconds):
    if seconds is None or seconds <= 0:
        return
    if _sim_active():
        RT.advance(float(seconds))
    else:
        RT.t += float(seconds)      # plain scripts: no physics, just a clock


def _time_now():
    return _EPOCH + RT.t


def _monotonic():
    return 1000.0 + RT.t


def install_time_patch():
    _time.sleep = _sleep
    _time.time = _time_now
    _time.monotonic = _monotonic
    _time.perf_counter = _monotonic


# ---------------------------------------------------------------- cv2 hooks
def install_cv2_hooks(cv2):
    def imshow(name, img):
        if len(RT.images) >= 16:
            return
        import numpy as np
        a = np.asarray(img)
        if a.dtype != np.uint8:
            a = np.clip(a, 0, 255).astype(np.uint8)
        ok, buf = cv2.imencode(".png", a)
        if ok:
            RT.images.append({"name": str(name), "png": base64.b64encode(buf.tobytes()).decode("ascii"),
                              "shape": list(a.shape)})

    cv2.imshow = imshow
    cv2.waitKey = lambda *a, **k: -1
    cv2.destroyAllWindows = lambda *a, **k: None
    cv2.destroyWindow = lambda *a, **k: None
    cv2.namedWindow = lambda *a, **k: None
    cv2.moveWindow = lambda *a, **k: None
    cv2.resizeWindow = lambda *a, **k: None


# ---------------------------------------------------------------- run lifecycle
def begin_run():
    """Fresh state for a new Playground run."""
    RT.reset()
    install_time_patch()


def export_json():
    data = RT.export()
    data["summary"]["used_sim"] = bool(RT.inited or RT.configured)
    data["images"] = RT.images
    return json.dumps(data, allow_nan=False, default=lambda o: None)
