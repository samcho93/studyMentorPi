"""Run Playground code in CPython exactly like tools/playground-worker.js does.

    python tests/pgrun.py python/examples/03_cmd_vel_square.py
    python tests/pgrun.py --lessons          # every ```python run block in content/**/*.md
"""
import io
import json
import os
import re
import sys
import time
import traceback
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PY = ROOT / "python"
sys.path.insert(0, str(PY))
for s in (sys.stdout, sys.stderr):
    if hasattr(s, "reconfigure"):
        s.reconfigure(encoding="utf-8", errors="replace")

import mentorpi_sim  # noqa: E402

_real_monotonic = time.monotonic


def run_source(source, name="main.py"):
    """Returns (status, stdout, stderr, result_dict, wall_seconds)."""
    os.chdir(PY / "samples")
    mentorpi_sim.begin_run()
    out, err = io.StringIO(), io.StringIO()
    status = "ok"
    t0 = _real_monotonic()
    with redirect_stdout(out), redirect_stderr(err):
        if "cv2" in source:
            try:
                import cv2
                mentorpi_sim.install_cv2_hooks(cv2)
            except Exception as e:  # pragma: no cover
                sys.stderr.write("cv2 load failed: %s\n" % e)
        try:
            code = compile(source, name, "exec")
            exec(code, {"__name__": "__main__", "__file__": name})
        except SystemExit as e:
            status = "ok" if e.code in (None, 0) else "error"
        except mentorpi_sim.SimTimeUp:
            status = "ok"
        except BaseException as exc:
            tb = [f for f in traceback.extract_tb(exc.__traceback__) if f.filename == name]
            lines = ["Traceback (most recent call last):\n"] + traceback.format_list(tb)
            lines += traceback.format_exception_only(type(exc), exc)
            sys.stderr.write("".join(lines))
            status = "error"
    wall = _real_monotonic() - t0
    res = json.loads(mentorpi_sim.export_json())
    return status, out.getvalue(), err.getvalue(), res, wall


def lesson_blocks():
    for md in sorted((ROOT / "content").rglob("*.md")):
        text = md.read_text(encoding="utf-8").replace("\r\n", "\n")
        for k, m in enumerate(re.finditer(r"^```python run[^\n]*\n(.*?)^```", text, re.M | re.S)):
            line = text[:m.start()].count("\n") + 1
            yield "%s:%d" % (md.relative_to(ROOT).as_posix(), line), m.group(1)


def main(argv):
    quiet = "-q" in argv
    argv = [a for a in argv if a != "-q"]
    if argv and argv[0] == "--lessons":
        items = list(lesson_blocks())
        if len(argv) > 1:
            items = [it for it in items if any(f in it[0] for f in argv[1:])]
    else:
        items = [(p, Path(p).read_text(encoding="utf-8")) for p in argv]
    bad = 0
    for name, src in items:
        status, out, err, res, wall = run_source(src)
        s = res["summary"]
        tag = "OK " if status == "ok" and not err.strip() else "ERR"
        if tag == "ERR":
            bad += 1
        print("=== [%s] %s  (wall %.1fs, sim %.1fs, collisions %d, images %d)" % (
            tag, name, wall, s["t"], s["collisions"], len(res.get("images", []))))
        if not quiet or tag == "ERR":
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print("--- stderr ---\n" + err.rstrip())
    print("\n%d/%d failed" % (bad, len(items)))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
