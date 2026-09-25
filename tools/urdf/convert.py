"""Convert the MentorPi URDF (mentorpi_description, xacro) into browser-friendly assets.

  python tools/urdf/convert.py "D:/MentorPi/Appendix/Source Code/src/simulations/mentorpi_description"

- expands the two xacro variants (mecanum.xacro / ack.xacro + imu.urdf.xacro) into plain URDF
- decimates every STL by vertex clustering (numpy only) and writes binary STL
- output: assets/urdf/mentorpi/{mentorpi_mecanum.urdf, mentorpi_ackermann.urdf, meshes/<variant>/*.stl}
Meshes: Hiwonder mentorpi_description (see assets/urdf/mentorpi/README.md).
"""
import math
import re
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "assets" / "urdf" / "mentorpi"
CELL = {"wheel": 0.0030, "cam": 0.0022, "default": 0.0014}   # clustering cell size [m]


def read_stl(path):
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    if 84 + n * 50 == len(data):
        rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
        return rec["v"].astype(np.float64)
    tris = [list(map(float, m)) for m in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)]
    return np.array(tris, dtype=np.float64).reshape(-1, 3, 3)


def decimate(tris, cell):
    v = tris.reshape(-1, 3)
    key = np.floor(v / cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inv, v)
    cnt = np.bincount(inv, minlength=len(uniq))[:, None]
    verts = sums / cnt
    f = inv.reshape(-1, 3)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[ok]
    fs = np.sort(f, axis=1)
    _, first = np.unique(fs, axis=0, return_index=True)
    f = f[np.sort(first)]
    return verts[f]


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    nrm = np.cross(b - a, c - a)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.where(ln > 0, nrm / np.where(ln > 0, ln, 1), 0).astype(np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = nrm, tris
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"studyMentorPi decimated mesh".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def expand(src_dir, xacro_name, variant):
    text = (src_dir / "urdf" / xacro_name).read_text(encoding="utf-8", errors="replace")
    if "imu.urdf.xacro" in text:
        z = 0.025 - 0.0295 + 0.0965 / 2 + 0.0678410821576746
        imu = ('  <link name="imu_link"/>\n  <joint name="imu_joint" type="fixed">\n'
               '    <origin xyz="0 0 %.6f" rpy="0 0 %.6f"/>\n    <parent link="base_link"/>\n'
               '    <child link="imu_link"/>\n  </joint>\n' % (z, -math.pi / 2))
        text = re.sub(r"\s*<xacro:include[^>]*imu\.urdf\.xacro[^>]*/>", "\n" + imu, text)
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    text = re.sub(r"<robot name=\"[^\"]*\"[^>]*>", '<robot name="mentorpi_%s">' % variant, text)
    text = text.replace("package://mentorpi_description/meshes/", "meshes/")
    text = re.sub(r"(meshes/[\w/]+)\.STL", lambda m: m.group(1) + ".stl", text)
    text = re.sub(r"\n\s*\n+", "\n", text)
    return text


def main(src):
    src = Path(src)
    OUT.mkdir(parents=True, exist_ok=True)
    for xacro, variant in (("mecanum.xacro", "mecanum"), ("ack.xacro", "ackermann")):
        urdf = expand(src, xacro, variant)
        (OUT / ("mentorpi_%s.urdf" % variant)).write_text(urdf, encoding="utf-8", newline="\n")
        for rel in sorted(set(re.findall(r'filename="(meshes/[^"]+)"', urdf))):
            name = Path(rel).stem
            folder = rel.split("/")[1]
            srcf = next((src / "meshes" / folder).glob(name + ".*"))
            tris = read_stl(srcf)
            kind = "wheel" if "wheel" in name else "cam" if "cam" in name else "default"
            dec = decimate(tris, CELL[kind])
            write_stl(OUT / rel, dec)
            print("%-28s %7d -> %6d tris" % (rel, len(tris), len(dec)))
    total = sum(p.stat().st_size for p in OUT.rglob("*.stl"))
    print("total mesh size: %.2f MB" % (total / 1e6))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "D:/MentorPi/Appendix/Source Code/src/simulations/mentorpi_description")
