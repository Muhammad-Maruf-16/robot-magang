#!/usr/bin/env python3
"""Gambar rute pengepelan di atas peta (tanpa robot).
  python3 test/gambar_rute.py maps/peta_baru.yaml X Y output.png [x1,y1 x2,y2 x3,y3 ...]  (batas area opsional)
"""
import math, os, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "deteksilokasi", "test"))
from navigasipengepelan.coverage import GridMap, plan_coverage  # noqa: E402
from uji_matcher_offline import load_map  # noqa: E402
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt  # noqa: E402

yaml_path, sx, sy, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
boundary = [tuple(map(float, p.split(","))) for p in sys.argv[5:]] or None
g, res, ox, oy = load_map(yaml_path)
m = GridMap(g, res, ox, oy)
t0 = time.time()
p = plan_coverage(m, (sx, sy), boundary=boundary)
dt = time.time() - t0
print(f"{len(p.rows)} baris, {len(p.waypoints)} titik, sudut {math.degrees(p.angle_rad):.1f} deg, "
      f"pel {p.coverage_length_m:.1f} m, pindah {p.travel_length_m:.1f} m, {dt:.1f}s")
for n in p.notes: print("  ", n)
ext = [ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res]
img = np.where(g == 100, 0, np.where(g == 0, 255, 180)).astype(float)
plt.figure(figsize=(14, 7))
plt.imshow(img, cmap="gray", origin="lower", extent=ext)
plt.imshow(np.ma.masked_where(~p.area_mask, p.area_mask), cmap="Greens", alpha=0.25, origin="lower", extent=ext, vmin=0, vmax=1.5)
xs = [sx] + [w.x for w in p.waypoints]; ys = [sy] + [w.y for w in p.waypoints]
col = {"row": "tab:blue", "turn": "tab:orange", "travel": "tab:red", "home": "tab:purple"}
for i, w in enumerate(p.waypoints):
    plt.plot([xs[i], xs[i + 1]], [ys[i], ys[i + 1]], color=col[w.kind], lw=1.4)
plt.plot(sx, sy, "go", ms=10, label="start / pulang")
if boundary:
    bx, by = zip(*(boundary + [boundary[0]])); plt.plot(bx, by, "r--", lw=2, label="batas area")
rr, cc = np.nonzero(p.area_mask)
pad = 1.0
plt.xlim(ox + cc.min() * res - pad, ox + cc.max() * res + pad); plt.ylim(oy + rr.min() * res - pad, oy + rr.max() * res + pad)
plt.title("biru = baris pel, oranye = pindah baris, merah = lewat jalur aman, ungu = pulang")
plt.legend(loc="lower right"); plt.tight_layout(); plt.savefig(out, dpi=70)
