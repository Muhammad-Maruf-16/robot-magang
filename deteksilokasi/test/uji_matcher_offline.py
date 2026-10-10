#!/usr/bin/env python3
"""Uji matcher tanpa robot: taruh robot di pose acak di peta asli,
buat scan tiruan (raycast + noise), lalu cek apakah posisinya ketemu.

  python3 test/uji_matcher_offline.py [path/ke/room.yaml]
"""

import math
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from deteksilokasi.matcher import MapMatcher, beams_from_scan  # noqa: E402


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    pgm = os.path.join(os.path.dirname(yaml_path), meta["image"])
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    occ = (255 - img.astype(np.float64)) / 255.0
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[occ > meta["occupied_thresh"]] = 100
    grid[occ < meta["free_thresh"]] = 0
    grid = np.flipud(grid)  # baris 0 = y terkecil, seperti OccupancyGrid
    return grid, meta["resolution"], meta["origin"][0], meta["origin"][1]


def fake_scan(grid, res, ox, oy, x, y, yaw, rng, n=720, max_r=12.0):
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, np.inf)
    steps = np.arange(0.05, max_r, res / 2)
    for i, a in enumerate(angles):
        px = x + steps * math.cos(yaw + a)
        py = y + steps * math.sin(yaw + a)
        cx = ((px - ox) / res).astype(int)
        cy = ((py - oy) / res).astype(int)
        inside = (cx >= 0) & (cx < grid.shape[1]) & (cy >= 0) & (cy < grid.shape[0])
        hit = np.zeros_like(inside)
        hit[inside] = grid[cy[inside], cx[inside]] == 100
        j = np.argmax(hit)
        if hit[j]:
            ranges[i] = steps[j] + rng.normal(0, 0.03)
    ranges[rng.random(n) < 0.1] = np.inf  # 10% sinar hilang
    return ranges, angles[0], angles[1] - angles[0]


def main():
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/Documents/Robot magang/robotpel/maps/room.yaml")
    grid, res, ox, oy = load_map(yaml_path)
    m = MapMatcher(grid, res, ox, oy)
    rng = np.random.default_rng(7)
    rows, cols = np.nonzero(m.candidate_mask)
    print(f"peta {grid.shape[1]}x{grid.shape[0]}, posisi kandidat {len(rows)}")
    ok = 0
    trials = 12
    for t in range(trials):
        i = rng.integers(len(rows))
        x, y = ox + (cols[i] + 0.5) * res, oy + (rows[i] + 0.5) * res
        x += rng.uniform(-0.07, 0.07)  # di antara titik grid pencarian
        y += rng.uniform(-0.07, 0.07)
        yaw = rng.uniform(-math.pi, math.pi)
        r, amin, ainc = fake_scan(grid, res, ox, oy, x, y, yaw, rng)
        clutter = rng.random(len(r)) < 0.15  # benda yang tidak ada di peta (orang, kursi)
        r[clutter] = rng.uniform(0.3, 2.0, clutter.sum())
        beams = beams_from_scan(r, amin, ainc, 0.15, 12.0)
        t0 = time.time()
        cands = m.global_search(beams)
        dt = time.time() - t0
        b = cands[0]
        err = math.hypot(b.x - x, b.y - y)
        yerr = abs(math.degrees(math.atan2(math.sin(b.yaw - yaw), math.cos(b.yaw - yaw))))
        second = cands[1].score if len(cands) > 1 else 0.0
        good = err < 0.2 and yerr < 5
        ok += good
        print(f"[{'OK ' if good else 'MISS'}] asli=({x:6.2f},{y:6.2f},{math.degrees(yaw):6.1f}) "
              f"hasil=({b.x:6.2f},{b.y:6.2f},{math.degrees(b.yaw):6.1f}) err={err:.2f}m/{yerr:.1f}deg "
              f"skor={b.score:.2f} kedua={second:.2f} waktu={dt:.1f}s")
    print(f"{ok}/{trials} benar")


if __name__ == "__main__":
    main()
