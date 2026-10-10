#!/usr/bin/env python3
"""Simulasi pengepelan DENGAN LiDAR tiruan (raycast ke peta) + satu "orang"
(lingkaran) yang berdiri di jalur, dan batas area dari operator.

  python3 test/simulasi_lidar.py peta.yaml batas.yaml start_x start_y orang_x orang_y [gambar.png]

Yang diukur: robot selesai & pulang, jarak terdekat titik tengah robot ke
garis batas (badan robot = lingkaran 0.25 m -> harus > 0.25 supaya tidak
menyentuh), dan jarak terdekat ke orang.
"""

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [ROOT, os.path.join(ROOT, "..", "robotfixproblemkembali3"), os.path.join(ROOT, "..", "deteksilokasi", "test")]

SIM_T = [1000.0]
time.monotonic = lambda: SIM_T[0]

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from navigasipengepelan.pengepel_node import PengepelNode  # noqa: E402
from robotfixproblemkembali3.manual_waypoint_driver_node import DriveState  # noqa: E402
from uji_matcher_offline import load_map  # noqa: E402

V, WHEEL_BASE, DT, BODY_R = 0.30, 0.45, 0.1, 0.25
NOISE = np.random.default_rng(3)
N_BEAMS, MAX_R = 360, 6.0


def wheel(b: int) -> float:
    # 97 = maju pelan (~0.30 m/s), makin kecil makin cepat; 157 = mundur; 98-156 diam
    if b <= 97:
        return V + (97 - b) * 0.01
    return -V if b >= 157 else 0.0


def wheels(k: int, n: int):
    """SATU_RODA_DIAM=1 (default, pesimis): satu roda saja tidak kuat menggerakkan
    badan -> keduanya diam (seperti robot asli di sebagian lantai).
    SATU_RODA_DIAM=0: satu roda masih bisa memutar robot (seperti saat kembali3 berhasil)."""
    vl, vr = wheel(k), wheel(n)
    if os.environ.get("SATU_RODA_DIAM", "1") == "1" and (vl == 0.0) != (vr == 0.0):
        return 0.0, 0.0
    return vl, vr


def seg_dist(px, py, poly):
    best = float("inf")
    pts = poly + poly[:1]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        vx, vy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))
        best = min(best, math.hypot(px - ax - t * vx, py - ay - t * vy))
    return best


def main() -> None:
    a = sys.argv[1:]
    yaml_path, batas_path = a[0], a[1]
    x, y = float(a[2]), float(a[3])
    ox_p, oy_p = float(a[4]), float(a[5])
    out_png = a[6] if len(a) > 6 else None
    yaw = 0.0
    boundary = [tuple(p) for p in yaml.safe_load(open(batas_path))["batas"]]
    g, res, ox, oy = load_map(yaml_path)
    # Dunia tiruan = dinding & benda besar saja (bintik kecil di peta umumnya noise
    # pemetaan, LiDAR sungguhan tidak melihatnya) -- sama dengan peta perencanaan.
    from navigasipengepelan.coverage import GridMap as _GM, clean_map
    _, occ = clean_map(_GM(g, res, ox, oy))
    # orang = lingkaran r 0.2 m, hanya terlihat LiDAR (tidak ada di peta perencanaan)
    rr, cc = np.mgrid[0:g.shape[0], 0:g.shape[1]]
    person = np.hypot(ox + (cc + 0.5) * res - ox_p, oy + (rr + 0.5) * res - oy_p) <= 0.2
    world = occ | person

    rclpy.init(args=["--ros-args", "--params-file", os.path.join(ROOT, "config", "pengepel_params.yaml"),
                     "-r", "__node:=pengepel_node", "-p", "boundary_file:=/nonexistent/batas.yaml"])
    node = PengepelNode()
    cmd = [127, 127]
    # tangkap perintah AKHIR yang benar-benar dikirim ke motor (setelah penyesuaian pengepel_node)
    node.motor_pub.publish = lambda msg: cmd.__setitem__(slice(0, 2), [int(msg.data[0]), int(msg.data[1])])
    msg = OccupancyGrid()
    msg.info.width, msg.info.height, msg.info.resolution = g.shape[1], g.shape[0], res
    msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
    msg.data = [int(v) for v in g.ravel()]
    node.on_map(msg)
    node.boundary = boundary
    node.current_pose = (x, y, yaw)
    node.mulai()
    total = len(node.waypoints)

    steps = np.arange(0.05, MAX_R, res / 2)
    beam_a = -math.pi + 2 * math.pi * np.arange(N_BEAMS) / N_BEAMS

    def scan_at(x, y, yaw):
        ang = yaw + math.pi + beam_a  # LiDAR terpasang menghadap belakang (yaw pi)
        px = x + np.cos(ang)[:, None] * steps[None, :]
        py = y + np.sin(ang)[:, None] * steps[None, :]
        r_ = ((py - oy) / res).astype(int)
        c_ = ((px - ox) / res).astype(int)
        inside = (r_ >= 0) & (r_ < g.shape[0]) & (c_ >= 0) & (c_ < g.shape[1])
        hit = np.zeros_like(inside)
        hit[inside] = world[r_[inside], c_[inside]]
        j = np.argmax(hit, axis=1)
        ranges = np.where(hit[np.arange(N_BEAMS), j], steps[j], np.inf)
        s = LaserScan()
        s.angle_min, s.angle_increment = float(beam_a[0]), float(beam_a[1] - beam_a[0])
        s.angle_max = float(beam_a[-1])
        s.range_min, s.range_max = 0.15, 12.0
        s.ranges = [float(v) for v in ranges]
        return s

    traj = []
    min_b, min_p = float("inf"), float("inf")
    t0 = SIM_T[0]
    while node.state != DriveState.FINISHED and SIM_T[0] - t0 < 3600:
        node.on_scan(scan_at(x, y, yaw))
        node.control_loop()
        vl, vr = wheels(cmd[0], cmd[1])
        v, w = (vl + vr) / 2, (vr - vl) / WHEEL_BASE
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = math.atan2(math.sin(yaw + w * DT), math.cos(yaw + w * DT))
        # posisi menurut "AMCL" = posisi asli + goyangan acak (seperti AMCL sungguhan)
        node.current_pose = (x + NOISE.normal(0, 0.03), y + NOISE.normal(0, 0.03), yaw + NOISE.normal(0, math.radians(1.5)))
        SIM_T[0] += DT
        traj.append((x, y))
        min_b = min(min_b, seg_dist(x, y, boundary))
        min_p = min(min_p, math.hypot(x - ox_p, y - oy_p))
    sx, sy = node._mission_start_xy
    print(f"status: {node.state.name}, titik {node.current_index}/{total} (rute akhir {len(node.waypoints)}), "
          f"waktu {(SIM_T[0] - t0) / 60:.1f} menit, akhir {math.hypot(x - sx, y - sy):.2f} m dari start")
    print(f"jarak terdekat titik tengah robot ke garis batas: {min_b:.2f} m "
          f"(badan {BODY_R} m -> {'TIDAK menyentuh' if min_b > BODY_R else 'MENYENTUH'})")
    print(f"jarak terdekat titik tengah robot ke orang (r 0.2 m): {min_p:.2f} m "
          f"(tepi badan ke tepi orang {min_p - BODY_R - 0.2:.2f} m)")
    if out_png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        ext = [ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res]
        img = np.where(g == 100, 0, np.where(g == 0, 255, 180))
        plt.figure(figsize=(14, 7))
        plt.imshow(img, cmap="gray", origin="lower", extent=ext)
        bx, by = zip(*(boundary + boundary[:1]))
        plt.plot(bx, by, "r--", lw=2, label="batas")
        tx, ty = zip(*traj)
        plt.plot(tx, ty, "b-", lw=1, label="jejak robot")
        plt.gca().add_patch(plt.Circle((ox_p, oy_p), 0.2, color="orange", label="orang"))
        plt.plot(sx, sy, "go", ms=9)
        plt.xlim(min(bx) - 0.5, max(bx) + 0.5)
        plt.ylim(min(by) - 0.5, max(by) + 0.5)
        plt.legend(loc="lower right")
        plt.tight_layout()
        plt.savefig(out_png, dpi=70)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
