#!/usr/bin/env python3
"""Simulasi pengepel_node TANPA robot: robot tiruan (diferensial) digerakkan
oleh byte motor dari node, posisinya diumpankan balik sebagai pose AMCL.
Waktu dipercepat (jam tiruan), tanpa LiDAR (tanpa halangan dinamis).

  python3 test/simulasi_pengepelan.py [peta.yaml] [x] [y] [yaw_deg] [gambar.png]

Model roda (sesuai robot asli): byte 97 = maju ~0.30 m/s, 157 = mundur
~0.30 m/s, 98-156 = roda DIAM (zona mati). Jarak roda 0.45 m.
"""

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "..", "robotfixproblemkembali3"))
sys.path.insert(0, os.path.join(ROOT, "..", "deteksilokasi", "test"))

SIM_T = [1000.0]
time.monotonic = lambda: SIM_T[0]  # jam tiruan untuk semua timer di node

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402

from navigasipengepelan.pengepel_node import PengepelNode  # noqa: E402
from robotfixproblemkembali3.manual_waypoint_driver_node import DriveState  # noqa: E402
from uji_matcher_offline import load_map  # noqa: E402

V, WHEEL_BASE, DT = 0.30, 0.45, 0.1


def wheel(b: int) -> float:
    return V if b <= 97 else (-V if b >= 157 else 0.0)


def main() -> None:
    a = sys.argv[1:]
    yaml_path = a[0] if a else os.path.join(ROOT, "maps", "peta_baru.yaml")
    x, y = (float(a[1]), float(a[2])) if len(a) > 2 else (-2.0, -0.8)
    yaw = math.radians(float(a[3])) if len(a) > 3 else 0.0
    out_png = a[4] if len(a) > 4 else None
    rclpy.init(args=["--ros-args", "--params-file", os.path.join(ROOT, "config", "pengepel_params.yaml"),
                     "-r", "__node:=pengepel_node"])
    node = PengepelNode()
    cmd = [127, 127]
    node._publish_motor = lambda k, n: cmd.__setitem__(slice(0, 2), [int(k), int(n)])
    g, res, ox, oy = load_map(yaml_path)
    msg = OccupancyGrid()
    msg.info.width, msg.info.height, msg.info.resolution = g.shape[1], g.shape[0], res
    msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
    msg.data = [int(v) for v in g.ravel()]
    node.on_map(msg)
    node.current_pose = (x, y, yaw)
    node.mulai()
    total = len(node.waypoints)
    traj = [(x, y)]
    turns = 0
    last_state = node.state
    t0 = SIM_T[0]
    while node.state != DriveState.FINISHED and SIM_T[0] - t0 < 4 * 3600:
        node.control_loop()
        if node.state == DriveState.TURNING and last_state != DriveState.TURNING:
            turns += 1
        last_state = node.state
        vl, vr = wheel(cmd[0]), wheel(cmd[1])
        v, w = (vl + vr) / 2, (vr - vl) / WHEEL_BASE
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = math.atan2(math.sin(yaw + w * DT), math.cos(yaw + w * DT))
        node.current_pose = (x, y, yaw)
        SIM_T[0] += DT
        if int((SIM_T[0] - t0) / DT) % 5 == 0:
            traj.append((x, y))
    dur = SIM_T[0] - t0
    sx, sy = node._mission_start_xy
    print(f"status: {node.state.name}, titik {node.current_index}/{total}, waktu tiruan {dur / 60:.1f} menit, "
          f"putar di tempat {turns}x, jarak akhir ke titik awal {math.hypot(x - sx, y - sy):.2f} m")
    if out_png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        ext = [ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res]
        img = np.where(g == 100, 0, np.where(g == 0, 255, 180))
        plt.figure(figsize=(14, 7))
        plt.imshow(img, cmap="gray", origin="lower", extent=ext)
        wx = [w.x for w in node.waypoints]
        wy = [w.y for w in node.waypoints]
        plt.plot(wx, wy, color="tab:blue", lw=0.6, alpha=0.6, label="rute")
        tx, ty = zip(*traj)
        plt.plot(tx, ty, color="tab:red", lw=1.0, label="jejak robot tiruan")
        plt.plot(sx, sy, "go", ms=9, label="start")
        plt.xlim(min(wx) - 1, max(wx) + 1)
        plt.ylim(min(wy) - 1, max(wy) + 1)
        plt.legend(loc="lower right")
        plt.tight_layout()
        plt.savefig(out_png, dpi=70)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
