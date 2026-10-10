#!/usr/bin/env python3
"""pengepel_node -- robot mengepel seluruh area kosong peta secara otomatis
(lurus bolak-balik), lalu kembali ke titik awal.

VERSI SEDERHANA (2026-10-09, permintaan operator): cara jalan = logika
robotfixproblemkembali3 apa adanya (turunan kelasnya, kembali3 TIDAK diubah) --
koreksi jalur, menghindar halangan kiri/kanan, berhenti kalau ada tembok di
depan, mundur kalau macet, tombol darurat SPASI. Tambahan pengepelan saja:
  * rute dibuat otomatis oleh coverage.py dari peta + batas area (garis merah);
  * di ujung tiap baris robot berhenti dan berputar di tempat ke baris berikutnya;
  * ujung baris dekat dinding dianggap tercapai saat dinding sudah di depan;
  * terhalang lama di tengah baris -> sisa baris dilewati;
  * penjaga garis batas: terlalu dekat ke garis merah & mengarah ke sana -> putar menjauh;
  * koreksi arah: satu roda berhenti, roda luar lebih bertenaga (92/87);
  * pola rute bawaan "kontur": lintasan sejajar garis batas, merata dari batas ke batas;
  * di akhir tidak menyusuri balik rute -- rute sudah termasuk jalan pulang.
Versi dengan pengaman tambahan (berhenti untuk orang, jalan memutar, rute dibuat
ulang, dll.) disimpan di pengepel_node_versi_pengaman.py.bak.

Batas area: klik "Publish Point" di RViz (sudut-sudut area). Mulai: tombol G
(/pengepelan/mulai) atau klik "2D Goal Pose".
"""

import math
import os
import time
from typing import List, Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker

from robotfixproblemkembali3.manual_waypoint_driver_node import (
    MOTOR_NEUTRAL,
    SPIN_LEFT,
    SPIN_RIGHT,
    DriveState,
    ManualWaypointDriverNode,
    Waypoint,
    normalize_angle,
    sector_min_range,
)

from navigasipengepelan.coverage import GridMap, plan_contour, plan_coverage, plan_rings


class PengepelNode(ManualWaypointDriverNode):
    def __init__(self) -> None:
        self.grid_map: Optional[GridMap] = None
        self.plan_status = "menunggu peta dan posisi robot"
        self.boundary: List[tuple] = []
        self.wp_kinds: List[str] = []
        self.previewed = False
        self.plan_info = ""
        self._anchor = None
        self._duty_tick = 0
        self._prog_xy = None
        self._prog_at = 0.0
        super().__init__()
        p = self.declare_parameter
        p("map_topic", "/map")
        p("row_spacing_m", 0.40)        # lebar pel 50 cm, tumpang tindih 10 cm
        p("wall_clearance_m", 0.30)     # titik tengah robot minimal segini dari dinding/benda besar
        p("min_obstacle_m2", 0.5)      # benda lebih kecil dari ini diabaikan saat membuat rute
        p("max_hole_m2", 0.5)           # lubang "tidak dikenal" sekecil ini di tengah area dianggap kosong
        p("densify_m", 1.0)             # titik tambahan tiap segini di baris panjang
        p("corner_deg", 45.0)           # belokan lebih tajam dari ini = berhenti lalu putar di tempat
        p("corner_wall_reach_m", 0.8)   # ujung baris tercapai kalau dinding di depan & sisa jarak <= ini
        p("row_end_extra_m", 0.20)      # ujung baris dimundurkan segini dari dinding/batas di depan
        p("obstacle_skip_s", 8.0)       # terhalang di tengah baris lebih lama dari ini -> lewati sisa baris
        p("min_row_len_m", 1.3)         # potongan baris lebih pendek dari ini dibuang
        p("boundary_clearance_m", 0.45) # titik tengah robot minimal segini dari garis batas (rute)
        p("lookahead_m", 0.7)           # pengikut garis: bidik titik sejauh ini di depan pada garis biru
        p("deadband_deg", 5.0)          # mulai koreksi kalau simpangan arah > ini ...
        p("deadband_exit_deg", 2.0)     # ... dan berhenti koreksi kalau sudah < ini (tidak bolak-balik)
        p("full_turn_deg", 20.0)        # simpangan sebesar ini -> koreksi terus-menerus (lebih kecil -> sebagian waktu)
        p("pola", "bolak_balik")        # "bolak_balik" = lurus sejajar & merata (bawaan) | "kontur" = keliling sejajar batas
        p("outer_gentle", 92)           # koreksi halus: roda dalam BERHENTI, roda luar 92 (lebih bertenaga dari 97)
        p("outer_sharp", 87)            # koreksi tajam: roda dalam berhenti, roda luar 87
        p("boundary_guard_m", 0.35)
        p("laser_yaw_rad", math.pi)     # mount LiDAR (sama dengan kembali3: sudut + pi)
        p("front_guard_half_width_m", 0.30)  # jalur selebar badan di depan yang diperiksa untuk benda baru
        p("front_guard_m", 0.45)        # benda baru (tidak ada di peta) sedekat ini di depan -> berhenti & tunggu     # tengah robot sedekat ini ke garis batas & mengarah ke sana -> putar menjauh
        p("boundary_file", os.path.expanduser("~/Robot magang/navigasipengepelan/maps/batas_area.yaml"))
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.row_spacing_m = float(gp("row_spacing_m"))
        self.wall_clearance_m = float(gp("wall_clearance_m"))
        self.min_obstacle_m2 = float(gp("min_obstacle_m2"))
        self.max_hole_m2 = float(gp("max_hole_m2"))
        self.densify_m = float(gp("densify_m"))
        self.corner_rad = math.radians(float(gp("corner_deg")))
        self.corner_wall_reach_m = float(gp("corner_wall_reach_m"))
        self.row_end_extra_m = float(gp("row_end_extra_m"))
        self.obstacle_skip_s = float(gp("obstacle_skip_s"))
        self.min_row_len_m = float(gp("min_row_len_m"))
        self.boundary_clearance_m = float(gp("boundary_clearance_m"))
        self.pola = str(gp("pola"))
        self.lookahead_m = float(gp("lookahead_m"))
        self.deadband_deg = float(gp("deadband_deg"))
        self.deadband_exit_deg = float(gp("deadband_exit_deg"))
        self._correcting = False
        self.full_turn_deg = float(gp("full_turn_deg"))
        self.outer_gentle = int(gp("outer_gentle"))
        self.outer_sharp = int(gp("outer_sharp"))
        self.boundary_guard_m = float(gp("boundary_guard_m"))
        self.laser_yaw_rad = float(gp("laser_yaw_rad"))
        self.front_guard_half_width_m = float(gp("front_guard_half_width_m"))
        self.front_guard_m = float(gp("front_guard_m"))
        self.static_occ: Optional[np.ndarray] = None
        self.boundary_file = str(gp("boundary_file"))
        self._load_boundary()
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(gp("map_topic")), self.on_map, map_qos)
        self.create_subscription(Empty, "/pengepelan/mulai", lambda _m: self.mulai(), 10)
        self.create_subscription(Empty, "/pengepelan/hapus_batas", lambda _m: self.hapus_batas(), 10)
        self.create_subscription(Empty, "/pengepelan/lihat_rute", lambda _m: self.pratinjau(), 10)
        self.create_timer(1.0, self._auto_preview)
        self.create_timer(1.0, self._publish_boundary_marker)
        self.status_pub = self.create_publisher(String, "/pengepelan/status", 10)
        self.create_timer(1.0, lambda: self.status_pub.publish(String(data=self.plan_status)))
        self.get_logger().info("pengepel_node siap -- tekan G (atau klik 2D Goal Pose) untuk mulai mengepel.")

    # ------------------------------------------------------------ input
    def on_map(self, msg: OccupancyGrid) -> None:
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.grid_map = GridMap(grid, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        k = max(1, int(round(0.15 / msg.info.resolution)))
        yy, xx = np.ogrid[-k:k + 1, -k:k + 1]
        self.static_occ = ndimage.binary_dilation(grid >= 65, structure=(xx * xx + yy * yy) <= k * k)
        self.get_logger().info(f"Peta diterima {msg.info.width}x{msg.info.height}.")

    def on_clicked_point(self, msg) -> None:
        """Klik "Publish Point" = tambah satu titik sudut batas area."""
        if self.state not in (DriveState.WAITING_POINTS, DriveState.FINISHED):
            self.get_logger().warn("Sedang mengepel -- batas area tidak bisa diubah sekarang.")
            return
        self.boundary.append((float(msg.point.x), float(msg.point.y)))
        self._save_boundary()
        self.get_logger().info(f"Titik batas area {len(self.boundary)}: ({msg.point.x:.2f}, {msg.point.y:.2f})")
        self._publish_boundary_marker()
        if len(self.boundary) >= 3:
            self.pratinjau()

    def hapus_batas(self) -> None:
        if self.state not in (DriveState.WAITING_POINTS, DriveState.FINISHED):
            self.get_logger().warn("Sedang mengepel -- batas area tidak bisa dihapus sekarang.")
            return
        self.boundary = []
        try:
            os.remove(self.boundary_file)
        except OSError:
            pass
        self.get_logger().info("Batas area dihapus -- seluruh peta dipakai.")
        self._publish_boundary_marker()
        self.pratinjau()

    def _load_boundary(self) -> None:
        try:
            with open(self.boundary_file) as f:
                pts = (yaml.safe_load(f) or {}).get("batas", [])
            self.boundary = [(float(x), float(y)) for x, y in pts]
            if self.boundary:
                self.get_logger().info(f"Batas area dimuat: {len(self.boundary)} titik dari {self.boundary_file}")
        except (OSError, yaml.YAMLError, TypeError, ValueError):
            self.boundary = []

    def _save_boundary(self) -> None:
        try:
            os.makedirs(os.path.dirname(self.boundary_file), exist_ok=True)
            with open(self.boundary_file, "w") as f:
                yaml.safe_dump({"batas": [[x, y] for x, y in self.boundary]}, f)
        except OSError as exc:
            self.get_logger().warn(f"Batas area tidak bisa disimpan: {exc}")

    def on_start_trigger(self, msg: PoseStamped) -> None:
        self.mulai()

    def _load_saved_waypoints(self) -> None:
        pass

    def _save_waypoints(self, waypoints) -> None:
        pass

    def _smooth_path(self, waypoints):
        return list(waypoints)

    # ------------------------------------------------------------ rute

    # ------------------------------------------------------------ rute
    def _make_plan(self) -> bool:
        """Buat rute dari posisi robot sekarang -> self.waypoints (+ jenisnya)."""
        if self.grid_map is None or self.current_pose is None:
            self.get_logger().warn("Belum bisa membuat rute: peta atau posisi robot belum ada (tunggu deteksi lokasi).")
            return False
        sx, sy, _ = self.current_pose
        bd = self.boundary if len(self.boundary) >= 3 else None
        extra = max(0.0, self.boundary_clearance_m - self.wall_clearance_m)
        if self.pola == "kontur" and bd is not None:
            # keliling bersarang sejajar garis batas merah (rapi, jarak rata)
            plan = plan_rings(
                self.grid_map, (sx, sy), bd, self.row_spacing_m,
                first_offset_m=self.boundary_clearance_m + 0.02, clearance_m=self.wall_clearance_m,
                min_obstacle_m2=self.min_obstacle_m2, max_hole_m2=self.max_hole_m2,
            )
        elif self.pola == "kontur":
            plan = plan_contour(
                self.grid_map, (sx, sy), self.row_spacing_m, self.wall_clearance_m,
                min_obstacle_m2=self.min_obstacle_m2, max_hole_m2=self.max_hole_m2,
                boundary=bd, boundary_extra_m=extra,
            )
        else:
            plan = plan_coverage(
                self.grid_map, (sx, sy), self.row_spacing_m, self.wall_clearance_m,
                min_row_len_m=self.min_row_len_m, min_obstacle_m2=self.min_obstacle_m2,
                max_hole_m2=self.max_hole_m2, boundary=self.boundary if len(self.boundary) >= 3 else None,
                boundary_extra_m=max(0.0, self.boundary_clearance_m - self.wall_clearance_m),
                row_end_extra_m=self.row_end_extra_m,
                hole_fill_m=0.3, bridge_gap_m=0.6,  # lintasan utuh, tidak terpotong bintik peta
            )
        for note in plan.notes:
            self.get_logger().warn(note)
        if not plan.waypoints:
            self.get_logger().error("Rute kosong -- tidak ada area yang bisa dipel dari posisi ini.")
            return False
        self.waypoints, self.wp_kinds = self._to_waypoints([(sx, sy, "row")] + [(w.x, w.y, w.kind) for w in plan.waypoints])
        self.waypoints, self.wp_kinds = self.waypoints[1:], self.wp_kinds[1:]
        self.current_index = 0
        batas = f", batas area {len(self.boundary)} titik" if len(self.boundary) >= 3 else ""
        self.plan_info = (
            f"{len(plan.rows)} baris, pel {plan.coverage_length_m:.0f} m, "
            f"pindah {plan.travel_length_m:.0f} m, arah baris {math.degrees(plan.angle_rad):.0f} deg{batas}"
        )
        return True

    def pratinjau(self) -> None:
        if self.state not in (DriveState.WAITING_POINTS, DriveState.FINISHED):
            return
        if self._make_plan():
            self.previewed = True
            self.plan_status = f"PRATINJAU {self.plan_info} -- tekan G untuk mulai"
            self.get_logger().info(f"Rute pratinjau: {self.plan_info}")

    def _auto_preview(self) -> None:
        if not self.previewed and self.grid_map is not None and self.current_pose is not None:
            self.pratinjau()

    def mulai(self) -> None:
        if self.state not in (DriveState.WAITING_POINTS, DriveState.FINISHED):
            self.get_logger().warn("Sedang mengepel -- perintah mulai diabaikan.")
            return
        self.get_logger().info("Membuat rute pengepelan dari peta...")
        if not self._make_plan():
            return
        sx, sy, _ = self.current_pose
        self.plan_status = f"MENGEPEL {self.plan_info}"
        self.get_logger().info(f"Rute siap: {self.plan_info}, {len(self.waypoints)} titik.")
        self._mission_start_xy = (sx, sy)
        self._anchor = None
        self.current_index = 0
        self._returning = True  # matikan jendela lurus-di-awal dan putar-balik bawaan kembali3
        self._start_straight_until = None
        self._enter(DriveState.TURNING)

    def _to_waypoints(self, pts: List[tuple]):
        """Tambah titik antara di baris panjang; tandai belokan tajam (ujung baris)
        sebagai hard_corner supaya robot berhenti dan berputar di sana.
        pts: (x, y, jenis). Hasil: (daftar Waypoint, daftar jenis)."""
        dense: List[tuple] = [pts[0][:2]]
        kinds: List[str] = [pts[0][2]]
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / self.densify_m)))
            for k in range(1, n + 1):
                dense.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
                kinds.append(b[2])
        out: List[Waypoint] = []
        for i, p in enumerate(dense):
            corner = True
            if 0 < i < len(dense) - 1:
                h_in = math.atan2(p[1] - dense[i - 1][1], p[0] - dense[i - 1][0])
                h_out = math.atan2(dense[i + 1][1] - p[1], dense[i + 1][0] - p[0])
                corner = abs(math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))) > self.corner_rad
            out.append(Waypoint(x=p[0], y=p[1], hard_corner=corner))
        return out, kinds

    # ------------------------------------------------------------ loop kontrol

    # ------------------------------------------------------------ loop kontrol
    def control_loop(self) -> None:
        """Sama seperti kembali3, kecuali: berhenti + putar di tempat di setiap ujung
        baris, ujung baris dekat dinding dianggap tercapai, terhalang lama -> lewati
        sisa baris, dan selesai di titik terakhir (sudah titik awal)."""
        if self.state in (DriveState.WAITING_POINTS, DriveState.FINISHED):
            return
        if self._emergency_stop:
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            return
        if self.current_pose is None:
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            return
        for _ in range(10):
            target = self.waypoints[self.current_index]
            x, y, _yaw = self.current_pose
            dx, dy = target.x - x, target.y - y
            dist = math.hypot(dx, dy)
            is_last = self.current_index == len(self.waypoints) - 1
            if is_last:
                reached = self._end_reached(x, y, target, dist)
            elif target.hard_corner:
                reached = self._corner_reached(x, y, target, dist)
            else:
                reached = self._passed_point(x, y, target)
            if dist > self.waypoint_tolerance_m and not reached:
                break
            # Garis biru TIDAK diubah -- robot yang dikoreksi kembali ke garis.
            # Pengecualian (internal, tidak terlihat): ujung baris yang dianggap tercapai
            # karena dinding sudah di depan -> sambungan pendek ke baris berikutnya
            # diukur dari posisi robot, supaya tidak dikira melenceng jauh di belokan.
            self._anchor = (self.current_index + 1, (x, y)) if (
                target.hard_corner and dist > self.waypoint_tolerance_m) else None
            self.current_index += 1
            # batas waktu kembali3 dihitung sejak masuk mode jalan -> ulang tiap titik
            self._state_entered_at = time.monotonic()
            if self.current_index >= len(self.waypoints):
                self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
                self._enter(DriveState.FINISHED)
                self.plan_status = "SELESAI -- robot sudah kembali ke titik awal"
                self.get_logger().info("Selesai mengepel -- robot sudah kembali ke titik awal.")
                return
            self.get_logger().info(
                f"Titik {self.current_index}/{len(self.waypoints)}" + (" (ujung baris)" if target.hard_corner else ""),
                throttle_duration_sec=0.5,
            )
            if target.hard_corner and self.state in (DriveState.DRIVING, DriveState.PAUSED_OBSTACLE):
                nxt = self.waypoints[self.current_index]
                self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
                self._enter(DriveState.TURNING)
                dx, dy = nxt.x - x, nxt.y - y
                break
        if (
            self.state == DriveState.PAUSED_OBSTACLE
            and time.monotonic() - self._state_entered_at > self.obstacle_skip_s
        ):
            self.skip_rest_of_row()
            return
        if self.state == DriveState.TURNING:
            self._do_turning(dx, dy)
        elif self.state == DriveState.DRIVING:
            self._do_driving(dx, dy)
        elif self.state == DriveState.PAUSED_OBSTACLE:
            self._do_paused_obstacle()
        elif self.state == DriveState.REVERSING:
            self._do_reversing()

    def _segment_start_xy(self):
        if self._anchor is not None and self._anchor[0] == self.current_index:
            return self._anchor[1]
        return super()._segment_start_xy()

    def _do_turning(self, dx: float, dy: float) -> None:
        """Putar di tempat menghadap titik berikutnya. Selesai kalau arah sudah dalam
        toleransi ATAU baru saja MELEWATI target (tanda selisih berubah): satu langkah
        kontrol bisa memutar robot lebih dari 2x toleransi, sehingga versi kembali3
        bisa bolak-balik melewati target terus sampai batas waktu habis."""
        _, _, yaw = self.current_pose
        err = normalize_angle(math.atan2(dy, dx) - yaw)
        elapsed = time.monotonic() - self._state_entered_at
        prev = self._turn_prev if getattr(self, "_turn_prev_at", None) == self._state_entered_at else None
        crossed = prev is not None and (err > 0) != (prev > 0) and abs(err) < 0.6
        if abs(err) <= self.turn_pivot_tolerance_rad or crossed or elapsed >= self.turn_pivot_timeout_s:
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            self._enter(DriveState.DRIVING)
            return
        self._turn_prev, self._turn_prev_at = err, self._state_entered_at
        self._publish_motor(*(SPIN_LEFT if err > 0 else SPIN_RIGHT))

    def _corner_reached(self, x: float, y: float, target: Waypoint, dist: float) -> bool:
        if self._passed_point(x, y, target):
            return True
        if dist <= self.corner_wall_reach_m and self.latest_scan is not None:
            front = sector_min_range(self.latest_scan, 180.0, self.front_sector_deg)
            return front < self.front_stop_from_lidar_m + self.front_clear_margin_m
        return False

    def skip_rest_of_row(self) -> None:
        """Terhalang terlalu lama di tengah baris: lompat ke titik setelah ujung
        baris berikutnya, lalu berputar menghadap ke sana."""
        if self.current_index >= len(self.waypoints) - 1:
            # Titik terakhir (titik awal) terhalang terus: jangan coba tanpa henti.
            self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
            self._enter(DriveState.FINISHED)
            self.plan_status = "SELESAI -- titik awal terhalang, robot berhenti di dekatnya"
            self.get_logger().warn("Titik awal terhalang terus -- robot berhenti di dekatnya. Pengepelan selesai.")
            return
        i = self.current_index
        while i < len(self.waypoints) - 1 and not self.waypoints[i].hard_corner:
            i += 1
        self.current_index = min(i + 1, len(self.waypoints) - 1)
        self.get_logger().warn(
            f"Terhalang lebih dari {self.obstacle_skip_s:.0f}s -- sisa baris dilewati, "
            f"lanjut ke titik {self.current_index + 1}/{len(self.waypoints)}."
        )
        self._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
        self._enter(DriveState.TURNING)

    # ------------------------------------------------------------ tampilan

    # ------------------------------------------------------------ tampilan
    def _publish_path_marker(self) -> None:
        """Baris pel = garis biru tebal, pindah/pulang = garis abu-abu tipis;
        bagian yang sudah dilewati jadi hijau. Satu marker LINE_LIST per jenis."""
        if not self.waypoints:
            return
        stamp = self.get_clock().now().to_msg()
        start = self._mission_start_xy if self.state not in (DriveState.WAITING_POINTS,) else None
        if start is None and self.current_pose is not None:
            start = self.current_pose[:2]
        prev = start or (self.waypoints[0].x, self.waypoints[0].y)
        groups = {"pel": [], "pindah": [], "selesai": []}
        for i, w in enumerate(self.waypoints):
            kind = self.wp_kinds[i] if i < len(self.wp_kinds) else "row"
            busy = self.state not in (DriveState.WAITING_POINTS, DriveState.FINISHED)
            key = "selesai" if busy and i < self.current_index else ("pel" if kind == "row" else "pindah")
            groups[key] += [Point(x=prev[0], y=prev[1], z=0.02), Point(x=w.x, y=w.y, z=0.02)]
            prev = (w.x, w.y)
        style = {"pel": (0, 0.06, (0.1, 0.45, 1.0, 0.95)),
                 "pindah": (1, 0.02, (0.55, 0.55, 0.55, 0.7)),
                 "selesai": (2, 0.06, (0.1, 0.8, 0.3, 0.8))}
        for key, (mid, width, rgba) in style.items():
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = "rute_pengepelan", mid, Marker.LINE_LIST
            m.action = Marker.ADD if groups[key] else Marker.DELETE
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.pose.orientation.w = 1.0
            m.points = groups[key]
            self.path_viz_pub.publish(m)

    def _publish_boundary_marker(self) -> None:
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = "batas_area", 0, Marker.LINE_STRIP
        pts = self.boundary + (self.boundary[:1] if len(self.boundary) >= 3 else [])
        m.action = Marker.ADD if len(pts) >= 2 else Marker.DELETE
        m.scale.x = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.95
        m.pose.orientation.w = 1.0
        m.points = [Point(x=x, y=y, z=0.05) for x, y in pts]
        self.path_viz_pub.publish(m)
        d = Marker()
        d.header = m.header
        d.ns, d.id, d.type = "batas_area", 1, Marker.SPHERE_LIST
        d.action = Marker.ADD if self.boundary else Marker.DELETE
        d.scale.x = d.scale.y = d.scale.z = 0.18
        d.color.r, d.color.g, d.color.b, d.color.a = 1.0, 0.1, 0.1, 0.95
        d.pose.orientation.w = 1.0
        d.points = [Point(x=x, y=y, z=0.05) for x, y in self.boundary]
        self.path_viz_pub.publish(d)



def main() -> None:
    rclpy.init()
    node = PengepelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_motor(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
