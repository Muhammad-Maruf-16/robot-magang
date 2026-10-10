#!/usr/bin/env python3
"""deteksi_lokasi_node -- robot menentukan sendiri posisinya di peta.

  * Saat start: scan LiDAR dicocokkan ke seluruh peta (matcher.py), posisi
    terbaik dikirim ke AMCL lewat /initialpose -- sama seperti klik
    "2D Pose Estimate" di RViz, tapi otomatis.
  * Saat robot diangkat lalu ditaruh di tempat lain: scan tidak lagi cocok
    dengan posisi AMCL sekarang, roda (odom) diam, dan scan sudah stabil
    (robot sudah diletakkan) -> cari lokasi lagi.
  * Kalau hasilnya ragu (skor rendah, atau dua tempat berbeda sama-sama
    cocok), posisi TIDAK dikirim; dicoba lagi setelah retry_seconds.
  * Paksa cari ulang kapan saja: ros2 topic pub --once /deteksi_lokasi/cari std_msgs/msg/Empty

Node ini tidak pernah menggerakkan motor.
"""

import math
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from deteksilokasi.matcher import Candidate, MapMatcher, beams_from_scan


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DeteksiLokasiNode(Node):
    def __init__(self) -> None:
        super().__init__("deteksi_lokasi_node")
        p = self.declare_parameter
        p("map_topic", "/map")
        p("scan_topic", "/scan_filtered")
        p("odom_topic", "/odom")
        p("amcl_pose_topic", "/amcl_pose")
        p("initial_pose_topic", "/initialpose")
        p("base_frame", "base_footprint")
        # Dipakai kalau TF base_footprint -> laser belum ada (mount robot ini: yaw 180 derajat).
        p("laser_yaw_fallback", math.pi)
        p("auto_on_start", True)
        p("start_delay_s", 3.0)
        p("accept_score", 0.65)       # skor minimum supaya hasil dipercaya
        p("accept_margin", 0.05)      # selisih minimum dengan kandidat lain yang beda tempat
        p("retry_seconds", 5.0)       # hasil ragu -> coba lagi setelah ini
        p("check_period_s", 1.0)
        p("lost_score", 0.40)         # scan vs posisi AMCL di bawah ini = "posisi sudah tidak cocok"
        p("lost_seconds", 3.0)
        p("still_odom_m", 0.05)       # roda dianggap diam kalau odom bergerak kurang dari ini
        p("stable_scan_m", 0.08)      # scan dianggap stabil (robot sudah ditaruh)
        p("cooldown_s", 8.0)
        p("search_stride_m", 0.15)
        p("search_angle_step_deg", 5.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base_frame = str(gp("base_frame"))
        self.laser_yaw_fallback = float(gp("laser_yaw_fallback"))
        self.auto_on_start = bool(gp("auto_on_start"))
        self.start_delay_s = float(gp("start_delay_s"))
        self.accept_score = float(gp("accept_score"))
        self.accept_margin = float(gp("accept_margin"))
        self.retry_seconds = float(gp("retry_seconds"))
        self.lost_score = float(gp("lost_score"))
        self.lost_seconds = float(gp("lost_seconds"))
        self.still_odom_m = float(gp("still_odom_m"))
        self.stable_scan_m = float(gp("stable_scan_m"))
        self.cooldown_s = float(gp("cooldown_s"))
        self.search_stride_m = float(gp("search_stride_m"))
        self.search_angle_step_deg = float(gp("search_angle_step_deg"))

        self.matcher: Optional[MapMatcher] = None
        self.map_frame = "map"
        self.scan: Optional[LaserScan] = None
        self.prev_scan_ranges: Optional[np.ndarray] = None
        self.scan_change_m = float("inf")
        self.amcl_pose: Optional[Tuple[float, float, float]] = None
        self.odom_hist: Deque[Tuple[float, float, float]] = deque(maxlen=200)
        self.started_at = time.monotonic()
        self.did_start_search = False
        self.retry_at: Optional[float] = None
        self.lost_since: Optional[float] = None
        self.last_search_at = -1e9
        self.force = False
        # Posisi yang baru dikirim, menunggu dipakai AMCL (kalau AMCL belum
        # aktif saat dikirim, pesannya hilang -> kirim ulang).
        self.pending_pose: Optional[Candidate] = None
        self.pending_tries = 0
        self.pending_check_at = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, str(gp("map_topic")), self.on_map, map_qos)
        self.create_subscription(LaserScan, str(gp("scan_topic")), self.on_scan, 10)
        self.create_subscription(Odometry, str(gp("odom_topic")), self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, str(gp("amcl_pose_topic")), self.on_amcl, 10)
        self.create_subscription(Empty, "/deteksi_lokasi/cari", self.on_force, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, str(gp("initial_pose_topic")), 10)
        self.status_pub = self.create_publisher(String, "/deteksi_lokasi/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/deteksi_lokasi/marker", 10)
        self.create_timer(float(gp("check_period_s")), self.tick)
        self.get_logger().info("deteksi_lokasi_node siap -- menunggu peta dan scan.")

    # ---------------------------------------------------------------- callbacks
    def on_map(self, msg: OccupancyGrid) -> None:
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.matcher = MapMatcher(grid, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_frame = msg.header.frame_id or "map"
        self.get_logger().info(
            f"Peta diterima {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m, "
            f"{int(self.matcher.candidate_mask.sum())} posisi kandidat."
        )

    def on_scan(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=np.float64)
        r = np.where(np.isfinite(r), np.clip(r, 0.0, msg.range_max), msg.range_max)
        if self.prev_scan_ranges is not None and len(self.prev_scan_ranges) == len(r):
            self.scan_change_m = float(np.median(np.abs(r - self.prev_scan_ranges)))
        self.prev_scan_ranges = r
        self.scan = msg

    def on_odom(self, msg: Odometry) -> None:
        self.odom_hist.append((time.monotonic(), msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self.amcl_pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_force(self, _msg: Empty) -> None:
        self.get_logger().info("Diminta cari lokasi sekarang.")
        self.force = True

    # ---------------------------------------------------------------- helpers
    def beams(self) -> Optional[np.ndarray]:
        if self.scan is None:
            return None
        lx = ly = 0.0
        lyaw = self.laser_yaw_fallback
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.scan.header.frame_id, rclpy.time.Time())
            lx, ly = t.transform.translation.x, t.transform.translation.y
            lyaw = yaw_from_quat(t.transform.rotation)
        except Exception:  # TF belum siap -> pakai fallback
            pass
        s = self.scan
        return beams_from_scan(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max, lx, ly, lyaw)

    def odom_still(self, seconds: float) -> bool:
        now = time.monotonic()
        pts = [(x, y) for t, x, y in self.odom_hist if now - t <= seconds]
        if len(pts) < 2:
            return True  # tidak ada data odom = roda tidak dilaporkan bergerak
        xs, ys = zip(*pts)
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < self.still_odom_m

    def status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    # ---------------------------------------------------------------- main loop
    def tick(self) -> None:
        if self.matcher is None or self.scan is None:
            return
        now = time.monotonic()
        if self.force:
            self.force = False
            self.search("diminta")
            return
        if self.auto_on_start and not self.did_start_search and now - self.started_at >= self.start_delay_s:
            self.did_start_search = True
            self.search("start")
            return
        if self.pending_pose is not None and now >= self.pending_check_at:
            self.confirm_pending(now)
            return
        if self.retry_at is not None and now >= self.retry_at:
            self.retry_at = None
            self.search("coba lagi")
            return
        self.watch_kidnap(now)

    def watch_kidnap(self, now: float) -> None:
        if self.amcl_pose is None or now - self.last_search_at < self.cooldown_s:
            self.lost_since = None
            return
        beams = self.beams()
        if beams is None:
            return
        score = self.matcher.score_pose(*self.amcl_pose, beams)
        if score >= self.lost_score:
            self.lost_since = None
            return
        if self.lost_since is None:
            self.lost_since = now
            self.get_logger().warn(f"Scan tidak cocok dengan posisi sekarang (skor {score:.2f}) -- robot dipindah?")
        if (
            now - self.lost_since >= self.lost_seconds
            and self.odom_still(self.lost_seconds)
            and self.scan_change_m < self.stable_scan_m
        ):
            self.lost_since = None
            self.search("robot dipindah")

    def search(self, reason: str) -> None:
        beams = self.beams()
        if beams is None:
            self.get_logger().warn("Scan belum cukup untuk mencari lokasi.")
            self.retry_at = time.monotonic() + self.retry_seconds
            return
        self.last_search_at = time.monotonic()
        self.status("MENCARI")
        self.get_logger().info(f"Mencari lokasi ({reason})...")
        t0 = time.monotonic()
        cands = self.matcher.global_search(beams, self.search_stride_m, self.search_angle_step_deg)
        dt = time.monotonic() - t0
        if not cands:
            self.get_logger().warn("Tidak ada kandidat posisi.")
            self.retry_at = time.monotonic() + self.retry_seconds
            return
        best = cands[0]
        second = cands[1].score if len(cands) > 1 else 0.0
        self.publish_markers(cands[:5])
        info = (f"x={best.x:.2f} y={best.y:.2f} arah={math.degrees(best.yaw):.0f}deg "
                f"skor={best.score:.2f} (kandidat lain {second:.2f}), {dt:.1f}s")
        if best.score < self.accept_score or best.score - second < self.accept_margin:
            self.get_logger().warn(
                f"Lokasi BELUM YAKIN: {info}. Tidak dikirim ke AMCL; coba lagi {self.retry_seconds:.0f}s. "
                "Geser robot sedikit, atau klik 2D Pose Estimate manual."
            )
            self.status(f"RAGU {info}")
            self.retry_at = time.monotonic() + self.retry_seconds
            return
        self.amcl_pose = None  # tunggu AMCL melaporkan posisi baru (lihat confirm_pending)
        self.publish_pose(best)
        self.pending_pose, self.pending_tries = best, 1
        self.pending_check_at = time.monotonic() + 2.0
        self.get_logger().info(f"Saya di sini: {info}")
        self.status(f"KETEMU {info}")

    def confirm_pending(self, now: float) -> None:
        c = self.pending_pose
        a = self.amcl_pose
        if a is not None and math.hypot(a[0] - c.x, a[1] - c.y) < 0.3:
            self.get_logger().info("AMCL sudah memakai posisi baru.")
            self.pending_pose = None
            return
        if self.pending_tries >= 5:
            self.get_logger().warn("AMCL tidak merespons /initialpose -- cek apakah AMCL aktif.")
            self.pending_pose = None
            return
        self.pending_tries += 1
        self.publish_pose(c)
        self.pending_check_at = now + 2.0

    def publish_pose(self, c: Candidate) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = c.x
        msg.pose.pose.position.y = c.y
        msg.pose.pose.orientation.z = math.sin(c.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(c.yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(3.0) ** 2
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

    def publish_markers(self, cands) -> None:
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for i, c in enumerate(cands):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = "kandidat", i, Marker.ARROW
            m.pose.position.x, m.pose.position.y = c.x, c.y
            m.pose.orientation.z, m.pose.orientation.w = math.sin(c.yaw / 2), math.cos(c.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = 0.5, 0.08, 0.08
            best = i == 0
            m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.9, 0.2, 1.0) if best else (1.0, 0.6, 0.0, 0.6)
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type = "skor", i, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = c.x, c.y, 0.4
            t.scale.z = 0.25
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{'SAYA DI SINI ' if best else ''}{c.score:.2f}"
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = DeteksiLokasiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
