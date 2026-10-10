#!/usr/bin/env python3
"""Sisi ROBOT (jalan di LattePanda) untuk kontrol manual saat membuat peta.

  /teleop_motor (Int16MultiArray [kiri, kanan]) -> diteruskan ke /motor_rpm
  Pengaman: kalau tidak ada perintah selama WATCHDOG_S (WiFi putus, keyboard
  ditutup), motor langsung dikirim netral -- robot tidak jalan terus sendiri.
  /simpan_peta (std_msgs/Empty) -> peta disimpan DI LATTEPANDA ke PETA_OUTPUT.
  Tombol di RViz (Interactive Markers, namespace /kontrol_robot, tool "Interact"):
  panah di sekitar robot -- satu klik = gerak BURST_S detik lalu berhenti sendiri;
  kotak merah = STOP; kotak biru = SIMPAN PETA.

Hanya byte yang aman yang diteruskan: 97 (maju), 127 (netral), 157 (mundur).
"""

import math
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer
from std_msgs.msg import Empty, Int16MultiArray, String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

NEUTRAL = 127
ALLOWED = {97, 127, 157}
WATCHDOG_S = 0.5
BURST_S = 1.0  # satu klik tombol RViz = gerak selama ini
FWD, REV = 97, 157
# nama: (perintah motor, posisi tombol di frame robot x/y, arah panah yaw, warna rgb, label)
BUTTONS = {
    "maju": ((FWD, FWD), (0.7, 0.0), 0.0, (0.1, 0.8, 0.2), "MAJU"),
    "mundur": ((REV, REV), (-0.7, 0.0), 3.14159, (0.1, 0.8, 0.2), "MUNDUR"),
    "kiri": ((REV, FWD), (0.0, 0.7), 1.5708, (1.0, 0.7, 0.0), "PUTAR KIRI"),
    "kanan": ((FWD, REV), (0.0, -0.7), -1.5708, (1.0, 0.7, 0.0), "PUTAR KANAN"),
}


class Relay(Node):
    def __init__(self) -> None:
        super().__init__("relay_kontrol")
        self.motor = self.create_publisher(Int16MultiArray, "/motor_rpm", 10)
        self.info = self.create_publisher(String, "/simpan_peta/hasil", 10)
        self.create_subscription(Int16MultiArray, "/teleop_motor", self.on_cmd, 10)
        self.create_subscription(
            Empty, "/simpan_peta",
            lambda _m: threading.Thread(target=self.on_save, args=(None,), daemon=True).start(), 10)
        self.last_cmd_at = 0.0
        self.moving = False
        self.burst_cmd = (NEUTRAL, NEUTRAL)
        self.burst_until = 0.0
        self.bursting = False
        self.saving = False
        self.create_timer(0.1, self.watchdog)
        self.server = InteractiveMarkerServer(self, "kontrol_robot")
        self.make_buttons()
        self.get_logger().info("relay_kontrol siap: /teleop_motor -> /motor_rpm (berhenti otomatis 0.5 s tanpa perintah).")

    def send(self, kiri: int, kanan: int) -> None:
        self.motor.publish(Int16MultiArray(data=[kiri, kanan]))

    def on_cmd(self, msg: Int16MultiArray) -> None:
        if len(msg.data) < 2 or msg.data[0] not in ALLOWED or msg.data[1] not in ALLOWED:
            return
        kiri, kanan = int(msg.data[0]), int(msg.data[1])
        self.send(kiri, kanan)
        self.last_cmd_at = time.monotonic()
        self.moving = (kiri, kanan) != (NEUTRAL, NEUTRAL)

    # ------------------------------------------------------------ tombol RViz
    def make_buttons(self) -> None:
        for name, (_cmd, (x, y), yaw, rgb, label) in BUTTONS.items():
            self.add_button(name, x, y, yaw, Marker.ARROW, (0.45, 0.15, 0.15), rgb, label)
        self.add_button("stop", 0.0, 0.0, 0.0, Marker.CUBE, (0.3, 0.3, 0.1), (0.9, 0.1, 0.1), "STOP")
        self.add_button("simpan", -1.3, 0.0, 0.0, Marker.CUBE, (0.35, 0.35, 0.1), (0.2, 0.4, 1.0), "SIMPAN PETA")
        self.server.applyChanges()

    def add_button(self, name, x, y, yaw, shape, scale, rgb, label) -> None:
        im = InteractiveMarker()
        im.header.frame_id = "base_footprint"  # tombol ikut bergerak bersama robot
        im.name, im.description, im.scale = name, label, 0.4
        im.pose.position.x, im.pose.position.y, im.pose.position.z = x, y, 0.1
        im.pose.orientation.z, im.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m = Marker()
        m.type = shape
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.9
        c = InteractiveMarkerControl()
        c.interaction_mode = InteractiveMarkerControl.BUTTON
        c.always_visible = True
        c.markers.append(m)
        im.controls.append(c)
        self.server.insert(im, feedback_callback=self.on_button)

    def on_button(self, fb: InteractiveMarkerFeedback) -> None:
        if fb.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        if fb.marker_name == "stop":
            self.burst_until = 0.0
            self.send(NEUTRAL, NEUTRAL)
            self.moving = False
        elif fb.marker_name == "simpan":
            if not self.saving:
                threading.Thread(target=self.on_save, args=(None,), daemon=True).start()
        elif fb.marker_name in BUTTONS:
            self.burst_cmd = BUTTONS[fb.marker_name][0]
            self.burst_until = time.monotonic() + BURST_S
            self.get_logger().info(f"Tombol RViz: {BUTTONS[fb.marker_name][4]}")

    # ------------------------------------------------------------ pengaman
    def watchdog(self) -> None:
        now = time.monotonic()
        if now < self.burst_until:
            self.send(*self.burst_cmd)
            self.last_cmd_at = now
            self.moving = self.bursting = True
            return
        if self.bursting:  # klik selesai -> langsung berhenti
            self.bursting = self.moving = False
            self.send(NEUTRAL, NEUTRAL)
            return
        if self.moving and now - self.last_cmd_at > WATCHDOG_S:
            self.send(NEUTRAL, NEUTRAL)
            self.moving = False
            self.get_logger().warn("Perintah kontrol berhenti datang -- motor dihentikan.")

    def on_save(self, _msg) -> None:
        if self.saving:
            return
        self.saving = True
        try:
            self._save()
        finally:
            self.saving = False

    def _save(self) -> None:
        self.burst_until = 0.0
        self.send(NEUTRAL, NEUTRAL)
        self.moving = False
        out = os.path.abspath(os.environ.get(
            "PETA_OUTPUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps", "peta_baru")))
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self.get_logger().info(f"Menyimpan peta ke {out}.yaml ...")
        r = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", out,
             "--ros-args", "-p", "map_subscribe_transient_local:=true"],
            capture_output=True, text=True, timeout=60,
        )
        ok = r.returncode == 0 and os.path.exists(out + ".yaml")
        text = f"TERSIMPAN {out}.yaml" if ok else f"GAGAL {(r.stdout + r.stderr)[-300:]}"
        (self.get_logger().info if ok else self.get_logger().error)(f"Peta {text}")
        self.info.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(NEUTRAL, NEUTRAL)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
