#!/usr/bin/env python3
"""Kontrol manual robot pakai keyboard (untuk membuat peta tanpa mendorong robot).
Bisa dijalankan di LATTEPANDA (lewat buat_peta.sh) atau di LAPTOP
(./kontrol_laptop.sh, di samping RViz). Perintah lewat /teleop_motor ke
relay_kontrol.py di LattePanda, yang menghentikan motor otomatis kalau
perintah berhenti datang (mis. WiFi putus).

  PANAH ATAS    = maju
  PANAH BAWAH   = mundur
  PANAH KIRI    = putar kiri
  PANAH KANAN   = putar kanan
  SPASI         = berhenti
  S             = SIMPAN PETA (di LattePanda: deteksilokasi/maps/peta_baru)
  Ctrl+C        = keluar (motor dihentikan)

Robot hanya bergerak SELAMA tombol panah ditahan.
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int16MultiArray, String

NEUTRAL, FWD, REV = 127, 97, 157
HOLD_S = 0.6  # > jeda auto-repeat keyboard, supaya menahan tombol tidak tersendat
KEYS = {
    "UP": ((FWD, FWD), "MAJU"),
    "DOWN": ((REV, REV), "MUNDUR"),
    "LEFT": ((REV, FWD), "PUTAR KIRI"),
    "RIGHT": ((FWD, REV), "PUTAR KANAN"),
}
ARROWS = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}


def read_key() -> str:
    """Satu tombol; panah dikirim terminal sebagai ESC [ A/B/C/D.
    os.read langsung dari terminal (bukan sys.stdin.read yang menyangga sisa
    kode panah, sehingga select() tidak melihatnya)."""
    fd = sys.stdin.fileno()
    get = lambda: os.read(fd, 1).decode(errors="ignore")  # noqa: E731
    ch = get()
    if ch != "\x1b":
        return ch.lower()
    if not select.select([fd], [], [], 0.05)[0] or get() not in ("[", "O"):
        return "esc"
    if not select.select([fd], [], [], 0.05)[0]:
        return "esc"
    return ARROWS.get(get(), "esc")


class Keyboard(Node):
    def __init__(self) -> None:
        super().__init__("kontrol_keyboard")
        self.pub = self.create_publisher(Int16MultiArray, "/teleop_motor", 10)
        self.save_pub = self.create_publisher(Empty, "/simpan_peta", 10)
        self.create_subscription(String, "/simpan_peta/hasil", self.on_result, 10)

    def send(self, kiri: int, kanan: int) -> None:
        self.pub.publish(Int16MultiArray(data=[kiri, kanan]))

    def on_result(self, msg: String) -> None:
        print(f"\r\n[PETA] {msg.data}\r")


def main() -> None:
    rclpy.init()
    node = Keyboard()
    old = termios.tcgetattr(sys.stdin)
    print(__doc__.replace("\n", "\r\n"))
    cmd, label, until = (NEUTRAL, NEUTRAL), "BERHENTI", 0.0
    stop_until = 0.0  # setelah berhenti, kirim netral sebentar lalu diam
    shown = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = read_key()
                if ch == "\x03":
                    break
                if ch in KEYS:
                    cmd, label = KEYS[ch]
                    until = time.monotonic() + HOLD_S
                elif ch in (" ", "x"):
                    until = 0.0
                elif ch == "s":
                    until = 0.0
                    node.send(NEUTRAL, NEUTRAL)
                    node.save_pub.publish(Empty())
                    print("\r\n[PETA] Minta simpan peta di LattePanda...\r")
            now = time.monotonic()
            moving = now < until
            if moving:
                node.send(*cmd)  # dikirim terus 10x/detik selama tombol ditahan
                stop_until = now + 0.5
            elif now < stop_until:
                node.send(NEUTRAL, NEUTRAL)
            now_label = label if moving else "BERHENTI"
            if now_label != shown:
                kiri, kanan = cmd if moving else (NEUTRAL, NEUTRAL)
                print(f"\r[{now_label}] kiri={kiri} kanan={kanan}        ", end="", flush=True)
                shown = now_label
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.send(NEUTRAL, NEUTRAL)
            time.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print("\r\n[STOP] Motor dihentikan.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
