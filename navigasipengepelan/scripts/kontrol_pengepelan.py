#!/usr/bin/env python3
"""Kontrol pengepelan dari laptop (dipanggil pengepel_laptop.sh).

  BATAS AREA: di RViz pilih "Publish Point", klik sudut-sudut area berurutan
             (garis merah). Rute pratinjau langsung diperbarui.
  G      = MULAI mengepel (rute dibuat otomatis dari posisi robot sekarang)
  SPASI  = BERHENTI DARURAT / lanjut lagi
  P      = lihat ulang rute (pratinjau)
  H      = hapus batas area
  Ctrl+C = selesai (robot dihentikan, program di LattePanda dimatikan)
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class Kontrol(Node):
    def __init__(self) -> None:
        super().__init__("kontrol_pengepelan")
        self.mulai_pub = self.create_publisher(Empty, "/pengepelan/mulai", 10)
        self.lihat_pub = self.create_publisher(Empty, "/pengepelan/lihat_rute", 10)
        self.hapus_pub = self.create_publisher(Empty, "/pengepelan/hapus_batas", 10)
        self.stop_pub = self.create_publisher(Bool, "/emergency_stop", 10)
        self.create_subscription(String, "/pengepelan/status", self.on_status, 10)
        self.create_subscription(String, "/deteksi_lokasi/status", self.on_lokasi, 10)
        self.status = ""
        self.lokasi = ""

    def on_status(self, msg: String) -> None:
        if msg.data != self.status:
            self.status = msg.data
            print(f"\r\n[PENGEPELAN] {msg.data}\r")

    def on_lokasi(self, msg: String) -> None:
        word = msg.data.split(" ")[0]
        if word != self.lokasi:
            self.lokasi = word
            print(f"\r\n[LOKASI] {msg.data}\r")


def main() -> None:
    rclpy.init()
    node = Kontrol()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(__doc__.replace("\n", "\r\n"))
    stopped = False
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not select.select([fd], [], [], 0.1)[0]:
                continue
            ch = os.read(fd, 1).decode(errors="ignore").lower()
            if ch == "\x03":
                break
            if ch == "g":
                if stopped:
                    stopped = False
                    node.stop_pub.publish(Bool(data=False))
                node.mulai_pub.publish(Empty())
                print("\r\n[MULAI] Minta mulai mengepel...\r")
            elif ch == "p":
                node.lihat_pub.publish(Empty())
                print("\r\n[RUTE] Minta pratinjau rute...\r")
            elif ch == "h":
                node.hapus_pub.publish(Empty())
                print("\r\n[BATAS] Minta hapus batas area...\r")
            elif ch == " ":
                stopped = not stopped
                node.stop_pub.publish(Bool(data=stopped))
                print("\r\n[STOP] BERHENTI DARURAT -- SPASI lagi untuk lanjut.\r" if stopped
                      else "\r\n[LANJUT] Robot boleh jalan lagi.\r")
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.stop_pub.publish(Bool(data=True))
            time.sleep(0.05)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\r\n[SELESAI] Robot dihentikan.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
