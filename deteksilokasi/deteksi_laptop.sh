#!/bin/bash
# SATU PERINTAH di laptop untuk DETEKSI LOKASI:
#   menyalakan deteksi lokasi di LattePanda (lewat SSH) lalu membuka RViz.
# Panah hijau "SAYA DI SINI" = lokasi robot. Pindahkan robot, diamkan ~3 detik -> dicari ulang.
# Tutup RViz = program di LattePanda ikut dimatikan.
DIR="$(cd "$(dirname "$0")" && pwd)"
LP="lattepanda@100.64.113.128"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 -i "$HOME/.ssh/lattepanda" "$LP")

stop_remote() {
    echo "[STOP] Mematikan deteksi lokasi di LattePanda..."
    "${SSH[@]}" 'pkill -TERM -f "^bash scripts/start.sh"' 2>/dev/null || true
}
trap stop_remote EXIT

echo "[1/2] Menyalakan deteksi lokasi di LattePanda..."
"${SSH[@]}" 'cd ~/"Robot magang/deteksilokasi" && (nohup setsid bash scripts/start.sh > /tmp/deteksilokasi.log 2>&1 < /dev/null &)' || {
    echo "[ERROR] Tidak bisa SSH ke LattePanda ($LP). Cek LattePanda menyala dan WiFi tersambung."; exit 1; }
for i in $(seq 1 40); do
    if "${SSH[@]}" 'grep -q "deteksi_lokasi_node siap" /tmp/deteksilokasi.log' 2>/dev/null; then
        echo "      Siap."
        break
    fi
    if "${SSH[@]}" 'grep -qE "^\\[ERROR\\] [^[]" /tmp/deteksilokasi.log' 2>/dev/null; then
        "${SSH[@]}" 'grep -E "^\\[ERROR\\] [^[]" /tmp/deteksilokasi.log | head -5'
        exit 1
    fi
    sleep 1
done
echo "[2/2] Membuka RViz. Tunggu panah hijau SAYA DI SINI."
"$DIR/buka_rviz_laptop.sh"
