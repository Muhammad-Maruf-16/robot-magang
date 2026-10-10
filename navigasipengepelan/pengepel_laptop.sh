#!/bin/bash
# SATU PERINTAH di laptop untuk MENGEPEL otomatis.
#   1. menyalakan pengepelan di LattePanda (lewat SSH): LiDAR, AMCL, deteksi lokasi, pengepel
#   2. membuka RViz (rute zig-zag + posisi robot)
#   3. terminal INI jadi kontrol keyboard:
#        G = MULAI mengepel, SPASI = berhenti darurat / lanjut, Ctrl+C = selesai
# Klik terminal ini dulu supaya tombol keyboard masuk ke sini (bukan ke RViz).
DIR="$(cd "$(dirname "$0")" && pwd)"
LP="lattepanda@100.64.113.128"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 -i "$HOME/.ssh/lattepanda" "$LP")
RVIZ_PID=""

selesai() {
    [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null
    echo "[STOP] Mematikan pengepelan di LattePanda..."
    "${SSH[@]}" 'pkill -TERM -f "^bash scripts/start.sh"' 2>/dev/null || true
}
trap selesai EXIT

echo "[1/3] Menyalakan pengepelan di LattePanda..."
"${SSH[@]}" 'cd ~/"Robot magang/navigasipengepelan" && (nohup setsid bash scripts/start.sh > /tmp/pengepelan.log 2>&1 < /dev/null &)' || {
    echo "[ERROR] Tidak bisa SSH ke LattePanda ($LP). Cek LattePanda menyala dan WiFi tersambung."; exit 1; }
for i in $(seq 1 40); do
    if "${SSH[@]}" 'grep -q "pengepel_node siap" /tmp/pengepelan.log' 2>/dev/null; then
        break
    fi
    if "${SSH[@]}" 'grep -qE "^\\[ERROR\\] [^[]" /tmp/pengepelan.log' 2>/dev/null; then
        "${SSH[@]}" 'grep -E "^\\[ERROR\\] [^[]" /tmp/pengepelan.log | head -5'
        exit 1
    fi
    sleep 1
done
echo "[2/3] Membuka RViz..."
"$DIR/buka_rviz_laptop.sh" > /dev/null 2>&1 &
RVIZ_PID=$!
sleep 3
echo "[3/3] Tunggu tulisan [LOKASI] KETEMU, lalu tekan G (klik terminal ini dulu):"
"$DIR/kontrol_laptop.sh"
