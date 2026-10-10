#!/bin/bash
# SATU PERINTAH di laptop untuk membuat peta baru.
#   1. menyalakan mapping di LattePanda (lewat SSH)
#   2. membuka RViz (peta tergambar live)
#   3. terminal INI jadi kontrol keyboard:
#        PANAH = gerak (tahan), SPASI = berhenti, S = SIMPAN PETA, Ctrl+C = selesai
# Klik terminal ini dulu supaya tombol keyboard masuk ke sini (bukan ke RViz).
DIR="$(cd "$(dirname "$0")" && pwd)"
LP="lattepanda@100.64.113.128"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 -i "$HOME/.ssh/lattepanda" "$LP")
RVIZ_PID=""

selesai() {
    [ -n "$RVIZ_PID" ] && kill "$RVIZ_PID" 2>/dev/null
    echo "[STOP] Mematikan mapping di LattePanda..."
    "${SSH[@]}" 'pkill -TERM -f "^bash scripts/buat_peta.sh"' 2>/dev/null || true
}
trap selesai EXIT

echo "[1/3] Menyalakan mapping di LattePanda..."
"${SSH[@]}" 'cd ~/"Robot magang/deteksilokasi" && (nohup setsid bash scripts/buat_peta.sh > /tmp/buat_peta.log 2>&1 < /dev/null &)' || {
    echo "[ERROR] Tidak bisa SSH ke LattePanda ($LP). Cek LattePanda menyala dan WiFi tersambung."; exit 1; }
for i in $(seq 1 40); do
    if "${SSH[@]}" 'grep -q "relay_kontrol siap" /tmp/buat_peta.log' 2>/dev/null; then
        break
    fi
    if "${SSH[@]}" 'grep -qE "^\\[ERROR\\] [^[]" /tmp/buat_peta.log' 2>/dev/null; then
        "${SSH[@]}" 'grep -E "^\\[ERROR\\] [^[]" /tmp/buat_peta.log | head -5'
        exit 1
    fi
    sleep 1
done
echo "[2/3] Membuka RViz..."
"$DIR/buka_rviz_mapping.sh" > /dev/null 2>&1 &
RVIZ_PID=$!
sleep 3
echo "[3/3] Kontrol keyboard (klik terminal ini dulu):"
"$DIR/kontrol_laptop.sh"
