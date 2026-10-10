# deteksilokasi

Robot menentukan sendiri posisinya di peta. Kalau robot diangkat lalu ditaruh di tempat lain, robot mencari lokasinya lagi tanpa perlu klik **2D Pose Estimate**.

## Cara kerja

1. Peta ruangan diubah jadi "peta kedekatan dinding": tiap titik tahu seberapa dekat ia ke dinding.
2. Scan LiDAR dicoba di **semua posisi bebas di peta** (tiap 15 cm) dan **semua arah** (tiap 5°). Skor = seberapa pas ujung-ujung sinar LiDAR jatuh di dinding peta (0 sampai 1).
3. Lima kandidat terbaik dihaluskan sampai ketelitian sekitar 2,5 cm dan 0,5°.
4. Kalau kandidat terbaik cukup yakin, posisinya dikirim ke AMCL lewat `/initialpose`, sama seperti klik 2D Pose Estimate. AMCL lalu melanjutkan pelacakan seperti biasa.
5. **Robot dipindah:** kalau scan tidak lagi cocok dengan posisi AMCL, roda (odom) diam, dan scan sudah stabil (robot sudah ditaruh) selama sekitar 3 detik, robot mencari lokasi lagi.

Hasil dianggap **ragu** dan tidak dikirim kalau skornya di bawah 0,65 atau ada tempat lain yang hampir sama cocoknya. Ini bisa terjadi di lorong yang bentuknya mirip. Robot mencoba lagi tiap 5 detik. Kalau tetap ragu, geser robot sedikit atau klik 2D Pose Estimate manual.

Node ini tidak pernah menggerakkan motor.

## Menjalankan

Cukup satu perintah di laptop (WiFi sama dengan LattePanda):
```bash
cd ~/Documents/"Robot magang"/deteksilokasi
./deteksi_laptop.sh
```
Perintah ini menyalakan deteksi lokasi di LattePanda lewat SSH, lalu membuka RViz. Menutup RViz ikut mematikan program di LattePanda.

Di RViz:
- **panah hijau "SAYA DI SINI"**: hasil deteksi, dengan skor di atasnya;
- **panah oranye**: kandidat lain yang mirip;
- **panah kuning**: posisi AMCL, yang ikut pindah ke panah hijau.

Paksa cari ulang kapan saja:
```bash
ros2 topic pub --once /deteksi_lokasi/cari std_msgs/msg/Empty
```
Status terakhir: `ros2 topic echo /deteksi_lokasi/status`.

## Membuat peta baru (satu perintah di laptop, robot dikontrol tombol panah)

```bash
cd ~/Documents/"Robot magang"/deteksilokasi
./buat_peta_laptop.sh
```
Perintah ini menyalakan mapping di LattePanda, membuka RViz, lalu menjadikan **terminal yang sama** sebagai kontrol keyboard. Klik terminalnya dulu supaya tombol masuk ke sana:

| Tombol | Gerak |
|---|---|
| ↑ / ↓ | maju / mundur (tahan) |
| ← / → | putar kiri / kanan (tahan) |
| SPASI | berhenti |
| **S** | **SIMPAN PETA** ke `maps/peta_baru.yaml` di LattePanda |
| Ctrl+C | selesai (RViz dan mapping dimatikan) |

Robot hanya bergerak selama tombol panah ditahan. Kalau WiFi putus, robot berhenti sendiri dalam 0,5 detik.

Tips supaya peta bersih: jalan pelan, ruangan kosong dari orang, susuri dinding, kembali ke titik awal, lalu tekan **S** dan tunggu tulisan `[PETA] TERSIMPAN`.

Setelah `maps/peta_baru.yaml` ada, deteksi lokasi otomatis memakai peta baru itu (`./deteksi_laptop.sh`). Peta lama `robotpel` tidak diubah. `kembali3` tetap memakai peta lama.

## Uji tanpa robot

```bash
python3 test/uji_matcher_offline.py ../robotpel/maps/room.yaml
```
Uji ini menaruh robot di 12 pose acak di peta asli, dengan scan tiruan yang diberi noise 3 cm, 10% sinar hilang, dan 15% pantulan benda yang tidak ada di peta. Hasil pertama: 11/12 benar (error sekitar 2 cm / kurang dari 1°), sekitar 2 detik per pencarian di laptop. Satu yang gagal memang kasus ragu (skor 0,57 lawan 0,56); di robot, kasus itu tidak dikirim ke AMCL.

## Parameter penting (`config/deteksi_lokasi_params.yaml`)

| Parameter | Default | Arti |
|---|---|---|
| `accept_score` | 0,65 | skor minimum supaya hasil dipakai |
| `accept_margin` | 0,05 | selisih minimum dengan tempat lain yang mirip |
| `lost_score` | 0,40 | di bawah ini, scan dianggap tidak cocok dengan posisi sekarang |
| `lost_seconds` | 3,0 | berapa lama tidak cocok sebelum mencari ulang |
| `search_stride_m` | 0,15 | jarak antar posisi yang dicoba |

Peta, filter scan, dan setting AMCL memakai milik `robotpel` apa adanya, tanpa diubah.
