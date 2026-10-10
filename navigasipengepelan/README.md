# navigasipengepelan

Robot mengepel **seluruh area kosong peta secara otomatis** dengan pola zig-zag, lalu **kembali ke titik awal**. Mirip robot pel Xiaomi.

## Menjalankan (satu perintah di laptop)

```bash
cd ~/Documents/"Robot magang"/navigasipengepelan
./pengepel_laptop.sh
```
Perintah ini menyalakan semua program di LattePanda lewat SSH, membuka RViz, lalu menjadikan terminal yang sama sebagai kontrol. Klik terminalnya dulu:

| Tombol | Fungsi |
|---|---|
| **G** | mulai mengepel (rute dibuat otomatis dari posisi robot saat itu) |
| **SPASI** | berhenti darurat / lanjut lagi |
| **P** | lihat ulang rute (pratinjau) |
| **H** | hapus batas area |
| Ctrl+C | selesai (robot dihentikan, program di LattePanda dimatikan) |

Urutannya:
1. Tunggu sampai muncul `[LOKASI] KETEMU`. Robot sudah tahu posisinya (dari `deteksilokasi`), dan rute pratinjau muncul di RViz.
2. **Batas area (sekali saja):** di RViz pilih tool **Publish Point**, lalu klik sudut-sudut area yang boleh dipel secara berurutan. Garis merah adalah batasnya, dan robot tidak akan keluar dari situ, termasuk melewati kaca yang tembus LiDAR. Rute pratinjau langsung diperbarui. Batas tersimpan otomatis untuk pemakaian berikutnya. Tekan **H** untuk menghapus dan menggambar ulang.
3. Tekan **G**. Rute dibuat ulang dari posisi robot saat itu, lalu robot mulai jalan. Di RViz, garis biru tebal adalah baris pel, garis abu-abu tipis adalah jalur pindah, dan yang sudah dilewati jadi hijau.
4. Robot mengepel baris demi baris. Di ujung tiap baris, robot berhenti lalu berputar ke baris berikutnya.
5. Setelah semua baris selesai, robot pulang ke titik awal dan muncul `SELESAI`.

Di LattePanda tidak perlu menjalankan apa-apa. LattePanda cukup menyala, tersambung ke WiFi yang sama, dan LiDAR serta STM32 tercolok.

## Cara kerja

**Rute** (`navigasipengepelan/coverage.py`), metode *boustrophedon coverage path planning*:
1. Area kosong diambil dari peta `deteksilokasi/maps/peta_baru.yaml`. Bintik kecil (noise atau kaki kursi) dan lubang kecil diabaikan, dan garis sinar tipis ke luar ruangan dibuang.
2. Titik tengah robot dijaga minimal 30 cm dari dinding dan benda besar.
3. Arah dinding utama ruangan dicari, lalu baris dibuat **lurus** sejajar dinding dengan jarak **40 cm** (lebar pel 50 cm, tumpang tindih 10 cm). Polanya **lurus bolak-balik** seperti robot Xiaomi: ujung-ujung baris di tiap blok disejajarkan ke satu garis lurus, sehingga sambungan antar baris pendek dan rapi.
4. Area dibagi menjadi blok-blok (*cell decomposition*). Tiap blok dipel zig-zag sampai habis, lalu robot pindah ke blok terdekat lewat jalur aman (Dijkstra).
5. Robot pulang ke titik awal lewat jalur aman.

**Cara jalan** (`pengepel_node.py`): memakai logika `robotfixproblemkembali3` yang sudah terbukti, sebagai turunan kelasnya tanpa mengubah `kembali3`. Logika itu mencakup koreksi jalur, berhenti kalau ada halangan di depan, tombol darurat, dan titik yang terlewat tetap dihitung. Tambahannya:
- **Berhenti lalu berputar di tempat** di setiap ujung baris.
- **Ujung baris dekat dinding** dianggap tercapai kalau dinding sudah di depan dan sisa jaraknya ≤ 0,8 m.
- **Terhalang di tengah baris lebih dari 8 detik:** sisa baris dilewati, lalu lanjut ke baris berikutnya.
- **Batas waktu per titik**, bukan sejak mulai jalan. Ini memperbaiki lompat titik palsu di baris panjang.

## Uji tanpa robot

```bash
python3 test/gambar_rute.py maps/peta_baru.yaml -2.0 -0.8 rute.png            # gambar rute
source /opt/ros/jazzy/setup.bash
python3 test/simulasi_pengepelan.py maps/peta_baru.yaml -2.0 -0.8 0 sim.png   # robot tiruan
```
Hasil simulasi di peta baru (ujung baris dirapikan): 63 baris, 375 titik, **selesai semua dan kembali ke titik awal** (berhenti 0,44 m dari titik awal), sekitar 21 menit waktu tiruan, **94% area aman terkena pel** (lebar pel 50 cm). Simulasi ini tanpa halangan dinamis dan tanpa LiDAR.

## Parameter (`config/pengepel_params.yaml`, bagian bawah)

| Parameter | Default | Arti |
|---|---|---|
| `row_spacing_m` | 0,40 | jarak antar baris |
| `wall_clearance_m` | 0,30 | jarak minimal titik tengah robot ke dinding di peta |
| `min_obstacle_m2` | 0,04 | benda lebih kecil dari ini diabaikan saat membuat rute |
| `corner_wall_reach_m` | 0,8 | ujung baris dianggap tercapai saat dinding sudah dekat |
| `obstacle_skip_s` | 8 | terhalang lebih lama dari ini, sisa baris dilewati |

Belum ada: menyusuri pinggir dinding (*edge cleaning*) dan dock pengisian.
