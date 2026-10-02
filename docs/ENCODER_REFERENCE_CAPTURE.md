# Encoder reference capture (MX Protocol 1.0 / U2D2)

Tujuan: merekam ticks aktual dan konfigurasi terpasang sebagai kandidat referensi.
Script tidak mengkalibrasi, tidak menerapkan offset demo, tidak mengubah mode,
dan tidak memanggil write servo. Offset demo tetap diarsipkan, bukan disahkan.
Pengambilan pose Centerized lama bukan otomatis kalibrasi geometris yang benar.

## 1. Pasang versi baru di Jetson

Jangan mengganti bridge saat robot sedang berdiri tanpa pengaman. Untuk berhenti
dari bridge versi lama, topang robot dahulu: versi lama mematikan torque.
Jika sudah ada modifikasi lokal di Jetson, periksa `git status` dan simpan perubahan
tersebut; jangan reset/overwrite. Di folder repo:

```bash
git status
git switch ariq
git pull --ff-only origin ariq
colcon build --packages-select darnet_description --symlink-install
source install/setup.bash
```

Gunakan workspace/environment yang sama pada terminal-terminal berikutnya.
Jangan menyalakan ulang bridge di pose sembarang: startup tetap mengaktifkan
torque seperti bridge sebelumnya, dan servo bisa mengejar target yang tersimpan.
Parameter baru HANYA mengubah perilaku shutdown, bukan startup/mapping/controller.

## 2. Jalankan bridge dengan handoff torque tetap ON

Dengan pengaman robot dan prosedur startup yang sebelumnya sudah berhasil:

```bash
ros2 run darnet_description ComsROS2U2D2 --ros-args -p keep_torque_on_exit:=true
```

Parameter harus diberikan saat startup. Default `false` mempertahankan perilaku
lama (torque OFF saat shutdown). Jangan mengandalkan perubahan parameter live.
Bridge masih menggunakan offset demo saat menerima command. Jangan hapus offset
atau menggantinya dengan nol untuk prosedur ini.

Jalankan Centerized dengan prosedur lama yang sudah berhasil, tunggu berhenti,
lalu verifikasi pose fisik terhadap pose nol model dan ambil foto depan/samping.
Jika pose tidak sesuai, rekam saja sebagai pose observasi (tanpa `--pose-confirmed`);
jangan menganggap ticks-nya sebagai nol baru.

Hentikan script pengirim gerakan, lalu Ctrl+C bridge. Dengan parameter tersebut,
bridge menutup port TANPA menulis Torque Enable ataupun Goal Position.
Servo tetap mengikuti goal terakhir selama torque/daya tersedia; bukan pose lock
baru, bukan jaminan tetap berdiri. Tidak ada supervisor controller setelah exit.
Tetap awasi, siapkan penyangga/tether serta pemutus daya. Error/overload, hilangnya
daya, atau aplikasi lain dapat menghilangkan torque. Jangan meninggalkan robot.

## 3. Pilih pembaca CSV ATAU Dynamixel Wizard

Jangan jalankan dua aplikasi pada U2D2 bersamaan. Pastikan bridge sudah berhenti.

### Pembaca CSV (direkomendasikan untuk semua 20 ID)

```bash
ros2 run darnet_description CaptureEncoderReference --pose-confirmed
```

`--pose-confirmed` hanya pernyataan operator bahwa pose geometris benar, bukan
hasil verifikasi script. Hilangkan flag jika belum yakin. Default: `/dev/ttyUSB0`,
1 Mbps, Protocol 1.0, ID 1..20, tiga putaran sampel, jeda 0.5 detik.
Port lain/subset ID dapat dipilih tanpa mengubah servo:

```bash
ros2 run darnet_description CaptureEncoderReference --port /dev/ttyUSB0 --ids 1 2 --samples 3
```

Output folder baru `encoder_capture_<timestamp>` di direktori kerja:

- `encoder_reference_centerized.csv`: ID/model, raw present/goal ticks, torque,
  limits, divider, voltage (raw 0.1 V unit), suhu, timestamp, dan error.
- `ComsROS2U2D2.py`, `Centerized.py`, `zero_offsets.json`: snapshot jika ditemukan.
- `capture_metadata.json`: sumber snapshot, konfirmasi operator, status/error.

Snapshot adalah environment proses capture, bukan bukti konfigurasi dalam memori
bridge sebelumnya. Jangan mengedit/rebuild konfigurasi antara startup dan capture.
Script hanya menerima model legacy MX-28 (29)/MX-64 (310); model lain ditandai error.
Ticks disimpan RAW: tidak diasumsikan sudut atau signed multi-turn. Baris error
bukan posisi nol. Baca `read_failures` dan `configuration_errors` pada metadata.
Pembacaan berurutan, bukan semua joint pada waktu persis sama. Capture tidak
mematikan torque juga saat selesai, Ctrl+C, atau error.

### Dynamixel Wizard

Setelah bridge/capture keluar: scan port U2D2, Protocol 1.0, 1 Mbps, ID 1..20.
Baca Control Table saja. Jangan mengubah goal, torque, limits, mode atau firmware.
Jika `Torque Enable` masih 1, itulah status yang tersisa dari bridge, bukan Wizard
atau capture yang mengaktifkannya. Snapshot/CSV otomatis tetap memerlukan script.

## 4. Setelah pengambilan

Kirim folder output dan foto pose. Jangan menimpa `zero_offsets.json` dari CSV.
Arah ticks serta command limit masih memerlukan verifikasi terpisah. Ini juga
tidak mengubah `/joint_states` bridge yang masih mencerminkan command, bukan feedback.
Topang robot sebelum torque/daya dimatikan; jangan membiarkan servo menahan beban
tanpa pengawasan. Capture read-only sengaja tidak menyediakan perintah torque OFF.

## Offline verification

```bash
python3 -m unittest discover -s src/darnet_description/test -p test_encoder_capture.py -v
```

Tes menggunakan mock; belum merupakan pengujian Jetson/U2D2/motor fisik.
