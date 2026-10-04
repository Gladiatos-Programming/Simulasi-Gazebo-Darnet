# CenterizedReference: baseline ticks tanpa offset demo

4 Oktober 2026. Script baru terpisah dari `Centerized.py` dan bridge lama.
Belum diuji pada motor fisik oleh asisten. Tidak membuat/mengubah URDF.

## Target dan perbedaannya dengan Centerized lama

```text
ID 8  (Paha Kiri Putar)  = 1946
ID 12 (Paha Bawah Kiri)  = 2081
ID lain 1..20           = 2048
```

Nilai adalah USER-REPORTED GOAL reference, bukan hasil pengukuran encoder terbaru.
Present 1948/2083 pada screenshot lama tidak mengganti target di atas otomatis.
Script memakai ID/ticks langsung melalui SDK Protocol 1.0, bukan JointTrajectory.
Tidak mengimpor/menjalankan bridge, membaca offsets untuk konversi, atau mengganti
`zero_offsets.json`. Salinan config lama dalam log hanya untuk provenance.

`Centerized.py` lama tetap mengirim 0 radian ke bridge yang memakai offsets lama;
itu tidak menghasilkan target mentah di atas jika config demo masih dipakai.
Jangan menjalankan script baru dan bridge/Wizard bersamaan.

## Pasang versi baru di Jetson

Perubahan ini harus sudah DISALIN/disinkronkan ke repo Jetson sebelum build.
Jika belum dipush ke remote, `git pull` tidak akan mendapat perubahan lokal ini.
Jangan reset/overwrite perubahan Jetson. Topang robot dahulu sebelum menghentikan
bridge lama karena shutdown default dapat mematikan torque.

Di root workspace yang berisi `src/darnet_description`:

```bash
git status
colcon build --packages-select darnet_description --symlink-install
source install/setup.bash
```

Tidak perlu menjalankan bridge baru untuk memakai tool ini. Startup bridge lama
dapat enable torque dan mengejar goal yang tersimpan. Script referensi baru tidak
enable/disable torque otomatis: jika torque OFF, inspection boleh, execute ditolak.
Siapkan torque/pose hanya melalui prosedur operator yang sebelumnya diverifikasi,
dengan penopang dan goal yang diketahui. Jangan menyalakan torque pada pose/goal
sembarang untuk sekadar melewati guard.

## 1. Preview offline: default tanpa hardware

```bash
ros2 run darnet_description CenterizedReference
```

Menampilkan 20 target. Tidak membuka serial port, tidak mengimpor SDK hardware,
tidak menulis servo, tidak membuat folder. `--help` juga offline.
Bisa tanpa ROS install jika SDK belum diperlukan:

```bash
python3 src/darnet_description/darnet_description/CenterizedReference.py
```

## 2. Inspection read-only

Robot ditopang dan semua aplikasi pemakai U2D2 sudah ditutup:

```bash
ros2 run darnet_description CenterizedReference --inspect
```

Default port `/dev/ttyUSB0`, baud 1000000. Ganti melalui `--port`/`--baud` jika
setup berbeda. Inspeksi tidak mengubah torque/goals/speed/EEPROM. Output baru
`centerized_reference_<UTC timestamp>` berisi register awal, blocker gerakan,
snapshot file terpasang serta metadata. Blocker bukan permintaan mengubah
EEPROM/mode otomatis; kirim hasilnya jika tidak jelas.

Execute mensyaratkan semua 20 ID dan model referensi: IDs1-6/19-20 legacy MX-28,
IDs7-18 legacy MX-64. Jika fisik/model/protocol berbeda, berhenti dan tinjau mapping.
Tool menolak model tidak dikenali sebelum akses register legacy.

## 3. Gerakan hanya untuk robot yang SUDAH dekat Centerized

Pastikan robot ditopang/tether, lintasan kecil bebas benturan, torque sudah ON,
tidak ada pengirim lain pada port, dan pemutus daya mudah dijangkau.

```bash
ros2 run darnet_description CenterizedReference \
  --execute --robot-supported --exclusive-port-confirmed
```

Tool membaca semua register, menampilkan current/target dan meminta operator
mengetik `MOVE CENTERIZED`. Tanpa persetujuan itu tidak ada write. Setelah prompt,
seluruh data dibaca ulang; perubahan pose selama konfirmasi menolak gerak.

Guard ENGINEERING (bukan measured safe ROM atau collision detector):

- Semua ID harus single-turn joint mode, tidak wheel/multiturn/torque-control;
  divider=1 dan multi-turn offset=0 sebagai baseline konservatif, walaupun dua
  register itu diabaikan dalam joint mode oleh firmware. Tidak diubah otomatis.
- Status-return-level=2 agar write beracknowledgement, torque sudah ON dan limit
  nonzero, tidak ada registered instruction pending, posisi awal diam.
- Present/existing-goal/reference dalam CW/CCW limit. Batas register tersebut
  bukan bukti mekanik aman atau ROM collision-free.
- Present dan goal harus dekat target (maksimum 128 ticks, ~11,25 derajat).
  Goal-present awal maksimum 16 ticks. Tool bukan stand-up/get-up controller;
  jika terlalu jauh, jangan memperbesar guard untuk memaksa pose.
- Suhu harus di bawah 55 C dan juga 5 C di bawah device temperature limit;
  tegangan harus dalam limits device. Ini bukan battery/thermal certification.
- Perubahan goal maksimum 4 ticks per frame, minimal interval 0,2 s. Semua speed
  RAM di-set nonzero 20 dan diverifikasi SEBELUM goal pertama. Tracking error
  >24 ticks, perubahan mode/limits/speed/goal, torque drop, suhu/voltage error,
  read/write error menghentikan kelanjutan trajectory.
- Tiga feedback settled berturut-turut diperlukan untuk label near target
  (present dalam 8 ticks, moving=0). Jika semua goal sudah target sejak awal,
  tidak ada write; script hanya melaporkan already-reference-goals.

Hanya RAM Goal Position(30) dan Moving Speed(32) yang ditulis. Torque Enable,
Torque Limit, gains, EEPROM, limits, offsets dan firmware TIDAK ditulis.
Moving Speed lama TIDAK direstore otomatis: nilai 0 berarti uncapped speed,
bukan stop. Nilai cap 20 akan tetap ada setelah gerakan sampai aplikasi lain
mengubahnya atau power cycle. Log awal menyimpan nilai sebelum perubahan ini.

**Ctrl+C/error BUKAN emergency stop:** script berhenti mengirim write baru dan
menutup port, tanpa torque-off/rollback. Last-written goal dapat masih bergerak,
dan fault write tanpa ACK bisa saja sudah diterima perangkat. Jaga penopang,
awasi robot dan gunakan prosedur pemutus daya dengan robot aman jika diperlukan.
Gerakan seluruh joint berurutan dalam frame juga bukan simulasi collision;
komunikasi failure bisa meninggalkan sebagian ID pada goal berbeda.

## 4. Verifikasi pose dan ambil snapshot

Setelah tool keluar, torque/goal tidak otomatis dimatikan. Periksa fisik badan,
kepala/kaki tegak, tangan sesuai nol CAD, dan kedua sol sejajar. Foto depan/samping.
Present tidak wajib sama persis dengan goal di bawah beban. Jangan mengoreksi
horn/offset/ticks berdasarkan selisih satu pembacaan tanpa analisis.

Jika pose benar, jalankan pembaca read-only pada environment sama, tanpa
mengedit/rebuild controller/config di antara kedua tool:

```bash
ros2 run darnet_description CaptureEncoderReference --samples 5 --pose-confirmed
```

Jika belum benar, hilangkan `--pose-confirmed` dan kirim laporan/fotonya apa adanya.
Kirim KEDUA folder: `centerized_reference_*` dan `encoder_capture_*`, serta foto.
Folder pertama membuktikan ticks/motion history dan Moving Speed sebelum tool;
folder kedua merekam present/goal/config setelah pose diverifikasi.

## Config terpasang yang dimaksud

| Bukti | Apa yang diambil | Tujuan |
| --- | --- | --- |
| File resolved dari environment | Bridge `ComsROS2U2D2.py`, `Centerized.py`, `CenterizedReference.py`, `CaptureEncoderReference.py` | Mengetahui versi install yang dipakai, ID/sign/conversion/shutdown behavior |
| Config package share | `config/zero_offsets.json` beserta hash | Mengetahui offsets demo yang masih terpasang; tidak otomatis diterapkan/disahkan |
| Environment terbatas | Python executable/version, ROS_DISTRO, AMENT_PREFIX_PATH, resolved file paths/hashes | Menemukan install/workspace overlay yang berbeda dari source repo |
| Control table servo | Model/firmware/ID/baud/return delay/status return, CW/CCW, offset/divider, torque, PID, goal/present/speed/load, voltage/temp/limits | Mengetahui konfigurasi hardware dan feedback nyata pada saat pembacaan |
| Khusus MX-64 | Torque Control Mode Enable(70) | Menolak mode torque yang tidak menerima goal position normal |

Snapshot file adalah environment proses tool, bukan dump konfigurasi dari memori
bridge yang sudah berjalan. Parameter ROS seperti keep_torque_on_exit tidak
dibaca otomatis dari proses bridge sebelumnya. Jika digunakan, catat command
startup/parameter tersebut; jangan menganggap isi file membuktikan nilai live.
Capture kini profile v2 extended; model-specific register diperiksa sebelum
dibaca. Present Load adalah nilai inferred, bukan sensor torque presisi.

Register/mode/RAM speed diverifikasi terhadap sumber primer ROBOTIS:
[MX-28 legacy](https://emanual.robotis.com/docs/en/dxl/mx/mx-28/),
[MX-64 legacy](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/).
Protocol 2.0/X-series menggunakan tabel berbeda dan tidak didukung tool ini.

## Tes offline

```bash
python3 -B -m unittest discover -s src/darnet_description/test -p test_centerized_reference.py -v
python3 -B -m unittest discover -s src/darnet_description/test -p test_encoder_capture.py -v
```

Mocks memeriksa target, preview, preflight/no-write, ramp, allowed writes,
readback, mode/stall/competing writer dan extended reader. Bukan hardware safety
approval. Asisten hanya menjalankan preview dan tes offline, bukan --inspect/--execute.
