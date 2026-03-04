# PLND Scripts (Roamer & Tether)

Dokumentasi singkat untuk dua script pendaratan presisi:
- `plnd_roamer.py` (varian roamer)
- `plnd_tether.py` (varian tether)

## Ringkas
Kedua script:
- Membaca video dari `rpicam-vid` (libcamera) dan mendeteksi ArUco.
- Mengirim data `LANDING_TARGET` ke autopilot via MAVLink.
- Mengendalikan LED/strobe via serial (`/dev/ttyUSB0`).
- Menulis log ke file: `plnd_roamer.log` atau `plnd_tether.log`.

## Requirements (sesuai script)
Python packages:
```
numpy==2.2.6
pymavlink==2.4.49
pyserial==3.5
PyYAML==6.0.2
RPi.GPIO==0.7.1
opencv-contrib-python==4.12.0.88
```

System tools:
- `rpicam-vid` (bagian dari libcamera di Raspberry Pi OS).

## Instalasi
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Konfigurasi yang dibutuhkan
1) File kalibrasi kamera:
   - Path yang dipakai script: `Camera-Conf/picam64.yml`
   - Format harus berisi `camera_matrix` dan `dist_coeff`.

2) Port serial:
   - Strobe LED: default `/dev/ttyUSB0`
   - MAVLink:
     - Roamer: default `/dev/ttyS0` (baud 921600)
     - Tether: default `/dev/ttyAMA4` (baud 921600)
   - Catatan: nama port serial bisa berbeda di setiap drone (tergantung wiring/OS/USB).
     Sesuaikan nilai port di script jika perangkat Anda memakai port lain.

3) ArUco markers:
   - Dictionary: `aruco.DICT_ARUCO_MIP_36H12`
   - Ukuran dan posisi marker sudah di-hardcode pada masing-masing script.
   - Gambaran posisi (satuan mm, koordinat relatif ke marker utama):
     - Roamer:
       - ID 5: size 50, pos [0, 0, 0] (marker utama)
       - ID 4: size 70, pos [0, 2.5, 0]
       - ID 6: size 70, pos [0, -3.5, 0]
       - ID 8: size 30, pos [-106, 178, 110]
       - ID 9: size 30, pos [106, 177, 110]
     - Tether:
       - ID 4: size 70, pos [0, 0, 0] (marker utama)
       - ID 5: size 50, pos [0, -2.5, 0]
       - ID 13: size 20, pos [0, 58, 73]

## Menjalankan
Roamer:
```
python3 plnd_roamer.py
```

Tether:
```
python3 plnd_tether.py
```

## Catatan
- Jika `rpicam-vid` tidak ditemukan, script akan berhenti dan memberi pesan error.
- Jika port serial tidak tersedia, script akan lanjut tetapi fitur terkait tidak aktif.
- Pastikan permission serial (mis. grup `dialout`) dan akses GPIO sudah benar.
