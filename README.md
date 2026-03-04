# PLND Scripts (`plnd_slr.py` dan `plnd_tether.py`)

README ini merangkum perilaku aktual dua script precision landing:
- `plnd_slr.py`
- `plnd_tether.py`

## Ringkasan Umum
Kedua script:
- Capture video dari `rpicam-vid` (`1280x720 @ 30fps`, `yuv420`, autofocus `continuous`).
- Deteksi ArUco (`aruco.DICT_ARUCO_MIP_36H12`) lalu estimasi pose marker.
- Kirim `MAVLink LANDING_TARGET` ke flight controller (frame `MAV_FRAME_BODY_NED`).
- Kontrol LED/strobe via serial (`/dev/ttyUSB0`) mode `1..6`.
- Mencoba auto-disarm saat estimasi jarak vertikal target sangat dekat (`< 0.1 m`).

## Requirements
Install dependency Python:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Kebutuhan sistem:
- `rpicam-vid` (libcamera, Raspberry Pi OS).
- File kalibrasi kamera: `Camera-Conf/picam64.yml` (key: `camera_matrix`, `dist_coeff`).
- Permission serial/GPIO sudah benar (misalnya user masuk grup `dialout`).

## Perbandingan Cepat
| Item | `plnd_slr.py` | `plnd_tether.py` |
|---|---|---|
| Log file | `plnd_roamer.log` | `plnd_tether.log` |
| MAVLink serial | `/dev/ttyS0` @ `921600` | `/dev/ttyS0` @ `921600` |
| Strobe serial | `/dev/ttyUSB0` @ `115200` | `/dev/ttyUSB0` @ `115200` |
| Startup delay sebelum konek MAVLink | Tidak ada | `10` detik |
| Strategi band | Berdasarkan **distance** + hysteresis | Berdasarkan **altitude** |
| TIMESYNC MAVLink | Ya | Tidak |
| Limit kirim `LANDING_TARGET` | Maks `20 Hz` | Tidak dibatasi eksplisit di kode |
| Debug image | `/home/intek/precision_landing/debug.jpg` | `/home/intek/precision_landing/debug.jpg` |

## Detail `plnd_slr.py`
Karakteristik utama:
- Menyimpan log ke `plnd_roamer.log` (sesuai kode saat ini).
- Menunggu heartbeat awal maksimal `10` detik, lalu fallback target `1/1` jika gagal.
- Pakai TIMESYNC untuk timestamp `LANDING_TARGET`.
- Drop frame stale jika umur frame > `250 ms`.
- Ada filter stabilisasi tambahan: LPF sudut, deadband center, reject lateral jump, dan fallback marker khusus band `medium`.

Marker base (`size` dalam mm, posisi relatif):
- ID `5`: `size=30`, `pos=[0.0, 0.0, 0.0]`
- ID `4`: `size=70`, `pos=[0.0, 2.5, 0.0]`
- ID `6`: `size=70`, `pos=[0.0, -2.5, 0.0]`

Pemilihan marker per band (berdasarkan distance ke target):
- `high`, `medium`: ID `4`, `6`
- `low`, `very_low`: ID `5`
- Saat `medium`, jika marker aktif tidak terlihat, script fallback ke marker known yang terlihat (`4/5/6`).

## Detail `plnd_tether.py`
Karakteristik utama:
- Menyimpan log ke `plnd_tether.log`.
- Ada startup delay `10` detik sebelum membuka koneksi MAVLink.
- Menunggu heartbeat awal maksimal `10` detik, lalu fallback target `1/1` jika gagal.
- Timestamp `LANDING_TARGET` memakai `time.time()` (tanpa TIMESYNC).
- Tidak ada limiter `LANDING_TARGET` rate eksplisit seperti di `slr`.

Marker base (`size` dalam mm, posisi relatif):
- ID `4`: `size=70`, `pos=[0.0, 0.0, 0.0]`
- ID `5`: `size=50`, `pos=[0.0, -2.5, 0.0]`
- ID `13`: `size=30`, `pos=[0.0, 58.0, 73.0]`

Pemilihan marker per band (berdasarkan altitude):
- `high`: ID `4`
- `medium`, `low`, `very_low`: ID `4`, `5`
- ID `13` terdefinisi di base marker, tapi tidak dipakai di mapping band aktif saat ini.

## LED/Strobe Mode (Keduanya)
- `1`: `CAMERA_FAIL`
- `2`: `NO_HEARTBEAT`
- `3`: `GPS_NO_FIX`
- `4`: `GPS_FIXED`
- `5`: `RTL`
- `6`: `CRITICAL_LANDING` (RTL/land dan altitude `< 3.0 m`)

## Menjalankan
```bash
python3 plnd_slr.py
python3 plnd_tether.py
```

## Catatan Runtime
- Jika `rpicam-vid` tidak ditemukan: script terminate.
- Jika `/dev/ttyS0` gagal dibuka: script terminate.
- Jika `/dev/ttyUSB0` gagal dibuka: script tetap jalan, tapi kontrol strobe nonaktif.
- `plnd_tether.py` saat gagal buka strobe mencetak pesan `Cannot open /dev/ttyAMA4`, meski port yang dicoba adalah `/dev/ttyUSB0` (mismatch pesan di kode).
