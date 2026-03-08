# 🚢 ÇELEBİLER USV - Autonomous Surface Vehicle

<div align="center">

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red.svg)
![ROS](https://img.shields.io/badge/ROS-2%20Humble-green.svg)
![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)
![TEKNOFEST](https://img.shields.io/badge/TEKNOFEST-2026%20İDA-orange.svg)

**TEKNOFEST 2026 İDA Competition Entry**

*Otonom İnsansız Su Üstü Aracı: GPS navigasyon, engelden kaçınma ve görev yürütme.*

</div>

---

## 📖 Genel Bakış

Bu proje, TEKNOFEST İDA (İnsansız Deniz Aracı) yarışması için Raspberry Pi 4 üzerinde Docker container içinde çalışan otonom USV yazılım altyapısını içerir. Sistem 3 parkurlu görev yapısını destekler:

| Parkur | Görev | Sensörler |
|--------|-------|-----------|
| **Parkur 1** | Nokta Takip (GPS Navigasyon) | Pixhawk + GPS |
| **Parkur 2** | Engelli Nokta Takip | Pixhawk + GPS + Lidar |
| **Parkur 3** | Kamikaze Angajman (Hedef Tespiti) | Pixhawk + GPS + Kamera |

## 🏗️ Sistem Mimarisi

```
HOST (Raspberry Pi 4)
├─ rpicam-vid (TCP 8888)
├─ Lidar modem (UDP 20108)
└─ Docker: ege_ros (Ubuntu 22.04 + ROS2 Humble)
   ├─ mavproxy.py -> UDP 14550 (telemetry) + 14551 (usv_main)
   ├─ telemetry.py (Dashboard + API, Port 8080)
   ├─ usv_main.py (State Machine / Otonom Beyin)
   ├─ cam.py (Port 5000, race modunda web kapalı / onboard aktif)
   ├─ lidar_map.py (Port 5001, race modunda açılmaz)
   └─ IPC dosyaları: /root/workspace/control
```

## 🛠️ Teknoloji Yığını

| Kategori | Teknoloji |
|----------|-----------|
| **Donanım** | Raspberry Pi 4, RPLidar S2E, Pixhawk, Pi Camera IMX708, STM32 |
| **İşletim Sistemi** | Raspberry Pi OS (Host) / Ubuntu 22.04 (Docker) |
| **Middleware** | ROS 2 Humble, MAVProxy |
| **Diller** | Python 3.10+, Bash |
| **Web Framework** | Flask (MJPEG streaming, REST API) |
| **Bilgisayar Görüşü** | OpenCV (renk tespiti, HUD overlay) |
| **Uçuş Kontrolü** | MAVLink via pymavlink (ArduRover) |
| **Veri İşleme** | NumPy |

## 📁 Proje Yapısı

```
CELEBILER_USV/
├── host_scripts/
│   ├── system_start.sh               # Ana başlatma scripti
│   ├── system_stop.sh                # Güvenli kapatma
│   ├── usv_logs.sh                   # Log izleme
│   ├── check_compliance_static.py    # Statik uyum kontrolü
│   └── check_compliance_behavior.py  # Davranışsal uyum kontrolü
├── docker_workspace/
│   ├── src/
│   │   ├── usv_main.py               # Otonom beyin (state machine)
│   │   ├── telemetry.py              # Dashboard + REST API (8080)
│   │   ├── cam.py                    # Kamera işleme / web stream (5000)
│   │   ├── lidar_map.py              # Lidar harita servisi (5001)
│   │   ├── compliance_profile.py     # Tüm eşik/frekans/mod profili
│   │   ├── console_utils.py          # UTF-8 + Windows konsol log formatı
│   │   ├── rc_test.py
│   │   └── auto_dry_test.py
│   ├── scripts/internal_start.sh     # Docker içi başlatma
│   ├── mission.json                  # Görev noktaları
│   └── logs/                         # CSV, video ve servis logları
├── config/usv_mode.cfg               # test | race
├── documents/ida_sartname.md
└── README.md
```

## 🚀 Hızlı Başlangıç

### 1. Repoyu klonla
```bash
git clone https://github.com/celebiler/CELEBILER_USV.git
cd CELEBILER_USV
```

### 2. Görev noktalarını ayarla
`docker_workspace/mission.json` dosyasını düzenle:
```json
{
  "parkur1": [[36.88838, 30.67384], [36.88848, 30.67394]],
  "parkur2": [[36.88818, 30.67394], [36.88808, 30.67404]],
  "parkur3": [[36.88828, 30.67414]],
  "target_color": "RED"
}
```

### 3. Sistemi başlat (Raspberry Pi üzerinde)
```bash
# Test modu (varsayılan)
./host_scripts/system_start.sh

# Yarışma modu
./host_scripts/system_start.sh race
```

### 4. Dashboard erişimi
Tarayıcı: `http://<RPi_IP>:8080`

## 🌐 Web Arayüzleri

| Port | Servis | Açıklama | Yarışma Modu |
|------|--------|----------|--------------|
| `8080` | Mission Control Dashboard | Görev kontrolü, telemetri, API | ✅ Aktif |
| `5000` | Kamera Stream (`cam.py`) | 720p MJPEG + renk tespiti overlay | ❌ Web kapalı, onboard işleme aktif |
| `5001` | Lidar Harita (`lidar_map.py`) | Canlı 2D engel haritası | ❌ Başlatılmaz |

## 🎮 Dashboard Özellikleri

Tek sayfa Mission Control arayüzü:

- **Mission paneli**: Görev timer, parkur durumu, progress
- **Kontrol butonları**: ▶ Görevi Başlat / ⛔ Acil Durdur
- **Telemetri kartları**: Batarya, GPS, hız/yön, atmosfer, RC, motor PWM
- **Sistem metrikleri**: CPU, RAM, sıcaklık
- **5 Hz dashboard polling** + **kritik olay akışı** (`/api/events`)

## 🔧 Temel Özellikler

- **Mod ayrımı**: `test` (tam geliştirme görünürlüğü), `race` (şartnameye uygun kısıtlı yayın).
- **Race start politikası**: Yarış modunda görev start **yalnızca RC CH5 >= 1700** ile yapılır. API start race modunda 403 döner.
- **Dosya tabanlı IPC**: `telemetry.py` ↔ `usv_main.py` (`/root/workspace/control`).
- **Fail-safe (çift katman)**: Telemetri link heartbeat ana tetik, onboard heartbeat yedek tetik (5s warning, 30s hold).
- **Manuel/otonom arbitraj**: Mission aktif/lock/estop/AUTO-GUIDED-HOLD modlarında manuel motor override neutral lock.
- **RC timeout güvenliği**: RC güncel değilse motor çıkışları neutral hold.
- **MAVProxy port paylaşımı**: Pixhawk verisi aynı anda `telemetry.py` ve `usv_main.py` tarafından tüketilir.
- **Simülasyon modu**: Donanım yoksa sahte telemetri ile akış devam eder.
- **Uyumluluk kontrolleri**: `check_compliance_static.py` ve `check_compliance_behavior.py`.
- **Windows terminal desteği**: `console_utils.py` ile UTF-8, okunabilir zaman damgalı ve renkli log formatı.

## 🌟 İnovasyonlar

### ✅ Entegre Edilen İnovasyon

- **Sensör Füzyonu (Kamera + Lidar Hayalet Hedef Koruması)**:
  - Parkur 2 ve Parkur 3 kararlarında kamera tespitleri, aynı doğrultuda Lidar doğrulaması olmadan kullanılmaz.
  - Lidar doğrulaması yoksa (veya veri stale ise) tespit fail-safe olarak reddedilir.
  - Kamera tarafındaki ham alanlar korunur (`*_raw`), otonom kararlar fused alanlar üzerinden yürür.
  - `gate_passed_event` sayımı için yakın zamanda fused gate doğrulaması şartı vardır (yalancı kapı geçişlerini azaltır).
  - Görev durumuna `sensor_fusion` bloğu eklenmiştir: `enabled`, `policy`, `ghost_gate_count`, `ghost_target_count`, `last_confirmed_gate_dist_m`, `last_confirmed_target_dist_m`, `lidar_ready`.
  - `/api/data` çıktısına `sensor_fusion` alanı ve `report_view.navigation_health.sensor_fusion` özeti eklenmiştir.

- **Dinamik Hız Profillemesi (Dynamic Speed Profiling / Trajectory Planning)**:
  - Parkur 1 ve Parkur 2'de hız, `heading_error` şiddetine göre bantlı profil ile otomatik düşürülür (`straight/soft/medium/hard`).
  - Dönüş bantlarında parkura göre minimum hız tabanı uygulanır (stall önleme), ancak hız hiçbir zaman base komutunu aşmaz.
  - P2'de merkez engel/failsafe hız limiti korunur ve dinamik profil çıktısından sonra `FAILSAFE_SLOW_MPS` ile sınırlandırılır.
  - Görev durumuna `dynamic_speed_profile` bloğu eklenmiştir: `enabled`, `mode`, `scope`, `active`, `band`, `factor`, `heading_error_abs_deg`, `base_speed_mps`, `output_speed_mps`.
  - `/api/data` ve `report_view.navigation_health.dynamic_speed_profile` ile profil görünürlüğü sağlanmıştır.
  - Konsolda throttle'lı `[DYN_SPEED]` logları ile hız kararları izlenebilir.

### 🧪 Sıradaki İnovasyon Adayları

- **Akıntı ve Rüzgar Düzeltme Asistanı** (integral tabanlı crab steering)
- **Termal ve Enerji Hayatta Kalma Modu** (ECO/Limp)
- **Lidar Kendi-Gövdesini Gizleme** (self-masking / ignore zone)
- **Geo-Fence & Safe-Halt Virtual Anchor** (sanal çit + bağlantı kaybında pozisyon tutma)

## 🔌 API Özeti

| Endpoint | Yöntem | Açıklama |
|----------|--------|----------|
| `/api/data` | GET | Dashboard verisi + report_view + health + `sensor_fusion` + `dynamic_speed_profile` alanları |
| `/api/mission_status` | GET | Görev aktiflik ve geçen süre |
| `/api/events` | GET | Kritik olay akışı (long-poll) |
| `/api/start_mission` | POST | Test modunda görev başlatma |
| `/api/emergency_stop` | POST | Acil durdurma |

Notlar:
- Race modunda `/api/start_mission` politika gereği engellidir.
- Olay akışında `gate_gecildi`, `failsafe`, `estop`, `timeout` yayınlanır.

## 📡 Port Haritası

| Port | Protokol | Kullanım |
|------|----------|----------|
| `8080` | HTTP | Dashboard + API |
| `5000` | HTTP/MJPEG | Kamera web stream (test) |
| `5001` | HTTP | Lidar haritası (test) |
| `8888` | TCP | Ham kamera (Host → Docker) |
| `14550` | UDP/MAVLink | mavproxy → telemetry.py |
| `14551` | UDP/MAVLink | mavproxy → usv_main.py |
| `20108` | UDP | RPLidar S2E |

## 📜 Lisans

Bu proje MIT lisansı altındadır.

## 👥 Takım

**AKDENİZ ÜNİVERSİTESİ — ÇELEBİLER İDA TAKIMI**

---

<div align="center">
Made with ❤️ for TEKNOFEST 2026
</div>
