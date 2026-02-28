# 🚢 ÇELEBİLER USV - Autonomous Surface Vehicle

<div align="center">

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red.svg)
![ROS](https://img.shields.io/badge/ROS-2%20Humble-green.svg)
![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)
![TEKNOFEST](https://img.shields.io/badge/TEKNOFEST-2026%20İDA-orange.svg)

**TEKNOFEST 2026 İDA Competition Entry**

*Otonom İnsansız Su Üstü Aracı — GPS navigasyon, engelden kaçınma ve görev yürütme.*

</div>

---

## 📖 Genel Bakış

Bu proje, TEKNOFEST İDA (İnsansız Deniz Aracı) yarışması için Raspberry Pi 4 üzerinde Docker container'da çalışan otonom USV yazılım altyapısını içerir. Sistem 3 parkurlu görev yapısını destekler:

| Parkur | Görev | Sensörler |
|--------|-------|-----------|
| **Parkur 1** | Nokta Takip (GPS Navigasyon) | Pixhawk + GPS |
| **Parkur 2** | Engelli Nokta Takip | Pixhawk + GPS + Lidar |
| **Parkur 3** | Kamikaze Angajman (Hedef Tespiti) | Pixhawk + GPS + Kamera |

## 🏗️ Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────────────┐
│                      RASPBERRY PI 4 (HOST)                       │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────────┐ │
│  │ RPi Camera  │  │ RPLidar S2E  │  │  Pixhawk (MAVLink)      │ │
│  │ TCP :8888   │  │ UDP :20108   │  │  /dev/ttyACM*           │ │
│  └──────┬──────┘  └──────┬───────┘  └───────────┬─────────────┘ │
│         │                │                      │               │
│         │                │               ┌──────┴──────┐        │
│         │                │               │  MAVProxy   │        │
│         │                │               │ UDP :14550  │        │
│         │                │               │ UDP :14551  │        │
│         │                │               └─────────────┘        │
├─────────┴────────────────┴──────────────────────────────────────┤
│                     DOCKER CONTAINER (ege_ros)                   │
│                     Ubuntu 22.04 + ROS 2 Humble                  │
├─────────────────────────────────────────────────────────────────┤
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐ │
│  │  cam.py    │  │lidar_map.py│  │telemetry.py│  │usv_main.py │ │
│  │  Port:5000 │  │  Port:5001 │  │  Port:8080 │  │  (Brain)   │ │
│  │ 720p MJPEG │  │ Lidar Map  │  │ Dashboard  │  │ State Mach.│ │
│  └────────────┘  └────────────┘  └──────┬─────┘  └──────┬─────┘ │
│                                  IPC (dosya) ◄──────────┘       │
│                                  /root/workspace/control/        │
└─────────────────────────────────────────────────────────────────┘
```

## 🛠️ Teknoloji Yığını

| Kategori | Teknoloji |
|----------|-----------|
| **Donanım** | Raspberry Pi 4, RPLidar S2E, Pixhawk, Pi Camera IMX708, STM32 |
| **İşletim Sistemi** | Raspberry Pi OS (Host) / Ubuntu 22.04 (Docker) |
| **Middleware** | ROS 2 Humble, MAVProxy |
| **Diller** | Python 3.10+, Bash |
| **Web Framework** | Flask (MJPEG streaming, REST API) |
| **Bilgisayar Görüşü** | OpenCV (Renk tespiti, HUD overlay) |
| **Uçuş Kontrolü** | MAVLink via pymavlink (ArduRover) |
| **Veri İşleme** | NumPy (Vektörize Lidar işleme) |

## 📁 Proje Yapısı

```
CELEBILER_USV/
├── host_scripts/
│   ├── system_start.sh          # 🔑 Ana başlatma scripti
│   ├── system_stop.sh           # Güvenli kapatma
│   └── usv_logs.sh              # Log izleme aracı
├── docker_workspace/
│   ├── src/
│   │   ├── usv_main.py          # 🧠 Otonom beyin (State Machine)
│   │   ├── telemetry.py         # 🌐 Dashboard + Telemetri (8080)
│   │   ├── cam.py               # 📷 Kamera işleme + web stream (5000)
│   │   ├── lidar_map.py         # 🗺️ Lidar harita görselleştirme (5001)
│   │   ├── rc_test.py           # RC override test aracı
│   │   └── auto_dry_test.py     # Kara testi scripti
│   ├── scripts/
│   │   └── internal_start.sh    # Docker içi başlatma (mavproxy + servisler)
│   ├── mission.json             # 📍 Görev noktaları (GPS waypoints)
│   └── logs/                    # Telemetri CSV & video logları
├── config/
│   └── usv_mode.cfg             # test | race mod seçimi
├── documents/
│   └── ida_sartname.md          # TEKNOFEST şartname
└── README.md
```

## 🚀 Hızlı Başlangıç

### 1. Repoyu Klonla
```bash
git clone https://github.com/celebiler/CELEBILER_USV.git
cd CELEBILER_USV
```

### 2. Görev Noktalarını Ayarla
`docker_workspace/mission.json` dosyasını düzenle:
```json
{
  "parkur1": [[36.88838, 30.67384], [36.88848, 30.67394]],
  "parkur2": [[36.88818, 30.67394], [36.88808, 30.67404]],
  "parkur3": [[36.88828, 30.67414]],
  "target_color": "RED"
}
```

### 3. Sistemi Başlat (Raspberry Pi üzerinde)
```bash
# Test modu (varsayılan)
./host_scripts/system_start.sh

# Yarışma modu
./host_scripts/system_start.sh race
```

### 4. Dashboard'a Eriş
Tarayıcıdan: `http://<RPi_IP>:8080`

## 🌐 Web Arayüzleri

| Port | Servis | Açıklama | Yarışma Modu |
|------|--------|----------|-------------|
| `8080` | Mission Control Dashboard | Görev kontrolü, telemetri, sensör verileri | ✅ Aktif |
| `5000` | Kamera Stream | 720p MJPEG + renk tespiti overlay | ❌ Kapalı (Şartname 3.7) |
| `5001` | Lidar Haritası | Canlı 2D engel haritası | ❌ Kapalı (Şartname 3.7) |

## 🎮 Dashboard Özellikleri

Tek sayfa Mission Control arayüzü:

- **Mission Control Paneli** — Görev timer, parkur durumu, progress bar, kontrol butonları
- **Kontrol Butonları** — ▶ Görevi Başlat / ⏭ Sonraki Parkur / ⛔ Acil Durdur
- **Kamera + Lidar Feed** — Canlı görüntü (sadece test modunda)
- **Telemetri Kartları** — Batarya, GPS, Hız/Yön, Atmosfer, RC Stickler, Motor PWM
- **Sistem Metrikleri** — CPU, RAM, Sıcaklık

## 🔧 Temel Özellikler

- **Mod Ayrımı:** `test` (tam web yayını + manuel parkur geçişi) / `race` (onboard only, Şartname uyumlu)
- **MAVProxy Port Paylaşımı:** Pixhawk'a `telemetry.py` ve `usv_main.py` aynı anda erişir
- **Dosya Tabanlı IPC:** Dashboard ↔ Otonom beyin arası güvenli iletişim
- **Simülasyon Modu:** Donanım yoksa otomatik sahte veri üretimi
- **Performans Optimizasyonu:** Native CSV, 25Hz motor loop, akıllı log filtreleme
- **STM32 Entegrasyonu:** Çevresel sensörler (Sıcaklık, Nem, Yağmur)
- **RC Kumanda:** Cruise control, soft-start ramping, vites mantığı (CH6 switch)
- **Şartname Uyumu:** 20dk uyarı timer, GPS JSON (float), bounding box formatı

## 📡 Port Haritası

| Port | Protokol | Kullanım |
|------|----------|----------|
| `8080` | HTTP | Telemetri Dashboard |
| `5000` | HTTP/MJPEG | Kamera Web Stream |
| `5001` | HTTP | Lidar Haritası |
| `8888` | TCP | Ham Kamera (Host → Docker) |
| `14550` | UDP/MAVLink | mavproxy → telemetry.py |
| `14551` | UDP/MAVLink | mavproxy → usv_main.py |

## 📜 Lisans

Bu proje MIT Lisansı altında lisanslanmıştır.

## 👥 Takım

**AKDENİZ ÜNİVERSİTESİ — ÇELEBİLER İDA TAKIMI**

---

<div align="center">
Made with ❤️ for TEKNOFEST 2026
</div>
