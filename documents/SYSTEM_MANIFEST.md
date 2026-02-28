# 🚀 ÇELEBİLER USV - SYSTEM MANIFEST (ANTIGRAVITY)

## 1. PROJE KİMLİĞİ VE AMAÇ
* **Proje Adı:** Çelebiler USV (İnsansız Deniz Aracı)
* **Hedef:** TEKNOFEST 2026 İDA Yarışması.
* **Mevcut Aşama:** 🟡 **TEST VE DEBUG AŞAMASI** (Operasyonel değil, Analitik mod).
* **Öncelik:** Tüm verilerin (Görüntü, Lidar, Telemetri) canlı izlenmesi, loglanması ve sistemin çökmeden ayağa kalkması.

## 2. DONANIM MİMARİSİ (HOST)
* **Ana Bilgisayar:** Raspberry Pi 4 Model B.
* **İşletim Sistemi (Host):** Raspberry Pi OS (64-bit).
* **Sensörler:**
    * **Lidar:** RPLidar S2E (UDP Modunda, Statik IP: 192.168.11.2).
    * **Uçuş Kontrolcü:** Pixhawk (Cube/Orange) - MAVLink protokolü üzerinden USB (/dev/ttyACM*).
    * **Kamera:** Raspberry Pi Camera Module (Host üzerinden `rpicam-vid` ile TCP yayını yapar).
    * **Telemetri:** RF Modüller veya 4G Modem (Gelecekte).
* **Ağ Yapısı:**
    * Host IP: `192.168.11.5` (Statik).
    * Docker Ağı: `--net=host` (Host portlarını doğrudan kullanır).

## 3. YAZILIM MİMARİSİ (DOCKER)
* **Konteyner Adı:** `ege_ros`
* **İmaj Tabanı:** Ubuntu 22.04 LTS.
* **Robotik Middleware:** ROS 2 Humble Hawksbill.
* **Dil:** Python 3.10+
* **Web Arayüzü:** Flask tabanlı mikro servisler.
* **Kritik Kütüphaneler:** `rclpy`, `pymavlink`, `opencv-python`, `pandas`, `flask`, `numpy`.

## 4. ÇALIŞMA MODLARI (TEST / YARIŞMA)
* **Test Modu (varsayılan):** Kamera (5000), Lidar harita (5001) ve Telemetri (8080) tam açık. Web üzerinden manuel test ve debug için.
* **Yarışma Modu:** Sadece telemetri (8080). Kamera ve lidar web yayını kapalı (IDA Şartname 3.7 - görüntü aktarımı yasak).
* **Geçiş:** `config/usv_mode.cfg` içinde `USV_MODE=test` veya `USV_MODE=race` yazın. Veya: `./system_start.sh race` (argüman öncelikli).

## 5. DOSYA YAPISI VE GÖREVLERİ
* `~/CELEBILER_USV/` (Ana Dizin)
    * `config/usv_mode.cfg`: **Mod Seçimi.** USV_MODE=test veya race.
    * `host_scripts/system_start.sh`: **Sistemi Başlatan Anahtar.** IP atar, mod okur, kamerayı açar, Docker'ı tetikler.
    * `docker_workspace/scripts/internal_start.sh`: **İç Kaptan.** Moda göre cam/lidar web sunucularını açıp açmaz; telemetri her zaman başlar.
    * `docker_workspace/src/`:
        * `cam.py`: Görüntü işleme + Web Yayın (Port 5000).
        * `lidar_map.py`: Lidar verisini haritalama + Web Yayın (Port 5001).
        * `telemetry.py`: Pixhawk verilerini okuma + Dashboard + CSV Kayıt (Port 8080).
        * `fusion_main.py`: **ANA BEYİN.** Sensör füzyonu ve motor kontrolü.

## 6. VERİ TESLİM (Şartname Bölüm 6)
* **Telemetri CSV:** `docker_workspace/logs/telemetri_verisi.csv` — 1 Hz, zorunlu alanlar: lat, lon, Speed, Roll, Pitch, Heading, Speed_Setpoint, Heading_Setpoint. İDA karaya alındıktan sonra USB ile teslim.

## 7. ANTIGRAVITY ÇALIŞMA PRENSİBİ (DEVOPS)
1.  **Geliştirme (Windows):** Kodlar VS Code üzerinde yazılır.
2.  **Senkronizasyon:** `git push` ile GitHub'a gönderilir.
3.  **Dağıtım (Raspberry Pi):** Terminalde `guncelle` komutu ile çekilir.
4.  **Çalıştırma:** Terminalde `start` komutu ile başlatılır.