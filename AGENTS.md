# CELEBILER_USV Agent Rules

Bu dosya bağlayıcı kural dosyasıdır. Ajan, alt ajan ve otomasyon işleri bu sırayla uymak zorundadır.

## 1. Simülasyon-Öncelikli Geliştirme Stratejisi

Proje **sim-first** stratejisiyle çalışır:
- Tüm geliştirme ve doğrulama önce simülasyonda yapılır.
- Simülasyon gerçek sistemi %100 temsil etmelidir: aynı kodlar, aynı MAVLink protokolleri, aynı state machine.
- Simülasyon sorunsuz çalıştığında aynı kodlar gerçek İDA üzerine deploy edilir.
- Köprü (bridge) mimarisi sim-to-real geçişini sağlar:
  - `sitl_gazebo_bridge.py`: SITL MAVLink <-> Gazebo fizik motoru (motor, pose, sensör)
  - `ros_to_tcp_cam.py`: ROS2 Image topic <-> TCP JPEG stream (`cam.py` aynı arayüzü kullanır)
  - ArduPilot SITL: Gerçek Pixhawk ile aynı firmware, `tcp:5760` üzerinden aynı MAVLink protokolü
- Sim modda farklı olan tek şey: fiziksel donanımlar yazılım köprüleriyle temsil ediliyor.
- Sim-only bypass yalnızca fiziksel donanım yokluğu içindir (ch7 PWM, serial port, USB storage); iş mantığı asla değişmez.
- `USV_SIM=1` ortam değişkeniyle sim/gerçek ayrıştırılır.

## 2. Belge Önceliği
1. `/documents/ida_sartname.md`
2. `/documents/rapor_calismasistemi.md`
3. `host_scripts/` içindeki start/stop/doğrulama akışları
4. Bu dosya

Üst sıradaki belge alt sıradaki kural ile çelişirse üst sıradaki geçerlidir.

## 3. Rol Ayrımı
- `SDA` yalnızca tanı, şartname uyum analizi ve plan üretir.
- `SDA`, `autonomy/control/mission` çekirdek kodunu doğrudan değiştirmez.
- `SDA`, onaylı planını `/memories/session/ida-sim-diagnosis.md` içine yazar.
- `Implementation Agent` yalnızca onaylı ve şartnameye uyumlu planı uygular.
- Kod değişikliği minimum diff ile yapılır.

## 4. Sistem Başlatma Sırası

Bileşenler aşağıdaki sırayla başlatılmalıdır. Sıra bozulursa bağımlılıklar karşılanmaz.

### Simülasyon (`run_sim_stack.sh`)
```
1.  Gazebo              (fizik motoru + dünya)
2.  ROS-GZ Bridge       (topic köprüleri: pose, scan, camera, odom, cmd_vel)
3.  ArduPilot SITL      (otonom pilot firmware, tcp:5760)
4.  sitl_gazebo_bridge   (MAVLink <-> Gazebo pose/motor/JSON)
5.  ros_to_tcp_cam       (ROS kamera <-> TCP:8888)
6.  cam.py              (görüntü işleme, port 5000)
7.  telemetry.py        (dashboard + API, port 8080)
8.  lidar_map.py        (lidar haritalama, port 5001)
9.  usv_main.py         (ana state machine, tcp:5760 -> SITL)
10. check_stack         (proses doğrulama)
11. wait_for_startup_readiness  (health gate: ready_state + camera + lidar)
12. compliance tests    (uyum doğrulama)
```

### Gerçek Donanım (`system_start.sh`)
```
1.  system_start.sh      (preflight: lidar ping, serial, kamera)
2.  Docker container     (ege_ros başlatma)
3.  internal_start.sh    (ROS + workspace setup)
4.  MAVProxy             (Pixhawk bağlantısı, varsa)
5.  rplidar_ros          (lidar driver, UDP)
6.  lidar_map.py         (test modda)
7.  cam.py -> telemetry.py -> usv_main.py
```

## 5. Görev Başlatma Kuralları

- Görev **asla** otomatik başlamaz; operatörün bilinçli komutu zorunludur.
- **Test modunda:** Dashboard "Start" butonu veya `POST /api/start_mission`.
- **Yarış modunda:** Yalnızca RC CH5 >= 1700 PWM ile başlar (API kapalı).
- Başlatma ön koşulları: `ready_state=true`, `camera_ready=true`, `lidar_ready=true`.
- Başladıktan sonra parkurlar arası geçiş (P1 -> P2 -> P3) otomatiktir, operatör müdahalesi gerekmez.
- Başladıktan sonra mission reload / retarget engellenir (post-start lock).
- Durdurma: E-stop (RC CH7 >= 1900 veya API `emergency_stop.flag`) veya CTRL+C/ESC (sim).

## 6. Zorunlu İş Akışı
- Kod değişikliği öncesi en az şu dosyalar okunur:
  - `/documents/ida_sartname.md`
  - `/documents/rapor_calismasistemi.md`
  - ilgili `host_scripts/*`
- Görevle ilgisiz değişiklikler geri alınmaz.
- E-stop, röle, RC override ve mission state şeması zayıflatılamaz.

## 7. Haberleşme ve Donanım Yasakları
- `2.4–2.8 GHz` ve `5.15–5.85 GHz` bandında çalışan yeni haberleşme yolu eklenmez.
- Hücresel modem, hotspot, Wi-Fi tabanlı görev/telemetri/görüntü aktarımı eklenmez.
- Otonomi, görüntü işleme ve sensör işleme gemi dışına taşınmaz.
- YKİ yalnızca arayüz, mission yükleme ve telemetri amaçlı kullanılır.
- Yarış modunda görüntü aktarımı açılmaz.

## 8. Kontrol ve Emniyet Kuralları
- Manuel/RC override, otonomiden daima önceliklidir.
- Görev başladıktan sonra acil durdurma dışında yeni operatör komutu kabul edilmez.
- E-stop zinciri fiziksel güç kesme mantığını zayıflatacak şekilde değiştirilemez.
- Fail-safe davranışları `HOLD`, hız düşürme ve güvenli pasif bekleme mantığıyla uyumlu kalır.
- Parkurlar arası geçiş kullanıcı girdisi olmadan otomatik olmalıdır.

## 9. Mod Kuralları
- Geliştirme ve dashboard doğrulaması `test` modunda yapılır.
- Yarış mantığı `race` modunda onboard odaklı ve görüntü aktarımı kapalı çalışır.
- Simülasyon işleri `USV_SIM=1` ile ayrıştırılır.

## 10. Zorunlu Loglama Standardı
- Simülasyonda bütün çekirdek servisler merkezi log init kullanır.
- Simülasyonda fonksiyon trace varsayılan olarak açıktır.
- Her servis en az şu dosyaları üretir:
  - `{component}.debug.log`
  - `{component}.jsonl`
- Fonksiyon trace en az şu olayları kapsar:
  - giriş, çıkış, hata, süre bilgisi
- Şu olaylar `INFO/WARN/ERROR` seviyesinde açıkça loglanır:
  - mode değişimi, mission load/start/stop
  - RC override, E-stop
  - GPS fix kaybı, lidar timeout, kamera timeout
  - failsafe geçişi, waypoint geçişi
- Loglar yalnızca yerel dosya sisteminde tutulur; şartnameye aykırı görüntü aktarımı için kullanılamaz.
- Log rotasyonu kapatılamaz.
- Simülasyonda üretilen yeni log dosyaları dashboard log görüntüleyicisinde görünür olmalıdır.

## 11. Veri ve Şema Kuralları
- Servisler arası mission/state paylaşımı mevcut dosya şemaları bozulmadan sürdürülür.
- `docker_workspace/mission.json`, `sim/control/*.json` ve benzeri ortak durum dosyalarında geriye dönük uyum korunur.
- Geçersiz sensör verisi doğrudan görev hattına verilmez; önce guard/filter uygulanır.

## 12. Dizin Sorumlulukları
- `docker_workspace/src/`: çalışma zamanı servisleri ve kontrol mantığı
- `sim/bin/`: simülasyon orkestrasyonu
- `sim/bridges/`: simülasyon haberleşme köprüleri (sim-to-real katmanı)
- `sim/control/`: simülasyon kontrol ve paylaşılan durum dosyaları
- `host_scripts/`: host başlatma, durdurma ve doğrulama

## 13. Doğrulama Zorunluluğu
- Her değişiklik sonrası en az aşağıdakiler çalıştırılır:
```bash
python3 -m py_compile docker_workspace/src/*.py
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
./sim/bin/run_sim_stack.sh
```
- Simülasyon çalıştırılamıyorsa neden açıkça not edilir.

## 14. Yarış Seviyesi Uyum Sertifikasyonu

### Zorunlu Ön-Yarış Doğrulaması
Yarış dağıtımından önce aşağıdakileri çalıştırın:
```bash
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
python3 host_scripts/check_compliance_race.py
./sim/bin/run_sim_stack.sh mode=race
```

### 8 Kabul Kriteri
1. **Mission Şeması**: Değişken uzunlukta parkurlar desteklenir, sabit indisler yok
2. **Görüntü Aktarımı**: Yarış moduyla hardening aktif, dışarı kaynakta stream yok
3. **P1 Yarış Saflığı**: Saf Pixhawk AUTO, Pi fallback veya override yok
4. **Post-Start Kilit**: Görev başlar sonrası misyon yeniden yükleme/retarget engellenir
5. **Mission Lifecycle**: Upload kaynağı ve doğrulama timestamp takip edilir
6. **Guidance Source**: Telemetri mod belirtici açıkça tanımlanır (`p1_pixhawk_auto`, `p1_pi_guided`, vb.)
7. **Adapter Uyumluluğu**: Mission adaptörü hem düz dizi hem de yapılandırılmış format işler
8. **Profile Enforcement**: Uyum kısıtlamaları kodda uygulanır, heuristic kontrol değil

### Hata Yönetimi
Herhangi bir kriter başarısız olursa:
- Uyum testleri ayrıntılı hata logu oluşturur
- Sistem yarış modunda arm edilemez
- CI/CD için sıfırdan farklı çıkış kodu döner

### Simülasyon Stack Entegrasyonu
Uyum testleri `./sim/bin/run_sim_stack.sh mode=race` içine entegre edilmiştir:
- Değişken uzunlukta misyon yüklenir
- Dış görüntü stream dinleyici yok doğrulanır
- P1 AUTO modunda doğrulanır
- Post-start komut kilidi doğrulanır
- Artefakt üretir: `logs/simulation/compliance_race_test.jsonl`

### Sertifikasyon Başarı Kriteri
- Tüm 3 uyum betiği geçer: `static`, `behavior`, `race`
- Simülasyon başarıyla başlayıp kapanır
- `logs/simulation/compliance_race_test.jsonl` hata içermez
- `logs/terminal.log` hata seviyesi uyarı göstermez
