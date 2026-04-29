# CELEBILER USV - TEKNOFEST 2026 IDA

TEKNOFEST 2026 Insansiz Deniz Araci yarismasi icin gelistirilen sim-first otonomi, telemetri ve sim-to-real altyapisidir. Sistem once simulasyonda ayni MAVLink/state-machine akisi ile dogrulanir; ayni runtime kodlari daha sonra gercek IDA uzerinde calisir.

## Sistem Ozeti

- Gelistirme stratejisi: Sim-first. `USV_SIM=1` yalniz fiziksel donanim yoklugunu ayirir; is mantigi sim ve gercekte ortaktir.
- Gorev modeli: Degisken uzunlukta mission girisi, otomatik waypoint/parkur ilerlemesi, post-start mission reload/retarget kilidi.
- Start politikasi: Gorev otomatik baslamaz. Test modunda dashboard/API start; race modunda yalniz RC CH5 `>= 1700`.
- Stop politikasi: E-stop RC CH7/API flag ve yazilim neutral/disarm; fiziksel guc kesme zinciri zayiflatilmaz.
- Goruntu politikasi: Race modunda yer tarafina goruntu veya lidar harita yayini yoktur. Kamera onboard isleme icin calisir.
- Motor kontrati: `1500` neutral, `1100-1500` reverse, `1500-1900` forward.
- Telemetri CSV header:
  `timestamp,lat,lon,ground_speed,roll,pitch,heading,speed_setpoint,heading_setpoint,left_pwm,right_pwm,mode,waypoint_index,obstacle_state`

## Ana Bilesenler

### Runtime

| Dosya | Sorumluluk |
|---|---|
| `docker_workspace/src/usv_main.py` | Ana state machine, mission lifecycle, safety gate, MAVLink komutlari |
| `docker_workspace/src/telemetry.py` | Dashboard, REST API, mission upload, start/estop IPC, CSV log |
| `docker_workspace/src/cam.py` | Onboard kamera isleme; test modda web stream, race modda stream kapali |
| `docker_workspace/src/lidar_map.py` | Test modu lidar haritasi; race modda baslamaz |
| `docker_workspace/src/compliance_profile.py` | Mod, esik, PWM, failsafe, frekans ve policy sabitleri |
| `docker_workspace/src/mission_config.py` | Mission split/profil ve hedef renk state okuma |
| `docker_workspace/src/mission_adapter.py` | Flat/structured mission adapter uyumlulugu |
| `docker_workspace/src/nav_guidance.py` | Waypoint guidance karar shimleri ve motor allocation wrapper |
| `docker_workspace/src/navigation.py` | Saf waypoint distance/bearing ve heading-first request helper |
| `docker_workspace/src/obstacle_avoidance.py` | Uc sektorlu lidar avoidance helper |
| `docker_workspace/src/motor_controller.py` | Ortak PWM clamp, mixer, trim, deadband, ramp ve jerk limiter |
| `docker_workspace/src/sensors.py` | Ortak sensor snapshot ve freshness kontratlari |
| `docker_workspace/src/adapters/` | Sim/real adapter sinir siniflari |

### Baslatma ve Simulasyon

| Yol | Sorumluluk |
|---|---|
| `host_scripts/system_start.sh` | Gercek donanim preflight, Docker ve servis baslatma |
| `host_scripts/system_stop.sh` | Host kamera, portlar ve Docker kapatma |
| `docker_workspace/scripts/internal_start.sh` | Docker icinde MAVProxy, lidar, cam, telemetry, usv_main baslatma |
| `sim/bin/run_sim_stack.sh` | Gazebo, ROS-GZ bridge, SITL, sim bridge, cam, telemetry, lidar_map, usv_main orkestrasyonu |
| `sim/bridges/sitl_gazebo_bridge.py` | SITL MAVLink ile Gazebo pose/motor/JSON koprusu |
| `sim/bridges/ros_to_tcp_cam.py` | ROS2 Image topic -> TCP JPEG kamera donanim arayuzu |

## Baslatma Sirasi

### Simulasyon

`./sim/bin/run_sim_stack.sh` su sirayi uygular:

1. Gazebo
2. ROS-GZ Bridge
3. ArduPilot SITL
4. `sitl_gazebo_bridge.py`
5. `ros_to_tcp_cam.py`
6. `cam.py`
7. `telemetry.py`
8. `lidar_map.py`
9. `usv_main.py`
10. `check_stack`
11. startup readiness kontrolu
12. compliance kontrolleri

Gorev otomatik baslamaz. Test modunda operator dashboard Start butonunu veya `POST /api/start_mission` endpointini kullanir.

### Gercek Donanim

```bash
./host_scripts/system_start.sh test
./host_scripts/system_start.sh race
```

Gercek baslatma akisi:

1. Lidar IP/preflight
2. Pixhawk + STM32 seri port kontrolu
3. Kamera kontrolu
4. Host kamera yayini: test modda acik, race modda kapali
5. Docker `ege_ros`
6. `internal_start.sh`
7. MAVProxy
8. ROS lidar driver
9. `cam.py`, `telemetry.py`, `usv_main.py`

Race modda `WIFI_DISABLE=true` ayari `config/usv_mode.cfg` icinde verilebilir.

## Mod Kurallari

| Konu | Test Modu | Race Modu |
|---|---|---|
| Mission start | Dashboard veya `/api/start_mission` | Yalniz RC CH5 `>= 1700` |
| `/api/start_mission` | Hazirlik gate gecerse flag yazar | `403` doner |
| Kamera web stream | Acik | Kapali, onboard isleme aktif |
| Lidar map web | Acik | Baslamaz / endpoint `403` |
| Mission upload | Start oncesi | Start oncesi |
| Start sonrasi komut | E-stop haric reddedilir | E-stop haric reddedilir |

Hazirlik gate kosullari: `ready_state=true`, `camera_ready=true`, `lidar_ready=true`.

## Otonomi Akisi

Aktif runtime state machine:

```text
IDLE -> NAV -> ENGAGE -> COMPLETED/HOLD
```

Minimal hedef state modeli su sekilde temsil edilir:

```text
IDLE -> LOAD_MISSION -> ARM_READY -> TURN_TO_WAYPOINT
-> GO_TO_WAYPOINT -> AVOID_OBSTACLE -> WAYPOINT_REACHED
-> NEXT_WAYPOINT -> MISSION_DONE -> FAILSAFE
```

Uygulamada NAV fazi waypoint listesini sirayla yurutur. Heading-first davranis `navigation.py` ve `nav_guidance.py` icinde uygulanir:

- Heading hatasi buyukse forward `0`, yaw duzeltme onceliklidir.
- Heading esik icine girince ileri hiz aktif olur.
- Waypoint yaklasiminda hiz dusurulur.
- Kabul yaricapi icinde motor neutral olur.

Obstacle avoidance:

- Lidar ana guvenlik sensorudur.
- Sektorler: left/front/right.
- `D_MIN_M=1.2`, `P2_LIDAR_WARN_M=1.5`.
- Front blocked ise hiz `0` veya `FAILSAFE_SLOW_MPS`, yaw daha acik sektore verilir.
- Kamera yalniz yardimci sinyal uretir; lidar yokken carpismasiz guvenlik varsaymaz.

## Motor Kontrol

Ortak motor profili `motor_controller.py` icindedir:

- Neutral: `PWM_NEUTRAL_US=1500`
- Clamp: `PWM_MIN_US=1100`, `PWM_MAX_US=1900`
- Deadband ve minimum efektif PWM tek noktadan uygulanir.
- Ramp limiter: `PWM_SLEW_RATE_US_PER_S`
- Jerk limiter: `PWM_JERK_LIMIT_US_PER_S2`
- Failsafe, E-stop, mission inactive, command lock durumlarinda cikis `1500/1500`.

Sim bridge `sim/control/motor_command.json` icindeki son `left/right` PWM kontratini okur; gercek donanimda MAVLink RC override/passthrough yolu kullanilir.

## API Ozeti

| Endpoint | Yontem | Mod | Aciklama |
|---|---|---|---|
| `/` | GET | test/race | Dashboard |
| `/api/data` | GET | test/race | Telemetri, health, mission state, report_view |
| `/api/mission_status` | GET | test/race | Gorev aktiflik/ilerleme ozeti |
| `/api/events` | GET | test/race | Kritik olay long-poll |
| `/api/start_mission` | POST | test | Start flag yazar; hazir degilse `409` |
| `/api/start_mission` | POST | race | Politika geregi `403` |
| `/api/emergency_stop` | POST | test/race | E-stop flag ve neutral/disarm |
| `/api/mission` | POST | test/race | Start oncesi mission upload; start sonrasi `409` |
| `/api/target_color` | POST | test/race | Start oncesi hedef renk; start sonrasi `409` |
| `/api/camera_stream` | GET | test | Kamera proxy |
| `/api/camera_stream` | GET | race | `403` |
| `/api/lidar_stream` / `/api/lidar_map` / `/api/spatial_map` | GET | test | Harita/gorsel debug |
| `/api/lidar_stream` / `/api/lidar_map` / `/api/spatial_map` | GET | race | `403` |
| `/api/log_files`, `/api/log_tail`, `/api/log_file` | GET | test/race | Yerel log goruntuleme |

## Loglar

Tum cekirdek servisler merkezi log init kullanir ve yerel dosya sistemine yazar.

| Konum | Icerik |
|---|---|
| `logs/terminal.log` | `run_sim_stack.sh` ana tee cikisi |
| `logs/system/` | `cam`, `telemetry`, `usv_main`, `lidar_map`, CSV ve video artefaktlari |
| `logs/simulation/` | Gazebo, SITL, ROS-GZ, bridge ve compliance race loglari |
| `logs/host/` | Host start/trace loglari |
| `logs/system/compliance/` | Static/behavior compliance raporlari |

Her servis en az su formatlari hedefler:

- `{component}.debug.log`
- `{component}.jsonl`

Fonksiyon trace simde varsayilan aciktir. Log rotasyonu `runtime_debug_log.py` tarafindan yapilir.

## Dogrulama

Her kod veya dokumantasyon degisikliginden sonra en az:

```bash
python3 -m py_compile docker_workspace/src/*.py docker_workspace/src/adapters/*.py
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
python3 host_scripts/check_compliance_race.py
```

Bu ajan/otomasyon kendi kendine `./sim/bin/run_sim_stack.sh --auto-test` calistirmaz ve otomatik gorev baslatan sim turevlerini kullanmaz. Surus, motion ve mission progression testi operator tarafindan manuel yapilir.

Manuel sim testi:

```bash
./sim/bin/run_sim_stack.sh
```

Dashboard hazir oldugunda test modda Start verilir. Beklenen ana gostergeler:

- `ready_state=true`
- `camera_ready=true`
- `lidar_ready=true`
- CSV minimal header ile yazilir
- `active_waypoint_index` artar
- Motor PWM ani sicrama yapmadan `1100-1900` icinde kalir
- Race modda stream endpointleri kapali kalir

## Port Haritasi

| Port | Protokol | Kullanim |
|---|---|---|
| `8080` | HTTP | Dashboard + API |
| `5000` | HTTP/MJPEG | Kamera web stream, test modu |
| `5001` | HTTP | Lidar haritasi, test modu |
| `8888` | TCP | Ham kamera kaynagi / sim TCP JPEG |
| `14550` | UDP/MAVLink | MAVProxy -> telemetry |
| `14551` | UDP/MAVLink | MAVProxy -> usv_main; sim bridge tarafinda tek MAVLink kopru |
| `5760` | TCP/MAVLink | ArduPilot SITL |
| `20108` | UDP | RPLidar S2E |

## Dizin Yapisi

```text
CELEBILER_USV/
├── documents/
│   ├── ida_sartname.md
│   └── rapor_calismasistemi.md
├── host_scripts/
│   ├── system_start.sh
│   ├── system_stop.sh
│   ├── check_compliance_static.py
│   ├── check_compliance_behavior.py
│   └── check_compliance_race.py
├── docker_workspace/
│   ├── src/
│   │   ├── usv_main.py
│   │   ├── telemetry.py
│   │   ├── cam.py
│   │   ├── lidar_map.py
│   │   ├── motor_controller.py
│   │   ├── navigation.py
│   │   ├── obstacle_avoidance.py
│   │   ├── sensors.py
│   │   └── adapters/
│   ├── scripts/internal_start.sh
│   └── mission.json
├── sim/
│   ├── bin/run_sim_stack.sh
│   ├── bridges/
│   ├── configs/
│   └── control/
├── logs/
│   ├── system/
│   ├── simulation/
│   └── host/
├── AGENTS.md
└── README.md
```

## Yarismaya Uyum Notlari

- 2.4-2.8 GHz ve 5.15-5.85 GHz bandinda yeni haberlesme yolu eklenmez.
- Hucresel modem, hotspot veya Wi-Fi tabanli gorev/telemetri/goruntu aktarimi eklenmez.
- Otonomi, goruntu isleme ve sensor isleme onboard kalir.
- YKI yalniz arayuz, mission yukleme ve telemetri icindir.
- Race modda goruntu aktarimi ve lidar map yayini kapali kalir.
- Gorev basladiktan sonra E-stop disinda yeni operator komutu kabul edilmez.

## Lisans ve Takim

MIT lisansi.

Akdeniz Universitesi - Celebiler IDA Takimi.
