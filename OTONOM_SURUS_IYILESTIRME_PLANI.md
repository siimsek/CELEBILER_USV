# En İyi Otonom Sürüş İyileştirme Planı

Bu plan repo ana klasörüne şu dosya olarak kaydedilmelidir:
`OTONOM_SURUS_IYILESTIRME_PLANI.md`

> **Plan modu uyarısı:** Bu belge başlangıçta yalnızca plan içeriyordu. Şimdi **uygulama modunda**; her adım tamamlandığında "## 0. Uygulama İlerlemesi" bölümü güncellenir. Tüm öneriler `documents/ida_sartname.md` ve `documents/rapor_calismasistemi.md` ile ilişkilidir. Varsayım yapılmamış, eksik bilgiler risk olarak işaretlenmiştir.

---

## 0. Uygulama İlerlemesi (Implementation Tracker)

> **Durum:** Plan uygulanıyor. Her adım tamamlandığında bu bölüm güncellenir. Kesinti olursa buradan devam edilir.
> **Kurallar:** Her değişiklik sonrası `py_compile` + 3 compliance betiği çalıştırılır. Motion/auto-test **asla** ajan tarafından çalıştırılmaz (AGENTS.md §13).

| Adım | Açıklama | Durum | Tarih |
|---|---|---|---|
| 1 | P0 idle-yaw teşhisi (sim bridge + usv_main izleme/log; param doğrulama) | ✅ Tamamlandı | 2026-07-01 |
| 2 | Karar otoritesi merkezileştirme (A/B) | ✅ Tamamlandı | 2026-07-01 |
| 3 | Güven skoru + fail-safe ağacı | ✅ Tamamlandı | 2026-07-01 |
| 4 | Costmap/wrong-target | ✅ Tamamlandı | 2026-07-01 |
| 5 | Smoothing birleştirme | ✅ Tamamlandı | 2026-07-01 |
| 6 | P3 TS3 şeffaflık | ✅ Tamamlandı | 2026-07-01 |
| 7 | Terminal okunabilirlik | ✅ Tamamlandı | 2026-07-01 |
| 8 | usv_main parçalanma (P2) | ⏳ Bekliyor | - |
| 9 | Sertifikasyon | ⏳ Bekliyor | - |

### Adım 1 Detay (P0 idle-yaw teşhisi) — ✅ Tamamlandı
- [x] `sitl_gazebo_bridge.py`: `publish_forces` içine zero-thrust yaw drift tanı logu (`idle_yaw_drift` jsonl + WARN) — `SIM_GZ_IDLE_YAW_DRIFT_DPS` env eşiği
- [x] `usv_main.py`: `_check_idle_yaw_drift` metodu + ana idle döngüsünde çağrı (`v_target=0` iken `observed_yaw_rate` izleme, `USV_IDLE_YAW_DRIFT_DPS` env eşiği)
- [x] `compare_pixhawk_param_baseline.py` çalıştır + SERVO trim risk kontrolü — EXIT=0; strict_blockers yalnız bilinen ARMING/FS riskleri
- [x] Statik testler: py_compile OK, static 92/92, behavior 30/30, race 8/8 — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 1 Bulguları (teşhis):**
- `config/pixhawk_ida.param` SERVO trim **zaten nötr**: `SERVO1_TRIM=1500, SERVO3_TRIM=1500, SERVO1/3_REVERSED=0, MOT_PWM_TYPE=0, FRAME_CLASS=2 (rover)`. Yani idle-yaw **param-file trim'den değil**.
- Kök neden: ArduPilot SITL Rover **GUIDED velocity+yaw** modu v=0'da heading-hold diferansiyel itki üretiyor (log kanıtı: `motor_left_norm=-0.2975, motor_right_norm=0.2975` yani avg_thrust≈0 ama yaw_cmd≠0).
- Teşhis logları (`idle_yaw_drift` jsonl + `[IDLE-YAW-DRIFT]` WARN) yerinde; operatör sim çalıştırınca teyit edilecek.
- **Düzeltme adayı (erteli, riskli):** GUIDED type mask / v=0'da yaw_rate setpoint kullanımı — gerçek araçta beklenmeyen hareket riski → yalnız sim'de ve log kanıtlıysa değerlendirilir (Adım 5 sonrası veya ayrı sertifika adımı).
- **Operatör görevi (manuel):** sim idle iken aracın dönmediği gözlemi; `logs/simulation/sitl_gazebo_bridge.jsonl` ve `logs/system/usv_main.jsonl`'de `idle_yaw_drift` olayı kontrolü.

### Adım 2 Detay (Karar otoritesi merkezileştirme A/B) — ✅ Tamamlandı
- [x] `compliance_profile.py`: 3 yeni tunable — `PLANNER_CORRIDOR_HYSTERESIS_BONUS` (env `USV_PLANNER_CORRIDOR_HYSTERESIS_BONUS`, default 0.8), `PLANNER_GATE_CENTER_BONUS` (env `USV_PLANNER_GATE_CENTER_BONUS`, default 0.6), `PLANNER_GATE_AUTHORITY` (env flag `USV_PLANNER_GATE_AUTHORITY`, default **off**)
- [x] `local_planner.py`: koridor hysteresis bonus artık env-tunable (hardcoded 0.8 → `PLANNER_CORRIDOR_HYSTERESIS_BONUS`); gate-center sub-goal güçlendirme (gate_conf≥0.60 + center_clear + çarpışmasız + bearing yakın → ek `PLANNER_GATE_CENTER_BONUS`)
- [x] `usv_main.py`: `PLANNER_GATE_AUTHORITY` import + P2 nav döngüsünde A/B — flag açık + `advanced_control_allowed` iken `FOLLOW_GATE` heading'i planner otoriter; kapalıyken mevcut davranış (yalnız hız ayarı) korunur
- [x] Statik testler: py_compile OK, static 92/92, behavior 30/30, race 8/8 — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 2 Bulguları:**
- A/B flag `USV_PLANNER_GATE_AUTHORITY` varsayılan **kapalı** → mevcut `compute_nav_decision` davranışı korunur (geriye dönük uyumlu). Operatör sim'de açıp gate merkezlenmesini test edebilir.
- Gate-center bonus yalnızca çarpışmasız + merkez sektör açık adalar için → güvenli koridoru bozmaz.
- `nav_guidance.py` `compute_nav_decision` henüz nominal-girdiye indirgenmedi (planın tam "indirgenme" adımı); mevcut fallback olarak korunuyor. Tam indirgenme Adım 8 (usv_main parçalanma) ile değerlendirilecek.

### Adım 3 Detay (Güven skoru + fail-safe ağacı) — ✅ Tamamlandı
- [x] `sensor_fusion.py`: `FusionState.confidence_breakdown` alanı eklendi (lidar/camera/gps yaş bazlı 0-1 + sensor_consistency + fresh_count); `get_fusion_state`'te per-sensor güven hesabı
- [x] `sensor_fusion.py`: merkezî eşiklerle degrade/HOLD ağacı eklendi (mevcut staleness/ttc ağacını **tamamlayıcı**, yalnızca ekler — `low_confidence_hold` / `low_confidence_slow`)
- [x] `compliance_profile.py`: `FUSION_CONF_SLOW_THRESHOLD` (env `USV_FUSION_CONF_SLOW`, default 0.5) + `FUSION_CONF_HOLD_THRESHOLD` (env `USV_FUSION_CONF_HOLD`, default 0.2)
- [x] `usv_main.py`: fusion snapshot'a `confidence_breakdown` eklendi (telemetri/izlenebilirlik)
- [x] Statik testler: py_compile OK, static 92/92, behavior 30/30, race 8/8, autonomy unit OK — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 3 Bulguları:**
- `fusion_confidence` agrega korundu (0.5/0.3/0.2) → planner ve `check_advanced_autonomy_unit` geriye dönük uyumlu.
- HOLD eşiği 0.2 konservatif: yalnızca toplam sensör kaybı (confidence < 0.2) anında HOLD → mevcut 2x-age bekleme ağacını tamamlayıcı, değiştirmez.
- SLOW eşiği 0.5: kamera+yok/lidar+yok durumlarında degrade → mevcut staleness degrade'iyle örtüşür, çakışmaz.
- Breakdown telemetri/JSON üzerinden izlenebilir; operatör `autonomy_confidence.json` + fusion runtime'da per-sensor güveni görebilir.

### Adım 4 Detay (Costmap/wrong-target) — ✅ Tamamlandı
- [x] `compliance_profile.py`: costmap decay-tazelik tunable'ları eklendi — `COSTMAP_STALE_DECAY_TIMEOUT_S`, `COSTMAP_STALE_DECAY_BOOST`, `COSTMAP_WRONG_TARGET_COST`
- [x] `local_costmap.py`: camera wrong-target girdileri yüksek maliyetli engel olarak costmap'e işleniyor (`COSTMAP_WRONG_TARGET_COST` + inflation)
- [x] `local_costmap.py`: LIDAR/camera güncellemesi bayatladığında decay hızlanıyor (`decay_factor ** boost`; eski engeller daha hızlı temizlenir)
- [x] `local_planner.py`: P3 wrong-target riski planner maliyetine giriyor; wrong-target bearing'e yakın adaylar cezalandırılıyor ve `WRONG_TARGET_AVOID` davranışı üretiliyor
- [x] `host_scripts/check_advanced_autonomy_unit.py`: wrong-target high-cost ve stale-decay regresyon testleri eklendi
- [x] Statik testler: py_compile OK, static 92/92, behavior 30/30, race 8/8, autonomy unit OK — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 4 Bulguları:**
- Wrong-target artık yalnız P3 latch bloklayıcı değil; local costmap ve predictive planner tarafından da kaçınılacak yüksek maliyetli nesne olarak görülüyor.
- Decay-tazelik düzeltildi: önceki `decay_factor * boost` yaklaşımı bayat sensörde maliyeti artırma riski taşıyordu; yeni hesap `decay_factor ** boost` ile 0-1 aralığında kalıp daha hızlı temizleme sağlar.
- Sim/motion doğrulaması çalıştırılmadı; AGENTS.md §13 gereği yalnız statik ve saf Python doğrulama yapıldı.

### Adım 5 Detay (Smoothing birleştirme) — ✅ Tamamlandı
- [x] `compliance_profile.py`: heading input filtresi merkezi tunable yapıldı — `HEADING_INPUT_FILTER_ENABLED` (`USV_HEADING_INPUT_FILTER`, default kapalı) ve `HEADING_INPUT_FILTER_ALPHA` (`USV_HEADING_INPUT_FILTER_ALPHA`, default 1.0)
- [x] `usv_main.py`: MAVLink `GLOBAL_POSITION_INT`, MAVLink `ATTITUDE`, `sim_nav_state` ve Gazebo `vehicle_position.json` heading güncellemeleri tek `_set_current_heading(...)` helper'ına bağlandı
- [x] Varsayılan runtime katman sayısı düşürüldü: heading input EMA kapalı/pass-through; planner smoothing + control smoothing kalır
- [x] `control_smoothing.jsonl`: heading input filtresi açık/kapalı, alpha ve kaynak bilgisi log alanlarına eklendi
- [x] Offline doğrulama: `analyze_heading_oscillation.py` çalıştı — sample=1501, sign_flip_rate=0.0127, yaw_rate_overshoot_count=1
- [x] Statik testler: py_compile OK, static 92/92, behavior 30/30, race 8/8, autonomy unit OK — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 5 Bulguları:**
- Eski `_heading_ema` katmanı kaldırıldı; tekrar gerekirse `USV_HEADING_INPUT_FILTER=1` ile ölçüm filtresi env üzerinden açılabilir.
- Sim heading artık Gazebo/`sim_nav_state` kaynağından doğrudan alınır; wrap güvenliği helper içinde korunur.
- Sim/motion doğrulaması çalıştırılmadı; operatör manuel sürüşte WP dönüşleri ve `analyze_heading_oscillation.py` çıktısını tekrar kontrol etmelidir.

### Adım 6 Detay (P3 TS3 şeffaflık) — ✅ Tamamlandı
- [x] `usv_main.py`: P3 wrong-target temas riski için geçiş bazlı sayaç eklendi — `p3_wrong_target_contact_count` ve geriye uyumlu `p3_wrong_target_contact_risk_count`
- [x] `usv_main.py`: `P3_TS3_WRONG_TARGET_RISK_ENTER/EXIT` JSONL olayları eklendi; sürekli aktif riskte spam yok, yalnız risk geçişleri loglanır
- [x] `mission_state.json`: `p3_ts3`, `p3_wrong_target_contact_count`, `p3_wrong_target_contact_risk_active`, `p3_wrong_target_last_event` alanları geriye dönük uyumlu şekilde eklendi
- [x] `p3_target_status`: anlık wrong-target risk, alan, bearing, sınıf ve `p3_contact_latch_blocked` alanlarıyla şeffaflaştırıldı
- [x] `check_compliance_behavior.py`: TS3 sayacı/state export görünürlük kontrolü eklendi
- [x] Statik testler: py_compile OK, static 92/92, behavior 31/31, race 8/8, autonomy unit OK — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 6 Bulguları:**
- Wrong-target temas riski aktifken P3 contact latch hâlâ `_p3_contact_evidence` içinde bloklanıyor; bu adım davranışı zayıflatmadan yalnız görünürlüğü artırdı.
- Sayaç her P3 angajman başlangıcında sıfırlanır; aynı risk sürekli aktif kaldığında bir kez sayılır, risk kapanıp tekrar açılırsa yeni geçiş sayılır.
- Sim/motion doğrulaması çalıştırılmadı; AGENTS.md §13 gereği yalnız statik ve saf Python doğrulama yapıldı.

### Adım 7 Detay (Terminal okunabilirlik) — ✅ Tamamlandı
- [x] `console_utils.py`: stdout/stderr `line_buffering=True`, `write_through=True`; default `print` flush açık
- [x] `run_sim_stack.sh`: `run_line_buffered` helper eklendi (`stdbuf -oL -eL` varsa kullanılır, yoksa normal komut)
- [x] `run_sim_stack.sh`: Gazebo, ROS-GZ, SITL, bridge ve Python servis başlangıçları line-buffered helper üzerinden çalışır; raw log redirect dosyaları korunur
- [x] `run_sim_stack.sh`: shutdown sırasında kayıtlı PID’ler `kill_if_running` ile beklenerek kapatılır; ham bash `Killed ...` satırı gürültüsü azaltılır
- [x] `check_compliance_static.py`: terminal line-buffering ve shutdown okunabilirliği görünürlük kontrolü eklendi
- [x] Statik testler: bash -n OK, py_compile OK, static 93/93, behavior 31/31, race 8/8, autonomy unit OK — hepsi EXIT=0
- [x] Plan dosyası güncellendi

**Adım 7 Bulguları:**
- Dosya logları hâlâ servis bazlı raw dosyalara yazılır (`gazebo.log`, `sitl.log`, `cam.log`, `telemetry.log`, `lidar_map.log`, `usv_main.log`); terminal okunabilirliği için log şeması değiştirilmedi.
- Renk davranışı mevcut `NO_COLOR`/CI korumasıyla uyumlu kaldı.
- Sim/motion doğrulaması çalıştırılmadı; AGENTS.md §13 gereği yalnız statik ve saf Python doğrulama yapıldı.

---

## 1. Mevcut Durum Analizi

### 1.1 Genel Mimari
Sistem **sim-first** stratejisiyle tutarlı bir biçimde kurulmuştur. Sim ve gerçek sistem aynı `usv_main.py` durum makinesini, aynı MAVLink protokolünü ve aynı karar modüllerini kullanır. Fark yalnızca I/O katmanındadır (SITL vs Pixhawk, Gazebo sensörleri vs gerçek sensörler).

Bileşen haritası (tümü `docker_workspace/src/` altında):

| Katman | Dosya | Sorumluluk |
|---|---|---|
| Durum makinesi | `usv_main.py` (11 528 satır / 561 KB) | Mission lifecycle, safety gates, MAVLink I/O, P1/P2/P3 dispatch, GUIDED setpoint gönderimi |
| Karar (nav) | `nav_guidance.py` | `compute_nav_decision`: waypoint + gate assist + lidar blend |
| Karar (planner) | `local_planner.py` | `PredictiveLocalPlanner`: candidate heading skorlama, çarpışma kontrolü, hysteresis, stuck detection |
| Engel kaçınma | `obstacle_avoidance.py` | 3-sektör kaçınma + TTC hız profili |
| Lokal harita | `local_costmap.py` | Rolling occupancy grid, inflation, decay, sektör metrikleri, cluster |
| Sensör füzyonu | `sensor_fusion.py` | LIDAR/kamera/GPS yaş + EKF + TTC + degraded/HOLD tavsiyesi |
| Dinamik engel | `trajectory_tracker.py` | Nearest-neighbor tracker + TTC sınıflandırma |
| Kalman | `kalman_filter.py` | 6D EKF (position/heading) |
| Navigasyon | `navigation.py` | heading-first waypoint request, L1 adaptive, NE velocity dönüşümü |
| Motor | `motor_controller.py` | Twin-thruster mix, slew/jerk limit, trim öğrenme |
| P3 hedef | `target_memory.py` | Vision dropout memory + arama spiral |
| P3 tahmin | `predicted_goals.py` | Trajectory + subgoal JSONL loglayıcı |
| Frame | `spatial_frame.py` | ENU/Gazebo/NED dönüşümleri, lidar local/world |
| Mission | `mission_config.py`, `mission_adapter.py` | Profil sözleşmesi, flat/structured adaptör |
| Profil | `compliance_profile.py` | Tüm threshold/parametre merkezi |
| Sim köprüleri | `sim/bridges/sitl_gazebo_bridge.py`, `ros_to_tcp_cam.py` | SITL MAVLink <-> Gazebo fizik, ROS image <-> TCP |

### 1.2 Durum Makinesi ve P1/P2/P3 Geçişi
- `USVStateMachine`: `STATE_IDLE(0) -> STATE_NAV(1) -> STATE_ENGAGE(2) -> STATE_COMPLETED(4)`, `STATE_HOLD(5)`.
- `run()` (satır 11346): `STATE_NAV -> run_nav() -> STATE_ENGAGE -> run_engage() -> STATE_COMPLETED`.
- **P1 (race):** `run_nav()` -> `_run_nav_auto_mission()` — saf Pixhawk AUTO, Pi pasif monitor (`guidance_source="nav_auto_monitor"`, `motor_limit_reason="pixhawk_auto"`). P1 bitiş kanıtı: `USV_RACE_P1_AUTO_WAYPOINTS` sayısı veya P2-entry evidence.
- **P2:** `run_nav()` -> test/race P2 `nav_waypoints[p2_start_idx:]` -> `_navigate_p2_waypoint()` -> `compute_nav_decision()` + opsiyonel `PredictiveLocalPlanner`.
- **P3:** `run_engage()` -> `_run_engage_attempt()` — HSV hedef takibi, wrong-target kaçınma, contact evidence quorum.

### 1.3 MAVLink/Setpoint Hattı
- Birincil aktüasyon: `_send_guided_speed_heading()` -> `SET_POSITION_TARGET_GLOBAL_INT` (`MAV_FRAME_GLOBAL_INT`, type_mask `0x9C7`) -> vx/vy (NE) + yaw.
- `compass_heading_to_ne_velocity(speed, heading)`: vx=speed*cos(h), vy=speed*sin(h) (0=N, 90=E).
- GUIDED başarısız -> **HOLD** (RC override fallback yasak, `GUIDED_SETPOINT_SEND_FAILED` -> `_enter_hold`).
- RC override yalnız bench/test fallback (`SIM_FALLBACK_MANUAL_RC_ACTUATION=1`), race'te kapalı.

### 1.4 Sim-to-Real Köprüsü
- `sitl_gazebo_bridge.py`: ArduPilot SITL servo çıkışı (CH1/CH3) -> `_servo_outputs_to_left_right` -> `_normalized_motor_outputs` -> `cmd_vel` (Twist: linear.x, angular.z).
- Yaw kontratı: `yaw_cmd_norm = (left_n - right_n)/2`, `yaw_diff = SIM_GZ_YAW_SIGN * yaw_cmd_norm` (default `-1.0`), `twist.angular.z = yaw_diff * MAX_YAW_RATE_RAD_S`. "left_n > right_n -> pozitif nav heading rate."
- Heading: `_nav_heading_deg_from_gazebo_yaw = (-degrees(yaw_rad) + 360) % 360` (compass).
- Pose: Gazebo `x=north, y=west`; ENU `x=east, y=north`; `gazebo_xy_to_spatial_enu = (-y, x)`.
- `motor_command.json` bench override: race'te zorunlu kapalı (`and USV_MODE != "race"`).

### 1.5 Loglama
- Merkezi: `runtime_debug_log.py` (`setup_component_logger`, `log_jsonl`), `run_logger.py` (run-id JSONL).
- Her servis: `{component}.debug.log` + `{component}.jsonl`.
- File3 lokal harita kaydı: `file3_local_map.mp4` + index CSV.
- `predicted_goals.jsonl`, `mission_state.json`, `camera_status.json`, `local_costmap.json`, `autonomy_confidence.json`, `behavior_status.json`, `candidate_trajectory_status.json`, `motor_command.json`, `vehicle_position.json`, `telemetry_link_state.json`.

### 1.6 Uyumluluk Doğrulaması
- `check_compliance_static.py`, `check_compliance_behavior.py`, `check_compliance_race.py` — 8 kabul kriteri.
- Race sertifikasyon: 3 betik + `run_sim_stack.sh mode=race` + `compliance_race_test.jsonl` hatasız + `terminal.log` hata seviyesi yok.

### 1.7 Mevcut Otonomi Davranışının Zayıf Noktaları (log kanıtlı)
- **P0 — İdle/stopta istemsiz dönüş:** `logs/system/usv_main.jsonl` ve `vehicle_position.json` kanıtı: `speed_mps=0.0`, `desired_left_pwm=1500`, `desired_right_pwm=1500` iken `servo_output_raw: servo1_raw=1375, servo3_raw=1625` (asimetrik) ve `observed_yaw_rate_dps=-4.93`. Araç dururken ~5 dps dönüyor. Bu, şartname 4.3 video kriteri "Hareket eksenlerinde istemsiz dönüş/sürüş -> video başarısız" ihlalidir ve sim-to-real bütünlüğünü bozar.
- **usv_main.py monolitik:** 11 528 satır tek dosya; nav/engage/safety/mavlink/health hepsi iç içe. Sürdürülebilirlik ve test edilebilirlik zayıf.
- **P2 karar çoğulluğu:** `compute_nav_decision` (nav_guidance) ve `PredictiveLocalPlanner` (local_planner) paralel çağrılıyor; hangisinin çıktısının geçerli olduğu runtime flag'lerle seçiliyor. Çift karar yolu karar tutarsızlığı ve titreme riski taşıyor.
- **Gate/duba geçiş merkezi seçimi:** Planner gate_candidatebearing'i var ama gate "geçiş merkezi" seçimi kapı eşiği yerine bearing assist ile sınırlı; dar koridorda merkezlenme zayıf olabilir.
- **Heading EMA + control smoothing katmanları üst üste:** `_heading_ema_alpha=0.15`, `_apply_control_smoothing` (alpha + rate limit), planner smoothing — üç katmanlı filtreleme gerçek dinamiği geciktirebilir.

---

## 2. Şartname Uyumu (`documents/ida_sartname.md`)

### 2.1 Gerekli Noktalar
- **3.5 Otonomi yazılımı İDA üzerinde:** Mevcut mimari uyumlu (Pi üzerinde tüm işleme). Plan bu kuralı korur.
- **3.7 Algılama — görüntü aktarımı yasak:** Race modunda stream kapalı (mevcut `SIM_GZ_ALLOW_MOTOR_COMMAND_JSON` ve kamera stream guard uyumlu). Plan race hardening'i güçlendirir.
- **3.4 Haberleşme kısıtları (2.4–2.8 / 5.15–5.85 GHz, hücresel yok):** Mevcut TCP/MAVLink yerel. Plan yeni RF yolu önermez.
- **4.2 Parkurlar arası geçiş otomatik:** Mevcut `run()` otomatik P1->P2->P3. Plan bu otomasyonu korur ve erken geçiş riskini azaltır.
- **4.2 Duba sayısına dayalı algoritma tavsiye edilmez:** Mevcut `split_nav_engage` değişken uzunluklu; sabit indis yok. Plan bu sözleşmeyi korur.
- **4.3 Otonomi kabiliyeti videosu — 3 bölme (YKİ, grafikler, dış kamera), istemsiz dönüş başarısız:** **P0 idle-dönüş bug doğrudan bu kriteri ihlal ediyor.** Plan en yüksek öncelikle bunu giderir.
- **6. Veri teslim (3 dosya, 20 dk, 1 Hz):** File1 kamera mp4, File2 telemetri CSV, File3 lokal harita. Mevcut kayıt altyapısı mevcut; plan tazelik ve frekans doğrulaması ekler.
- **9.2 Puanlama — P1 çarpma, P2 geçiş/çarpma, P3 TS3 (yanlış hedef temas sayısı):** P3 wrong-target kaçınma mevcut; plan wrong-target latch ve TS3 loglamasını güçlendirir.

### 2.2 Riskler
- **P1 erken geçiş:** `USV_RACE_P1_AUTO_WAYPOINTS` varsayılan yorumu ve P2-entry evidence timeout (`P1_AUTO_WAYPOINT_TIMEOUT_S=180`) yanlış ayarlanırsa P1 tamamlanmadan P2'ye sıçrama. Risk: Pixhawk AUTO'dan GUIDED'a erken geçiş P1 saflığını bozar.
- **TS3 puan kaybı:** Wrong-target temas riski aktifken angajman latch kurulması mevcut kodda engelli (`_p3_contact_evidence` quorum `wrong_target_contact_risk` ile bloklanıyor) — bu doğru; ancak wrong-target bearing/area eşiği kalibrasyonu DSB renkler netleşene kadar risk taşır.
- **File3 formatı DSB:** Şartname "Format: TBD" — mevcut mp4 + CSV index yaklaşımı makul ama hakem onayı gerekir (varsayım yapılmaz).

### 2.3 Yasaklar (korunacak)
- P1 race'te Pi motor/PWM/RC override üretimi -> mevcut koruma `USV_USE_RC_OVERRIDE=1` race'te yasak, GUIDED fallback yok. Plan bu zinciri zayıflatmaz.
- Wi-Fi/hücresel/yasak frekans -> önerilmez.
- Görev otomatik başlama -> mevcut RC CH5/API gate korunur.
- Post-start reload/retarget -> mevcut command_lock korunur.

---

## 3. Çalışma Sistemi Uyumu (`documents/rapor_calismasistemi.md`)

### 3.1 Gerçek Malzeme <-> Yazılım Uyumu
- **Pixhawk + ArduPilot Rover:** Düşük seviye kontrol, motor/ESC, heading kapalı çevrim, AUTO/GUIDED — uyumlu. `config/pixhawk_ida.param` baseline manuel sürüş referansı (`ARMING_CHECK=0`, `SERVO1/3` 1100/1500/1900). Plan bu baseline'ı değiştirmez; değişiklik önerilirse su testi sertifikası şartı korunur.
- **Raspberry Pi:** Üst seviye otonomi, kamera/lidar işleme, telemetri — uyumlu.
- **Kamera:** HSV duba/kapı/hedef algılama (`cam.py`) — uyumlu.
- **Lidar:** RPLidar (gerçek) / Gazebo LaserScan (sim) — `lidar_map.py` `/scan` subscriber + dünya nokta birikimi. Uyumlu.
- **STM32:** Yardımcı sensör USB seri — mevcut kodda kullanımı sınırlı; plan varsayım yapmadan bırakır.
- **Crossfire RC:** Manuel/start/E-stop — `rc_channels` ch1/ch3/ch5/ch7 işleme. Uyumlu.
- **433 MHz telemetri:** Mission yükleme + MAVLink — uyumlu.

### 3.2 Simülasyon <-> Gerçek Uyumu
- SITL aynı firmware, `tcp:5760` aynı MAVLink — uyumlu.
- Gazebo fizik `sitl_gazebo_bridge.py` ile SITL servo -> cmd_vel — uyumlu prensipte, **ancak P0 idle-dönüş bug sim fizik/trim katmanında** ve gerçek araçta farklı tezahür edebilir.
- Tekne ölçüleri ~160×81×35 cm (AGENTS.md §9) — costmap `COSTMAP_BOAT_WIDTH_M/LENGTH_M` ile uyumlu olmalı; plan kalibrasyon doğrulaması önerir.

### 3.3 Eksik Bilgiler / Varsayım Yapılmayacak Yerler
- **Hedef dubaları (P3):** tip/çap/yükseklik/3 renk DSB. Plan, renk kilitlenebilir ama renk değerleri DSB olduğu için HSV eşik kalibrasyonu gerçek dubayla saha öncesi yapılmalı; plan sabit renk varsayımı önermez.
- **File3 formatı:** TBD — plan mevcut mp4+CSV saklar, format değişikliği önermez.
- **Tekne dinamiği (kütle/atalet/damping/motor tersliği/yaw sign/max hız):** AGENTS.md §9 su testiyle kalibre edilecek. Plan sim kalibrasyon parametrelerini (`SIM_GZ_YAW_SIGN`, `LEFT_MOTOR_REVERSED`, `MAX_YAW_RATE_RAD_S`, `MAX_LINEAR_VELOCITY_MPS`) doğrulama adımı olarak listeler ama değer önermez.
- **Pixhawk baseline failsafe:** `ARMING_CHECK=0` vb. kapalı — yarış öncesi fiziksel E-stop/kontaktor veya Pixhawk pre-arm profiliyle sertifikasyon şart (README §7). Plan bu sertifikasyon adımını hatırlatır.

---

## 4. Aynı Düzlem / Frame / Kontrol Tutarlılığı

### 4.1 Mevcut Frame Sözleşmesi (doğrulandı)
- **Compass heading:** 0=N, 90=E, saat yönü pozitif. `compass_heading_to_ne_velocity`: vx=speed*cos(h), vy=speed*sin(h). ✓
- **Heading error:** `normalize_heading_error(target - current)`, pozitif = saat yönü (sağa dönüş). ✓
- **Motor mix:** pozitif heading error -> sağ thruster daha güçlü (`right_mix = surge + yaw*gain`). ✓
- **Sim yaw:** `yaw_cmd_norm = (left_n - right_n)/2`, `SIM_GZ_YAW_SIGN=-1` -> `twist.angular.z = -1*yaw_cmd_norm*max_rate`. Gazebo angular.z pozitif CCW; nav heading pozitif CW -> sign -1 doğru. ✓ (kontrat `left_n > right_n -> pozitif nav heading rate`).
- **Gazebo pose:** x=north, y=west -> ENU east=-y, north=x. ✓
- **Nav heading from Gazebo yaw:** `(-deg(yaw)+360)%360`. ✓

### 4.2 Tespit Edilen Riskler
- **P0 — İdle dönüş (frame değil, trim/SITL servo):** Komut 1500/1500 (nötr) iken SITL `SERVO_OUTPUT_RAW` 1375/1625 asimetrik -> `observed_yaw_rate=-4.93 dps`. Bu frame hatası değil; **ArduPilot SITL servo trim / `SERVO3_REVERSED` / `MOT_TRIM` / `SIM_GZ_*` normalizasyon** kaynaklı residual yaw. Sim-to-real'de gerçek ESC trim/farklı thruster karakteristiğiyle daha kötü olabilir. **En yüksek öncelik.**
- **Heading EMA + smoothing katmanları:** Üç katmanlı filtreleme (EMA 0.15, control smoothing alpha, planner smoothing) dönüşleri geciktirip aşarı düzeltmeye (overshoot) sebep olabilir. Gerçek tekne dinamiğinde su direnciyle birlikte salınım riski.
- **NED/ENU karışıklığı potansiyeli:** `handle_odometry` Gazebo odometry'yi NED'e çeviriyor (`x_ned=p.x, y_ned=-p.y`) ama heading `q_to_euler` ile ayrı işleniyor. Lat/lon hesabı `home_lat + x_ned/111320` — bu NED x'in kuzey olduğunu varsayıyor; Gazebo `p.x` zaten kuzey olduğundan tutarlı, ama birim test kapsamı olmalı.
- **LIDAR frame:** `local_costmap` base_link (x=forward, y=left); `spatial_frame.world_enu_to_lidar_local` heading_rad ile dönüyor. `lidar_local_to_world_enu_gazebo` ayrı bir fonksiyon — iki dönüşüm yolu tutarsızsa harita/costmap kayması. Doğrulama testi gerek.
- **Kamera bearing:** `cam.py` `gate_center_bearing_deg` ve `target_bearing_deg` — hangi frame'de (görüntü x ekseninde derece, 0=merkez, +=sağ?) `nav_guidance` ile uyumu `resolve_camera_bearing_half_deg` üzerinden. Compass heading'e bearing ekleme mantığı `compute_nav_decision`'da `gate_assist_bias` olarak uygulanıyor — işaret tutarlılığı birim testle doğrulanmalı.

### 4.3 Plan: Frame Tutarlılık Doğrulaması
- Statik birim test: `check_spatial_frame_unit.py` (mevcut) genişletilsin — ENU<->Gazebo<->NED<->compass heading gidiş-dönüş (round-trip) tutarlılığı.
- Sim-to-real yaw kontratı testi: `analyze_heading_oscillation.py` (mevcut) ile WP1->WP2 ters dönüşte `heading_error` işareti <-> `observed_yaw_rate` işareti tutarlılığı doğrulansın.
- **P0 idle-dönüş için:** SITL servo nötr kalibrasyonu ve `stop_motors`/`_send_guided_speed_heading(0,...)` sonrası `observed_yaw_rate≈0` assert'u (log tabanlı, otomatik motion değil).

---

## 5. Önerilen Mimari İyileştirmeler

### 5.1 Karar Yolu Tek Lezzeti (Single Decision Authority)
- **Sorun:** `compute_nav_decision` (nav_guidance) ve `PredictiveLocalPlanner` paralel; runtime'da hangisinin geçerli olduğu `_run_predictive_local_planner` çıktısına göre override ediliyor. Bu çift yollar karar titremesi ve hata ayıklama zorluğu yaratır.
- **Öneri:** P2/P3 için **tek karar otoritesi** olarak `PredictiveLocalPlanner` merkezîleştirilsin; `compute_nav_decision` planner'a **nominal waypoint/gate bias girdisi** sağlayan bir hesaplayıcıya indirgensin (safe/validated nominal heading üretir, planner bunu maliyetlendirir). Böylece tek skorlama tek geçerli çıktı üretir.
- **Risk:** Planner parametreleri yanlış ayarlanırsa tüm P2/P3 bozulur -> mevcut `compute_nav_decision` fallback olarak tutulur, planner disabled flag'iyle A/B geçiş yapılabilir.
- **Fayda:** Karar tutarlılığı, tek log akışı, daha az titreme, kolay tuning.

### 5.2 P0 — İdle/Stop Sıfır-Yaw Garantisi
- **Sorun:** `speed=0` komutunda SITL servo asimetrik -> dönüş.
- **Öneri:** (a) `stop_motors`/`_hold_waypoint_neutral`/`_send_guided_speed_heading(0,...)` sonrası `observed_yaw_rate_dps` izlensin; eşiği aşarsa uyarı + `SERVO_OUTPUT_RAW` loglansın. (b) SITL parametre `SERVO1/3_TRIM`, `SERVO3_REVERSED`, `MOT_PWM_TYPE` ve `SIM_GZ_PWM_TRIM_US` nötr kalibrasyonu doğrulansın (statik param karşılaştırma `compare_pixhawk_param_baseline.py` ile). (c) Gerekiyorsa `_send_guided_speed_heading` sıfır hızda `yaw_rate` istenen heading'e sabitlensin (type mask yaw_rate kullanımı değerlendirilsin) ama **sıfır hızda yaw_rate setpoint eklemek gerçek araçta beklenmeyen hareket yaratabilir** -> önce SITL trim düzeltimi denenmeli.
- **Risk:** Sıfır hızda yaw_rate setpoint gerçek dinamiği bozabilir -> bu seçenek yalnız sim'de ve log kanıtlı SITL trim sorunu ise değerlendirilir.
- **Fayda:** Video kriteri ihlali giderilir; sim-to-real duruş güvenliği.

### 5.3 Refactor — usv_main.py Parçalanması (sınırlı, geri alınabilir)
- **Sorun:** 11 528 satır monolitik.
- **Öneri:** Saf fonksiyon grupları ayrı modüllere taşınır (mevcut import'larla geriye dönük uyumlu):
  - `usv_health.py`: trust/health/watchdog/preflight.
  - `usv_mavlink_io.py`: heartbeat/GPS/EKF/SYS_STATUS/RC parsing, mode set, GPS origin seed.
  - `usv_safety.py`: E-stop, command_lock, virtual anchor, HOLD, failsafe.
  - `usv_nav_engine.py`: P2 waypoint döngüsü, align phase, leg progress, gate event tracking.
  - `usv_engage_engine.py`: P3 attempt, contact evidence, wrong-target, retreat.
  - `USVStateMachine` lifecycle + I/O koordinasyonu kalır.
- **Risk:** Büyük refactor -> **yalnız plan olarak**, küçük adımlara bölünmüş, her adımda `py_compile` + 3 compliance betiği çalıştırılır.
- **Fayda:** Test edilebilirlik, hata ayıklama, sim-to-real doğrulama kolaylığı.

### 5.4 Terminal Log Okunabilirliği
- **Mevcut:** `run_sim_stack.sh` `ui_*` helper'ları renkli (`SIM_UI_COLOR`, `NO_COLOR`, CI algılama). `console_utils.py` bileşen-renk eşlemesi var ama servis stdout'ları dosyalarına yönlendirildiği için terminalde yalnız `[SIM]` orchestrator mesajları görünür.
- **Öneri:** Her servis başlatılırken `stdbuf -oL` ile satır tamponlama + `console_utils.make_console_printer` (mevcut) servis içine takılsın; renk `NO_COLOR`/CI ile kapatılır, dosya loglarını etkilemez. Servis prefix: `[GAZEBO]`, `[ROS-GZ]`, `[SITL]`, `[MAVPROXY]`, `[CAM]`, `[LIDAR]`, `[USV]`, `[TELEM]`. `run_sim_stack.sh` servis çıktılarını `sed`/prefix wrapper ile etiketleyip terminal akışına katsın (dosya logları bozulmadan).
- **Risk:** Çok fazla terminal çıktı -> özet/seviye filtresi; hata/uyarı her zaman görünür.
- **Fayda:** Operatör hata kaynağını hızlı tespit eder.

---

## 6. Refactor / Baştan Yazma Planı

| Alan | Mevcut Sorun | Önerilen Değişiklik | Risk | Beklenen Fayda | Öncelik |
|---|---|---|---|---|---|
| İdle/stop yaw | `speed=0` iken SITL servo 1375/1625 -> ~5 dps dönüş | SITL servo trim doğrulama + sıfır-yaw izleme/log; gerekirse param düzeltimi | Sıfır-hızda yaw_rate setpoint gerçek aracı bozabilir -> yalnız sim/trim | Video kriteri + duruş güvenliği | P0 |
| Karar otoritesi | nav_guidance + planner paralel override | Planner tek otorite, nav_guidance nominal girdi | Planner tuning -> fallback tutulur | Karar tutarlılığı, az titreme | P1 |
| usv_main monolitik | 11 528 satır | Saf grupları 5 modüle taşı (lifecycle kalır) | Büyük refactor -> küçük adımlar + compliance | Test/sürdür | P2 |
| Heading katmanları | EMA + control smoothing + planner smoothing üst üste | Tek smoothing katmanı (planner içine), EMA kaldırma değerlendir | Daha az gecikme ama daha çok titreşim riski | Dönüş kararlılığı | P1 |
| Costmap gate center | Bearing assist sınırlı merkezleme | Gate eşiğinden geçiş merkezi sub-goal üretimi | Ek karmaşıklık | Dar koridor geçiş puanı | P1 |
| P3 wrong-target latch | Mevcut quorum doğru ama TS3 log detayı | TS3 sayaç + wrong-target geçiş logu güçlendir | Düşük | P3 puan/şeffaflık | P2 |
| Terminal prefix | Servis stdout dosyaya, terminal sade | `console_utils` servis içi + `run_sim_stack` prefix | Çıktı kalabalığı -> seviye filtre | Operatör tanı | P2 |
| File3 tazelik | 1 Hz kayıt varsayılan | Frekans/tazelik doğrulama testi | Düşük | Teslim uyumu | P3 |

> **Not:** Baştan yazma yalnız `usv_main.py` parçalanması için düşünülür ve o da küçük, geri alınabilir adımlara bölünür. Diğer modüller (`local_planner`, `local_costmap`, `sensor_fusion`, `nav_guidance`) mevcut yapıları sağlam; refactor değil, merkezileştirme/iyileştirme önerilir.

---

## 7. P1/P2/P3 Davranış Planı

### 7.1 P1 (Parkur-1)
- **Race:** Saf Pixhawk `AUTO` saflığı korunur. Pi yalnız: sağlık, log, mission progress, failsafe izler. `_run_nav_auto_mission` `guidance_source="nav_auto_monitor"`, motor/PWM/RC override üretmez.
- **P1->P2 erken geçiş riski:** `USV_RACE_P1_AUTO_WAYPOINTS` ve P2-entry evidence timeout (`P1_AUTO_WAYPOINT_TIMEOUT_S=180`) doğrulansın; P1 tamamlanmadan GUIDED'a sıçrama olmasın. P1 bitti kanıtı: `mission_reached_seq` veya explicit-count acceptance.
- **P1 sonrası P2 hazır kontrolü:** `ready_state`, `camera_ready`, `lidar_ready` P2'ye geçişte zorunlu (`_wait_p2_ready`). Kamera/lidar bayatsa P2'ye geçiş engellensin veya HOLD.
- **Test modunda:** P1 de Pi GUIDED (Pixhawk AUTO yok) — bu test davranışı korunur ama race'te AUTO saflığı bozulmaz.
- **Plan iyileştirme:** P1 monitor'e "P1 geçiş onay kanıtı" logu (reached_seq + dist) eklensin; şeffaflık artar.

### 7.2 P2 (Parkur-2)
- **LIDAR öncelikli kaçınma:** Costmap + 3-sektör + TTC. LIDAR tehditi kamera hedeflemesinden öncelikli (mevcut `_blend_with_avoidance` ve planner collision reject).
- **Kamera destekli gate/duba:** `gate_assist_bias` waypoint bearing etrafında sınırlı (mevcut ±35° gate delta, ±12° bias). Gate center geçiş merkezi sub-goal olarak planner'a girdi eklensin.
- **Güvenli geçiş koridoru:** `TRAVERSABLE_CORRIDOR_*` parametreleri (lookahead 7m, half-width 0.75m) — dar geçişte hız düşürme (`P2_LIDAR_WARN_M=2.5`, `D_MIN_M=2.0`).
- **En az 2 gate/duba geçişi:** `p2_min_gate_count=2` (mission_config). `_track_gate_event` + `_wait_for_p2_gate_minimum`.
- **Final P2 noktası:** Son nav_waypoint acceptance (`R_WP_M`) + gate minimum -> P3.
- **Dar geçiş hız düşürme:** `schedule_waypoint_speed` approach window + obstacle warn cap + `FAILSAFE_SLOW_MPS`.
- **Sensör bayatlık:** `sensor_fusion` degraded/HOLD; LIDAR `LIDAR_DEGRADED_HOLD_TIMEOUT_S=8s` -> HOLD.
- **Karar logu:** `nav_track` jsonl (heading_error, dist, guidance_source, blocked_level, gate, cross_track) — mevcut; planner `candidate_trajectory_status.json` + `behavior_status.json`.

### 7.3 P3 (Parkur-3)
- **Hedef rengi start öncesi kilit:** `mission_profile.target_color` + `validation_timestamp`; start sonrası değişim reddedilir (command_lock).
- **Yanlış hedefler kaçınılacak nesne:** `wrong_target_*` alanları `camera_status.json`; `_p3_contact_evidence` wrong-target riski varsa latch kurulmaz; wrong-target bearing'e kaçınma (`p3_wrong_target_avoid`).
- **Doğru hedefe kontrollü yaklaşma:** `_p3_speed_for_target_area` (area↑ -> hız↓), `P3_TARGET_BEARING_GAIN=0.85`, `P3_TARGET_HEADING_CLAMP_DEG=45`.
- **Hedef kaybolursa:** `target_memory` 8s memory + arama spiral (`search_bearing`); körlemesine GPS fallback sınırlı (`P3_GPS_FALLBACK_SPEED_MPS=0.55`).
- **Temas sensörü yok:** Dolaylı kanıt quorum (`vision_area`, `gps_proximity`, `lidar_proximity`, `lidar_collision`, `low_speed`) + 0.6s sürdürme -> `p3_contact_confirmation_source` loglanır.
- **Yanlış hedef riski varsa latch yok:** Mevcut `quorum_ok = ... if not wrong_target_contact_risk` — korunur.
- **P3 tamam kararı:** Açık gerekçeyle loglanır (`P3_CONTACT_LATCH` jsonl: source, dist, area, lidar, collision).
- **Plan iyileştirme:** TS3 (yanlış hedef temas sayısı) sayaç ve geçiş logu `p3_wrong_target_contact_count` mission_state'e geriye dönük uyumlu ek; angajman tamam şeffaflığı.

---

## 8. Engel Kaçınma ve Rota Planlama Planı

### 8.1 LIDAR Sektör Analizi
- Mevcut: `local_costmap.extract_sector_metrics` (left/center/right min + clear + near_collision) + `classify_lidar_scan_sector`/`classify_lidar_map_sector`.
- **İyileştirme:** Sektör sayısı 3'ten 5'e (center-left/center/center-right ek) — daha yumuşak koridor seçimi; ama 3-sektör `obstacle_avoidance.decide_three_sector_avoidance` ile uyumlu kalacak (gruplama).

### 8.2 Güvenli Koridor Seçimi
- Mevcut: `_select_corridor` (center/left/right/blocked) + planner candidate heading.
- **İyileştirme:** Koridor kararlılığı için hysteresis margin (mevcut `PLANNER_HYSTERESIS_SCORE_MARGIN`); sol-sağ titreme için `NAV_AVOID_BIAS_SLEW_DEG_PER_S` slew limit (mevcut) doğrulansın.

### 8.3 Yakın Engel Önceliği + Kritik Çarpışma Bölgesi
- Mevcut: `OBSTACLE_TTC_STOP/EMERGENCY/DANGER/WARN_S` + `speed_profile_for_ttc` (stop->0, emergency->0, danger->0.25, warn->0.50).
- **İyileştirme:** `near_collision` (costmap) -> `COLLISION_IMMINENT` behavior -> HOLD/stop; mevcut behavior priority map korunur.

### 8.4 Orta Mesafe Erken Kaçınma + Hız Düşürme
- Mevcut: `P2_LIDAR_WARN_M=2.5` warn, `D_MIN_M=2.0` sert.
- **İyileştirme:** Warn mesafesinde planner erken candidate shift + `dynamic_ttc` ile orantılı hız düşürme (mevcut `speed_factor`).

### 8.5 Heading Smoothing + Titreme Önleme
- Mevcut: `_apply_control_smoothing` (heading rate limit `CONTROL_HEADING_RATE_LIMIT_DPS`, alpha) + planner smoothing.
- **İyileştirme:** Üç katman -> ikiye (EMA + control smoothing birleştirilsin veya EMA kaldırılsın); `avoid_switch_count` izleme (mevcut) ile titreme tespiti.

### 8.6 Stuck Detection + Recovery
- Mevcut: `PredictiveLocalPlanner._detect_stuck` (progress/distance + `PLANNER_STUCK_TIMEOUT_S`) + `recovery_action="switch_corridor"`.
- **İyileştirme:** Recovery eylem seti genişletsin: (1) koridor değiştir, (2) geri çekil (`P3_REVERSE_*` benzeri P2 için kısa geri), (3) HOLD+log. Sim recovery `_maybe_trigger_sim_avoidance_recovery` mevcut.

### 8.7 Çarpışma Riski -> HOLD/Fail-safe
- Mevcut: `COLLISION_IMMINENT` -> HOLD; `SENSOR_DEGRADED` -> slow/HOLD.
- **İyileştirme:** Eşik ve davranış korunsun; log şeffaflığı.

### 8.8 Öngörülü Rota
- Mevcut: `_predict_body_trajectory` (horizon `PREDICTED_TRAJECTORY_HORIZON_S`, step) + candidate heading skorlama (progress/collision/gate/smoothness/sensor/wrong_target/boundary).
- **İyileştirme:** (a) Gate/duba geçiş merkezi sub-goal candidate seed; (b) çarpışmalı rota eleme (mevcut `rejected_collision`); (c) hedefe ilerleme skoru (mevcut `PLANNER_W_PROGRESS`); (d) rota pürüzsüzleştirme (mevcut `PLANNER_SMOOTHING_ALPHA`); (e) faz bazlı davranış (P2 gate-assist, P3 target-track) — mevcut `state` parametresi.

---

## 9. Lokal Harita / Costmap Planı

### 9.1 Rolling Local Costmap
- Mevcut: `local_costmap.py` base_link grid (`COSTMAP_FRONT/REAR/SIDE_M`, resolution 0.2m, width/height cells). Rolling (araç hareket ettikçe).
- **İyileştirme:** Grid boyutu tekne eni + dönüş yarıçapı güvenlik payı (`COSTMAP_SAFETY_MARGIN_M`) doğrulansın.

### 9.2 Obstacle Inflation
- Mevcut: `COSTMAP_INFLATION_RADIUS_M`, `COSTMAP_OBSTACLE_COST/FATAL/INFLATED`.
- **İyileştirme:** Inflation yarıçapı tekne eninin yarısı + margin (mevcut `boat_width_m`).

### 9.3 Decay
- Mevcut: `COSTMAP_DECAY_FACTOR`, `COSTMAP_DECAY_INTERVAL_S` — eski engel kaybolma.
- **İyileştirme:** Decay oranı LIDAR tazelik ile ilişkilendirilsin (bayat sensör -> decay hızlansın).

### 9.4 Sensör Freshness
- Mevcut: `CostmapSnapshot.fresh` (freshness_timeout 2.0s), `drop_reason`.
- **İyileştirme:** Bayat costmap -> planner `degraded`, hız düşürme/HOLD.

### 9.5 Kamera -> LIDAR Desteği
- Mevcut: `COSTMAP_CAMERA_COST` kamera tespiti costmap'e işlenir.
- **İyileştirme:** Gate/duba merkezleri costmap'te geçilebilir (low cost), wrong-target engel (high cost).

### 9.6 Yanlış Hedef -> Engeller
- Mevcut: `PLANNER_W_WRONG_TARGET` planner maliyeti.
- **İyileştirme:** Wrong-target konumu costmap'e high-cost engel olarak işlensin (P3'te).

### 9.7 Race Modda Dış Stream Yok
- Mevcut: File3 yerel mp4; race'te dış stream kapalı.
- **İyileştirme:** Costmap görselleştirme yalnız yerel dosya (dashboard test modunda).

---

## 10. Fail-Safe ve Güvenlik Planı

### 10.1 Güven Skoru
- Mevcut: `autonomy_confidence.json` (sensor/clearance/collision_free/stability) + `TRUST_*` threshold + `fusion_confidence`.
- **İyileştirme:** Güven breakdown'u genişletsin: LIDAR güveni, kamera güveni, GPS/heading güveni, costmap güveni, planner güveni, kontrol güveni + sensörler arası tutarlılık. Düşük güven -> hız düşürme/degraded/HOLD karar ağacı netleşsin:
  - `confidence < SLOW_THRESHOLD` -> hız düşür.
  - `confidence < HOLD_THRESHOLD` veya `hold_recommended` -> HOLD.
  - `ttc_level in (stop, emergency)` -> HOLD.
  - LIDAR bayat > `LIDAR_DEGRADED_HOLD_TIMEOUT_S` -> HOLD.
  - GPS bayat -> degraded (heading yalnız LIDAR/camera).
  - GUIDED gönderim başarısız -> HOLD (RC override yok).

### 10.2 Fail-Safe Zinciri
- **E-stop:** RC CH7 ≥ 1900 veya API `emergency_stop.flag` -> `_trigger_estop` -> fiziksel güç kesme mantığı zayıflatılmaz.
- **RC override:** `_is_rc_stick_active` -> her zaman otonomiden öncelikli.
- **HOLD:** `_enter_hold` -> `stop_motors` + `HOLD` mode + virtual anchor (failsafe).
- **Guided fail:** `GUIDED_SETPOINT_SEND_FAILED` -> HOLD (RC fallback yasak).

### 10.3 Plan
- Fail-safe geçişleri net loglansın (reason + timestamp).
- Güven skoru eşikleri `compliance_profile.py`'den merkezi; environment override yalnız test.

---

## 11. Loglama ve Doğrulama Planı

### 11.1 Kritik Karar Logları (her döngü/olay)
- Aktif davranış (`behavior_status.json`), seçilen koridor/heading/hız (`candidate_trajectory_status.json`, `nav_track`/`p3_track` jsonl), karar nedeni (`decision_reason`), sensör tazeliği (`fusion_state`), confidence (`autonomy_confidence.json`), fail-safe nedeni, P3 temas doğrulama kaynağı (`p3_contact_confirmation_source`), planner risk (`cost_breakdown`), stuck/recovery, gate candidate.

### 11.2 Mevcut Loglar Bozulmadan
- `{component}.debug.log` + `{component}.jsonl` korunur; rotasyon kapalı kalmaz.
- Sim yeni loglar dashboard log görüntüleyicisinde görünür (AGENTS.md §10).

### 11.3 Race Modda Dış Stream Yok
- Tüm log/çıktı yerel dosya; görüntü aktarımı amacıyla kullanılmaz.

### 11.4 Doğrulama
- File1/File2/File3 frekans ≥1 Hz ve tazelik testi (statik).
- `predicted_goals.jsonl`, `local_costmap.json` ≥1 Hz.

---

## 12. Değiştirilecek Dosyalar

| Dosya | Yapılacak İş |
|---|---|
| `docker_workspace/src/usv_main.py` | P0 idle-yaw izleme/log; karar otoritesi tek lezzeti (planner merkezî); smoothing katman birleştirme; (P2) saf grupların modüllere taşınması |
| `docker_workspace/src/local_planner.py` | Tek karar otoritesi; gate center sub-goal; koridor hysteresis; recovery eylem seti |
| `docker_workspace/src/nav_guidance.py` | `compute_nav_decision` -> planner nominal girdi hesaplayıcıya indirgenme |
| `docker_workspace/src/local_costmap.py` | Sektör 5'e genişletme (opsiyonel); wrong-target high-cost engel; decay-tazelik ilişkisi |
| `docker_workspace/src/obstacle_avoidance.py` | 5-sektör gruplama uyumu (3-sektör API korunarak) |
| `docker_workspace/src/compliance_profile.py` | Yeni threshold/parametre merkezi (güven skoru eşikleri, recovery, gate center) |
| `docker_workspace/src/sensor_fusion.py` | Güven breakdown genişletme; tutarlılık metrikleri |
| `docker_workspace/src/console_utils.py` | Servis içi prefix/renk entegrasyonu (mevcut genişletme) |
| `sim/bin/run_sim_stack.sh` | Servis stdout prefix/satır tampon; seviye filtre |
| `sim/bridges/sitl_gazebo_bridge.py` | P0 idle-yaw tanı logu (observed yaw @ neutral) |
| `host_scripts/check_spatial_frame_unit.py` | Frame round-trip test genişletme |
| `host_scripts/analyze_heading_oscillation.py` | WP dönüş yaw-tutarlılık doğrulama |
| `host_scripts/check_compliance_static.py` | Yeni kontrat görünürlükleri (güven skoru, gate center, TS3) |
| `config/pixhawk_ida.param` | **Yalnız su testi sertifikasyonu sonrası** idle trim doğrulama (değişiklik önerilmez önce) |
| `OTONOM_SURUS_IYILESTIRME_PLANI.md` | Bu dosya |

> **Önemli:** `config/pixhawk_ida.param` ve `sim/bridges` motor eşleşmesi/yaw sign değerleri su üstü testiyle doğrulanmadan değiştirilmez (AGENTS.md §8, §9). Plan yalnız **doğrulama** önerir, değer önermez.

---

## 13. Test Planı

### 13.1 Statik/Doğrulayıcı Testler (ajan çalıştırabilir)
```bash
python3 -m py_compile docker_workspace/src/*.py
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
python3 host_scripts/check_compliance_race.py
python3 host_scripts/check_spatial_frame_unit.py
python3 host_scripts/test_obstacle_avoidance_advanced.py
python3 host_scripts/test_target_memory.py
python3 host_scripts/test_trajectory_tracker.py
python3 host_scripts/test_kalman_filter.py
python3 host_scripts/check_advanced_autonomy_unit.py
python3 host_scripts/check_nav_guidance_unit.py
python3 host_scripts/check_mission_profile_unit.py
python3 host_scripts/check_race_readiness_score.py
python3 host_scripts/compare_pixhawk_param_baseline.py
```

### 13.2 Manuel Simülasyon Kontrolleri (operatör yapar)
- `run_sim_stack.sh` test modu: P1->P2->P3 manuel start, waypoint geçiş, gate geçiş, engel kaçınma, P3 angajman gözlem.
- `run_sim_stack.sh mode=race`: P1 AUTO saflığı, P2/P3 GUIDED, post-start lock, RC CH5 start.
- İdle-dönüş gözlem: start öncesi aracın dönmediği doğrulansın (P0).
- WP1->WP2 ters dönüşte heading_error <-> yaw_rate işaret tutarlılığı (`analyze_heading_oscillation.py`).

### 13.3 Çalıştırılmayacak Otomatik Motion Testleri (nedeni)
- `./sim/bin/run_sim_stack.sh --auto-test` ve türevleri **ajan tarafından asla çalıştırılmaz** (AGENTS.md §13). Sürüş/motion/mission progression operatör manuel yapar. Ajan yalnız log inceleme + statik/uyum doğrulaması.

---

## 14. Riskler

1. **P0 idle-dönüş sim/trim kaynaklı:** SITL parametre düzeltimi gerçek araçta farklı tezahür edebilir -> su testi öncesi sim'de doğrulama, gerçek araçta manuel idle gözlem zorunlu.
2. **Planner tek otorite tuning:** Yanlış ağırlık (`PLANNER_W_*`) tüm P2/P3'ü bozabilir -> mevcut `compute_nav_decision` fallback korunacak, A/B geçiş flag'i.
3. **usv_main parçalanması:** Büyük refactor -> her adımda compliance + py_compile; geri alınabilir küçük adımlar.
4. **Smoothing katman birleştirme:** Daha az gecikme ama daha çok titreşim -> `avoid_switch_count` ve `analyze_heading_oscillation` ile doğrulama.
5. **Hedef renkleri DSB:** HSV kalibrasyonu saha öncesi gerçek dubayla; plan sabit renk varsayımı önermez.
6. **File3 formatı TBD:** Mevcut mp4+CSV saklanır; hakem onayı beklenir.
7. **Pixhawk baseline `ARMING_CHECK=0`:** Yarış öncesi fiziksel E-stop/kontaktor veya pre-arm profili sertifikasyonu zorunlu (README §7).
8. **Tekne dinamiği kalibrasyonu:** Kütle/atalet/damping/motor tersliği/yaw sign/max hız su testiyle; plan değer önermez.

---

## 15. Uygulama Sırası (küçük adımlar)

1. **P0 idle-yaw teşhisi:** `sitl_gazebo_bridge.py` + `usv_main.py` neutral-yaw izleme/log; `compare_pixhawk_param_baseline.py` ile SITL servo trim doğrulama; statik testler. (Operatör manuel sim idle gözlem.)
2. **Karar otoritesi merkezileştirme (A/B):** `local_planner.py` gate center sub-goal + koridor hysteresis; `nav_guidance.py` nominal girdiye indirgenme; fallback flag; statik testler.
3. **Güven skoru + fail-safe ağacı:** `sensor_fusion.py` breakdown + `compliance_profile.py` eşikler; `usv_main.py` degrade/HOLD karar netleşme; statik testler.
4. **Costmap/wrong-target:** `local_costmap.py` wrong-target high-cost + decay-tazelik; `local_planner.py` P3 wrong-target candidate; statik testler.
5. **Smoothing birleştirme:** `usv_main.py` EMA + control smoothing birleştirme; `analyze_heading_oscillation` doğrulama (operatür manuel).
6. **P3 TS3 şeffaflık:** `usv_main.py` wrong-target geçiş sayacı + `mission_state.json` geriye dönük uyumlu alan; statik testler.
7. **Terminal okunabilirlik:** `console_utils.py` + `run_sim_stack.sh` prefix/satır tampon; dosya logları bozulmadan.
8. **(P2) usv_main parçalanma:** Küçük adımlarla saf grupları modüllere taşı; her adımda compliance.
9. **Sertifikasyon:** 3 compliance betiği + `run_sim_stack.sh mode=race` (operatör) + `compliance_race_test.jsonl` + `terminal.log` hata seviyesi.

---

## 16. TODO

- [ ] P0: `sitl_gazebo_bridge.py` + `usv_main.py` neutral PWM'de `observed_yaw_rate` izleme ve uyarı logu
- [ ] P0: SITL servo trim (`SERVO1/3_TRIM`, `SERVO3_REVERSED`, `SIM_GZ_PWM_TRIM_US`) `compare_pixhawk_param_baseline.py` ile doğrulama
- [ ] P0: Operatör manuel sim idle dönüş gözlemi (aracın durduğu teyit)
- [ ] P1: `local_planner.py` gate/duba geçiş merkezi sub-goal üretimi
- [ ] P1: `local_planner.py` koridor hysteresis + titreme önleme doğrulama
- [ ] P1: `nav_guidance.py` `compute_nav_decision` nominal girdi hesaplayıcıya indirgenme (fallback flag ile)
- [x] P1: `usv_main.py` heading EMA + control smoothing katman birleştirme
- [ ] P1: `sensor_fusion.py` güven breakdown (LIDAR/kamera/GPS/costmap/planner/kontrol + tutarlılık)
- [ ] P1: `usv_main.py` güven eşik -> degrade/HOLD karar ağacı netleşme
- [ ] P2: `local_costmap.py` wrong-target high-cost engel + decay-tazelik ilişkisi
- [ ] P2: `local_planner.py` P3 wrong-target candidate maliyet
- [x] P2: `usv_main.py` P3 TS3 wrong-target temas sayacı + `mission_state.json` geriye dönük uyumlu alan
- [x] P2: `console_utils.py` + `run_sim_stack.sh` servis prefix/satır tampon (dosya logları bozulmadan, `NO_COLOR`/CI uyumlu)
- [ ] P2: `usv_main.py` saf gruplarının modüllere taşınması (küçük adımlar, her adımda compliance)
- [ ] P3: `host_scripts/check_spatial_frame_unit.py` ENU<->Gazebo<->NED<->compass round-trip test genişletme
- [ ] P3: File1/File2/File3 ≥1 Hz tazelik doğrulama testi
- [ ] P3: `host_scripts/check_compliance_static.py` yeni kontrat görünürlükleri (güven skoru, gate center, TS3)
- [ ] Sertifika: 3 compliance betiği + `run_sim_stack.sh mode=race` (operatör) + artefakt hata kontrolü

---

> **Not:** Bu plan `documents/ida_sartname.md` (Bölüm 2/3/4/6/9) ve `documents/rapor_calismasistemi.md` (Bölüm 2/3/4/5/6/7) ile uyumludur. P1 race saflığı, RC override önceliği, E-stop zinciri, post-start lock, görüntü aktarım yasakları ve otonominin İDA üzerinde kalması kuralları korunmuştur. Hiçbir öneri yasak frekans, hücresel bağlantı veya hayali donanım içermez. Eksik bilgiler (hedef renk DSB, File3 format TBD, tekne dinamiği kalibrasyonu) varsayım yapılmadan risk olarak işaretlenmiştir.
