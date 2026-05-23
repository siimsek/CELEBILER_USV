# Manuel Operatör Doğrulama Checklist — CELEBILER USV

Bu checklist **ajan tarafından otomatik koşturulmaz**. Operatör test modunda sim stack veya gerçek donanım üzerinde mission progression doğrular.

**Ön koşul:** Statik uyum geçmiş olmalı:
```bash
python3 -m py_compile docker_workspace/src/*.py
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
python3 host_scripts/check_compliance_race.py
python3 host_scripts/check_mission_profile_unit.py
python3 host_scripts/check_race_readiness_score.py
```

---

## A. Sim stack hazırlık (operatör)

| # | Adım | Pass |
|---:|---|---|
| A1 | `./sim/bin/run_sim_stack.sh` (auto-test **yok**) | ☐ |
| A2 | Dashboard `http://127.0.0.1:8080` — `ready_state`, `camera_ready`, `lidar_ready` true | ☐ |
| A3 | `sim/bin/check_stack.sh` — core servisler OK | ☐ |
| A4 | `logs/system/compliance/pixhawk_param_baseline.json` incelendi | ☐ |

---

## B. Test modu mission progression

| # | Adım | Beklenen | Pass |
|---:|---|---|---|
| B1 | Structured mission + target_color yükle (flat-only değil) | API 200 | ☐ |
| B2 | Dashboard **Start** veya `POST /api/start_mission` | Preflight geçer, NAV başlar | ☐ |
| B3 | P1 waypoint ilerlemesi | `nav_track` / waypoint index artar | ☐ |
| B4 | P2 gate assist | `gate_assist_bias_deg` loglanır, min 2 gate | ☐ |
| B5 | P3 engage | target_color lock, multi-cue olmadan latch yok | ☐ |
| B6 | E-stop / abort | HOLD, motor neutral, post-start upload 409 | ☐ |

---

## C. Yarış modu dry-run (görüntü kapalı)

| # | Adım | Pass |
|---:|---|---|
| C1 | `USV_MODE=race` stack — kamera/lidar map stream 403 | ☐ |
| C2 | API start reddedilir; yalnız RC CH5 start yolu | ☐ |
| C3 | `mission_profile_race_ready` false iken start reddedilir | ☐ |

---

## D. Log audit (post-run)

| # | Log | Kontrol |
|---:|---|---|
| D1 | `usv_main.jsonl` | mode, guidance_source, P2/P3 olayları |
| D2 | `cam.jsonl` | wrong_target_policy, pipeline_metrics |
| D3 | `sitl_gazebo_bridge.jsonl` | sim_motor_contract, yaw sign |
| D4 | `python3 host_scripts/analyze_heading_oscillation.py` | max heading error / sign flips |
| D5 | `usv_main.jsonl` → `guided_setpoint` | `turn_phase=true` iken body-forward creep (`speed_mps` ≤ 0.18 sim) |
| D6 | `usv_main.jsonl` → `nav_track` | WP geçişi align **< 15 s**, ileri hareket **< 20 s**; `decision_mode` çoğunlukla `nav_waypoint_track*` |
| D7 | `sitl_gazebo_bridge.jsonl` | `left_norm` / `right_norm` sürekli negatif olmamalı |
| D8 | Lidar canvas + `lidar_frame` | Noktalar kalıcı; `map_occupied_cells` sık >5 düşüş yapmamalı |

---

## E. Saha / su testi (yarış öncesi zorunlu)

| # | Belge | İmzalı |
|---:|---|---|
| E1 | `documents/servo_mapping_water_test_plan.md` | ☐ |
| E2 | `documents/race_cert_pixhawk_arming_failsafe.md` | ☐ |

---

## Onay

| Rol | Ad | Tarih | Not |
|---|---|---|---|
| Operatör | | | |
| Teknik sorumlu | | | |

**Yarış deploy:** B + D tamamlanmadan sim sign-off verilmez; E tamamlanmadan gerçek yarış deploy verilmez.
