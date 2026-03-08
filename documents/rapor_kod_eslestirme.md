# Rapor -> Kod Eslestirme ve Uyum Kaydi

Bu belge, `documents/rapor_calismasistemi.md` maddelerini kod/konfig karsiliklariyla
eslestirmek ve 2 asamali kontrol sonucunu tek yerde tutmak icin olusturulmustur.

## 1) Kritik Esik ve Parametre Eslestirme

| Rapor Maddesi | Beklenen | Kod Karsiligi | Durum |
|---|---|---|---|
| R_wp | 2.5 m | `docker_workspace/src/compliance_profile.py` -> `R_WP_M` | Uygun |
| T_hold | 2.0 s | `compliance_profile.py` -> `T_HOLD_S` | Uygun |
| D_min | 2.0 m | `compliance_profile.py` -> `D_MIN_M` | Uygun |
| P3 timeout | 180 s | `compliance_profile.py` -> `P3_TIMEOUT_S` | Uygun |
| P3 retry | 60 s, 1 kez | `compliance_profile.py` -> `P3_RETRY_S`, `P3_RETRY_COUNT` | Uygun |
| Fail-safe uyari | 5.0 s | `compliance_profile.py` -> `HEARTBEAT_WARN_S` | Uygun |
| Fail-safe tetik | 30.0 s | `compliance_profile.py` -> `HEARTBEAT_FAIL_S` | Uygun |
| Genel telemetri | 5 Hz | `telemetry.py` -> `_request_mavlink_streams` | Uygun |
| Engel/guvenlik | 5 Hz | `compliance_profile.py` -> `OBSTACLE_TELEMETRY_HZ` (API profil) | Uygun |
| Lidar isleme | 10 Hz sinifi | `compliance_profile.py` -> `LIDAR_PROCESSING_HZ` (API profil) | Uygun |

## 2) Uctan Uca Akis ve Emniyet Eslestirme

| Rapor Bolumu | Kod Karsiligi | Not |
|---|---|---|
| 1.3 HEALTH_CHECK + READY | `usv_main.py` -> `_refresh_health_check`, `start_mission` READY kapisi | Eksik bayraklar `ready_missing` ile yayinda |
| 1.6 Komut kisiti | `usv_main.py` + `telemetry.py` -> mission lock + race start kilidi | Mission sirasinda manuel gecis yok |
| 1.7 Parkurlar arasi otomatik gecis | `usv_main.py` -> `_wait_for_next_parkur` | Kullanici girdisi beklenmiyor |
| 1.10 gate_gecildi | `cam.py` + `usv_main.py` | Kamera olayi + 0.5s dogrulama |
| 1.12 Heartbeat fail-safe | `usv_main.py` -> `_update_watchdog` | 5s uyari, 30s hold |
| 1.13 E-stop guc kesme tetigi | `usv_main.py` + `telemetry.py` | RC7 force yolu korunuyor |
| 3.1 Link topolojisi | `compliance_profile.py` -> `LINK_TOPOLOGY` | API ve state uzerinden yayinlaniyor |
| 3.2-3.3 Haberlesme kisitlari | `compliance_profile.py` -> `COMMS_POLICY` | Race modu goruntu aktarimi kapali |
| 3.4 Telemetri veri gruplari | `telemetry.py` -> `/api/data/report_view` | Mod, gorev, olay, saglik, emniyet, enerji |

## 3) Kritik Uyumsuzluk Onceliklendirme (A2)

| Oncelik | Baslik | Durum |
|---|---|---|
| P0 | Emniyet: fail-safe, E-stop, komut kilidi | Kapatildi |
| P1 | Gorev akisi: otomatik parkur gecisi, READY kapisi | Kapatildi |
| P2 | Arayuz/API: manuel parkur endpointi, race gorunurluk kurallari | Kapatildi |

## 4) Kabul Kriteri ve Kanit Tipleri (A3)

- Kod kontrolu: sabit/esik/frekans ve endpoint kurallari.
- Davranis kontrolu: API dry-run + route davranisi + state ciktisi.
- Saha zorunlu kalemler: bu turda `DOĞRULA` etiketli beklemede.

## 5) Acik Riskler (K3)

| Risk | Etki | Sahip | Hedef |
|---|---|---|---|
| USB fiziksel yazma ortami race sahada bagli degilse READY bloklayabilir | Orta | Yazilim + Saha Operasyon | Yarisma oncesi saha testi |
| Video/saha kanit maddeleri koddan dogrulanamaz | Yuksek | Operasyon | Saha cekim gunu |

