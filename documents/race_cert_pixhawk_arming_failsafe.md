# Pixhawk Arming / Failsafe — Yarış Sertifikasyonu

Bu belge `config/pixhawk_ida.param` baseline'ındaki **bilinçli güvenlik sapmalarını** yarış deploy öncesi kapatır veya fiziksel/operasyonel telafi ile sertifiye eder.

**Baseline referans:** `config/pixhawk_ida.param`  
**Karşılaştırma aracı:** `python3 host_scripts/compare_pixhawk_param_baseline.py`  
**İlgili su testi:** `documents/servo_mapping_water_test_plan.md`

---

## 1. Kapsam

| Katman | Sorumluluk |
|---|---|
| Pixhawk / ArduPilot | Arming, native failsafe, ESC PWM, P1 `AUTO` |
| Raspberry Pi | P2/P3 karar, GUIDED setpoint, E-stop latch, mission watchdog |
| Fiziksel zincir | Kontaktör / güç kesme, RC E-stop, operatör prosedürü |

Yarış modunda Pi **RC override ana otonomi yolu değildir**; Pixhawk PWM üretir.

---

## 2. Mevcut baseline sapmaları (manuel sürüş profili)

Aşağıdaki değerler **manuel test kolaylığı** içindir; yarışa taşınmadan önce karar verilmelidir.

| Param | Baseline | Tipik ArduRover default | Risk |
|---|---:|---:|---|
| `ARMING_CHECK` | 0 | 1 | HIGH — pre-arm kontrolleri kapalı |
| `ARMING_REQUIRE` | 0 | 1 | HIGH — arm sequence zorunlu değil |
| `FS_THR_ENABLE` | 0 | 1 | HIGH — RC failsafe kapalı |
| `FS_GCS_ENABLE` | 0 | 1 | MEDIUM — GCS heartbeat failsafe kapalı |
| `BATT_FS_LOW_ACT` | 0 | 1 | HIGH — düşük batarya aksiyonu yok |
| `BATT_FS_CRT_ACT` | 0 | 2 | HIGH — kritik batarya aksiyonu yok |

---

## 3. Yarış için iki onaylı yol

Operatör **A veya B** yolundan birini seçer; ikisi birden zorunlu değildir.

### Yol A — Pixhawk pre-arm / failsafe profili (tercih edilen)

1. Yarış öncesi param profili yükle veya mevcut dosyada en az şunları gözden geçir:
   - `ARMING_CHECK=1`
   - `ARMING_REQUIRE=1`
   - `FS_THR_ENABLE=1` (RC link varsa)
   - `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` uygun HOLD/RTL aksiyonuna alınır
2. Mission Planner'dan yeni dump al: `config/pixhawk_ida_race.param` (baseline'ı ezmeden ayrı dosya).
3. Su üstü arm/disarm + RC kesinti + düşük batarya simülasyonu test et.
4. Bu belgedeki imza tablosunu doldur.

### Yol B — Fiziksel E-stop / kontaktör sertifikasyonu (baseline korunur)

Baseline `ARMING_CHECK=0` / `ARMING_REQUIRE=0` kalabilir **yalnızca** aşağıdakiler saha testiyle kanıtlanırsa:

1. **Fiziksel E-stop** motor gücünü ≤ 1 kontrol döngüsü + kontaktör gecikmesi içinde keser.
2. **RC CH7 ≥ 1900** veya dashboard E-stop → Pi `HOLD` + güvenli çıkış; Pixhawk neutral PWM.
3. **Pi watchdog / link loss** → `HOLD`, setpoint durur, operatör müdahalesi prosedürü yazılı.
4. Batarya düşük seviye operatör tarafından izlenir; otomatik Pixhawk aksiyonu yoksa **manual abort** prosedürü eğitilmiş olmalı.
5. `host_scripts/compare_pixhawk_param_baseline.py --strict` çıktısı incelenir; HIGH risk maddeler bilinçli olarak kabul edilir.

---

## 4. Pi tarafı zorunlu davranış (her iki yol)

- Yarış modunda görev **RC CH5 ≥ 1700** veya operatör prosedürü dışında başlamaz.
- E-stop latch sonrası mission reload / retarget kapalı (`409`).
- GUIDED setpoint başarısız → Pi RC override **yapmaz**; `HOLD`/failsafe.
- `USV_USE_RC_OVERRIDE=1` yarışta **yasak**.

---

## 5. Sertifikasyon kontrol listesi

| # | Kriter | Yol A | Yol B | Kanıt |
|---:|---|---|---|---|
| 1 | Param baseline karşılaştırması çalıştırıldı | ☐ | ☐ | `logs/system/compliance/pixhawk_param_baseline.json` |
| 2 | Arming/failsafe kararı yazılı | ☐ | ☐ | Bu belge §3 |
| 3 | Fiziksel E-stop zinciri test edildi | ☐ | ☐ | Test log / foto |
| 4 | RC kaybı / HOLD davranışı doğrulandı | ☐ | ☐ | `usv_main.jsonl` |
| 5 | Servo mapping su testi imzalandı | ☐ | ☐ | `servo_mapping_water_test_plan.md` |
| 6 | Sim SITL profili baseline ile uyumlu | ☐ | ☐ | `sim/configs/sitl_sim_gps.parm` diff |

---

## 6. İmza

| Rol | Ad | Tarih | Yol (A/B) | Not |
|---|---|---|---|---|
| Operatör | | | | |
| Teknik sorumlu | | | | |

**Yarış deploy onayı:** Yukarıdaki tablo + su testi sign-off tamamlanmadan `mode=race` saha deploy yapılmaz.
