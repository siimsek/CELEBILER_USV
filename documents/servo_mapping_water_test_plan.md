# Servo Mapping — Su Üstü Test Planı

Bu plan `SERVO1` / `SERVO3` eşleşmesi, ileri/geri yön, yaw işareti ve sim-real uyumunu **param baseline değiştirilmeden önce** doğrular.

**Baseline:** `config/pixhawk_ida.param`  
**Beklenen eşleşme:** `SERVO1_FUNCTION=74` (sol throttle), `SERVO3_FUNCTION=73` (sağ throttle)  
**PWM sözleşmesi:** min 1100, trim 1500, max 1900  
**Sim köprü env:** `SIM_GZ_SERVO1_MOTOR`, `SIM_GZ_SERVO3_MOTOR`, `SIM_GZ_YAW_SIGN`, `SIM_GZ_PWM_*_US`

Param baseline'a işlenmeden yarışa geçilmez.

---

## 1. Ön koşullar

- [ ] Tekne sakin su / emniyetli alan, can yeleği, kısa menzil operatör
- [ ] RC manual mod; propeller çıkarılmış **dry-run** veya düşük güç alanı
- [ ] Pixhawk + Pi stack çalışır; telemetri `SERVO_OUTPUT_RAW` veya dashboard motor PWM görünür
- [ ] Fiziksel E-stop hazır
- [ ] Mevcut param dump alındı: `config/pixhawk_ida.param`

---

## 2. Test matrisi

Her satır için **beklenen hareket** ve **gözlemlenen** doldurulur.

| ID | Komut | Beklenen hareket | SERVO1 PWM eğilimi | SERVO3 PWM eğilimi | Pass |
|---|---|---|---|---|---|
| T1 | RC ileri tam | Düz ileri | ↑ | ↑ | ☐ |
| T2 | RC geri tam | Düz geri (veya neutral if no reverse) | ↓ | ↓ | ☐ |
| T3 | RC sağ yaw | Burnu sağa (saat yönü) | sol↑ sağ↓ veya skid eşdeğeri | | ☐ |
| T4 | RC sol yaw | Burnu sola | sol↓ sağ↑ veya skid eşdeğeri | | ☐ |
| T5 | Neutral bırak | Durma / trim | ~1500 | ~1500 | ☐ |
| T6 | Pi GUIDED ileri (test mod) | Düz ileri, Pi setpoint | log `guidance_source` | | ☐ |
| T7 | Pi GUIDED yaw step | Kontrollü dönüş, overshoot notu | heading log | | ☐ |

**Fail kriteri:** T1–T5'ten biri ters ise `SERVO*_REVERSED`, function swap veya kablo düzeltmesi yap; **param dosyasını güncellemeden** önce tekrar koş.

---

## 3. Sim-real hizalama (bench + sim)

Sim stack açıkken (`USV_SIM=1`):

1. `sitl_gazebo_bridge.jsonl` içinde `sim_motor_contract` olayını kontrol et:
   - `servo1_motor=left`, `servo3_motor=right` (veya su testi sonucu)
   - `yaw_sign` komut dönüşü ile Gazebo heading aynı yönde artıyor mu?
2. Manuel sim sürüş: ileri komut → `vehicle_position.json` heading değişimi T3/T4 ile tutarlı mı?
3. Gerekirse yalnızca **env** override (param değil):
   - `SIM_GZ_YAW_SIGN=-1` veya `1`
   - `SIM_GZ_LEFT_MOTOR_REVERSED` / `SIM_GZ_RIGHT_MOTOR_REVERSED`
   - `SIM_GZ_SERVO1_MOTOR` / `SIM_GZ_SERVO3_MOTOR` swap

Su testi sonucu sim env ile eşleşene kadar sim motion doğrulaması **yarış sertifikası sayılmaz**.

---

## 4. Log toplama

| Kaynak | Dosya / olay |
|---|---|
| Pixhawk PWM | `telemetry.jsonl`, `SERVO_OUTPUT_RAW` |
| Pi guidance | `usv_main.jsonl` — `nav_track`, `guidance_source` |
| Sim bridge | `sitl_gazebo_bridge.jsonl` — `sim_motor_contract` |
| Oscillation (opsiyonel) | `python3 host_scripts/analyze_heading_oscillation.py` |

---

## 5. Param güncelleme kapısı

Aşağıdakilerden biri değişecekse **bu plan yeniden imzalanır**:

- `SERVO1_FUNCTION`, `SERVO3_FUNCTION`
- `SERVO*_REVERSED`, `SERVO*_MIN/TRIM/MAX`
- `FRAME_CLASS`, `FRAME_TYPE`
- `SIM_GZ_*` motor/yaw env seti

Güncelleme sonrası:

```bash
python3 host_scripts/compare_pixhawk_param_baseline.py --strict
python3 host_scripts/check_compliance_static.py
```

---

## 6. Sign-off

| Alan | Değer |
|---|---|
| Tarih | |
| Su / sim lokasyon | |
| T1–T7 sonuç | Pass / Fail |
| Yaw sign (sim env) | `SIM_GZ_YAW_SIGN=` |
| Servo1 motor | left / right |
| Servo3 motor | left / right |
| Notlar | |

| Rol | Ad | İmza |
|---|---|---|
| Operatör | | |
| Teknik sorumlu | | |

**Onay:** Tüm zorunlu satırlar Pass + sim env kaydı tamamlanmadan param baseline yarış profiline taşınmaz.
