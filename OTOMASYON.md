# CELEBILER USV Otomasyon Mimarisi

Bu belge, TEKNOFEST 2026 IDA sartnamesine uygun tek otomasyon mimarisini tanimlar. Esas ilke sim-first gelistirmedir: sim ve gercek sistem ayni state machine'i, ayni MAVLink yolunu, ayni mission/state semasini ve ayni emniyet kurallarini kullanir. Simulasyonda farkli olan kisim yalniz fiziksel donanimin yazilim kopruleriyle temsil edilmesidir.

Kaynak onceligi:

1. `documents/ida_sartname.md`
2. `documents/rapor_calismasistemi.md`
3. `host_scripts/` baslatma/dogrulama akislari
4. Bu belge

## 1. Mimari Roller

### Raspberry Pi

- Ust seviye otonomi merkezidir.
- `usv_main.py` ana state machine'ini calistirir.
- Kamera, lidar, STM32 seri telemetri, loglama ve gorev saglik durumunu toplar.
- Parkur-2 ve Parkur-3 icin `v_target` ve `heading_target` uretir.
- Yarista motor PWM'i uretmez ve ana otonomi yolu olarak `RC_CHANNELS_OVERRIDE` kullanmaz.
- Yarista Pixhawk'a yalniz MAVLink `GUIDED` hiz/yaw setpoint'i gonderir; dusuk seviye heading/PWM kontrolu Pixhawk'ta kalir.
- Yarista goruntu veya sensor islemeyi gemi disina tasimaz; tum algilama onboard calisir.
- Yerel kayit merkezidir: debug loglari, jsonl olaylari, kamera isleme kayitlari, lidar/cost-map ciktisi ve CSV telemetriyi saklar.

### Pixhawk / ArduPilot

- Dusuk seviye kontrol, arming, arac modu, GNSS/IMU durum kestirimi ve ESC/thruster cikisindan sorumludur.
- Heading/yaw kapali cevrim kontrolu, hiz takibi, mixer ve ESC PWM cikislari Pixhawk/ArduPilot sorumlulugudur.
- Parkur-1 yarista saf `AUTO` mission yurutucusudur.
- Parkur-2 ve Parkur-3'te `GUIDED` setpoint takip katmanidir.
- Yarista Pi fallback veya RC override ana otonomi yolu olarak kullanilamaz.
- `RC_CHANNELS_OVERRIDE` yalniz manuel emniyet/bench veya sim test fallback kapsamindadir; race modda otonomi icin kullanilmaz.

### Kamera

- Raspberry Pi Camera v3 veya simde `ros_to_tcp_cam.py` tarafindan ayni TCP JPEG arayuzuyle beslenir.
- `cam.py` onboard isleme yapar ve `camera_status.json` uretir.
- Yarista canli goruntu aktarimi kapali kalir; onboard isleme ve yerel kayit devam eder.

### Lidar

- Parkur-2 icin birincil guvenlik sensorudur.
- Sol/orta/sag sektor mesafeleri, minimum engel metrikleri ve lokal cost-map uretir.
- Gecersiz, bayat veya kalite disi veri gorev hattina dogrudan verilmez; once guard/filter uygulanir.

### STM32, DHT22 ve Yagmur Sensoru

- DHT22 ve yagmur sensoru STM32 uzerinden JSON seri veri olarak Raspberry Pi'ye aktarilir.
- `Env_Temp`, `Env_Hum`, `Rain_Val`, `Rain_Status` telemetri ve CSV kaydina girer.
- Bu sensorler kritik start gate degildir; saglik, kayit ve uyari amaciyla kullanilir.

### YKI

- Mission yukleme, arayuz ve telemetri izleme amaclidir.
- Otonomi, goruntu isleme veya sensor isleme YKI'ye tasinmaz.
- Yarista goruntu aktarimi acilmaz.

## 2. Haberlesme ve Mod Kurallari

- Yasak bantlarda yeni haberlesme yolu eklenmez: `2.4-2.8 GHz` ve `5.15-5.85 GHz`.
- Hucresel modem, hotspot veya Wi-Fi tabanli gorev/telemetri/goruntu aktarimi kullanilmaz.
- Mission yukleme ve telemetri Pixhawk/YKI MAVLink hattindan yurutulur.
- Arac ici Pi/Pixhawk haberlesmesi MAVLink uzerindendir.
- `test` modu dashboard/API dogrulama ve gelistirme icindir.
- `race` modu onboard odakli calisir; goruntu stream ve harita stream disari acilmaz.
- Simulasyon ayrimi `USV_SIM=1` ile yapilir; is mantigi degismez.

## 2.1 Kontrol Sahipligi: Heading, PWM ve Override Karari

Yaris icin tek karar:

- Pixhawk/ArduPilot PWM sahibidir.
- Raspberry Pi hedef hiz ve hedef heading/yaw uretir.
- Raspberry Pi bu hedefleri MAVLink `SET_POSITION_TARGET_GLOBAL_INT` ile `GUIDED` setpoint olarak gonderir.
- ArduPilot bu setpoint'leri kendi hiz/heading kontrolcusu, mixer ve servo cikislariyla ESC PWM'e cevirir.
- Otonomi akisi sirasinda Pi tarafindan motor PWM override yapilmaz.

Parkur bazinda kontrol sahipligi:

| Faz | Arac modu | Karar katmani | Dusuk seviye kontrol | PWM sahibi |
|---|---|---|---|---|
| P1 | `AUTO` | Pixhawk mission | ArduPilot | Pixhawk |
| P1 -> P2 bekleme | `AUTO` / emniyetli gecis | Pi saglik gate izler | ArduPilot | Pixhawk |
| P2 | `GUIDED` | Pi sensor fuzyon + waypoint/gate/kacinma | ArduPilot setpoint takip | Pixhawk |
| P3 | `GUIDED` | Pi hedef rengi + angajman karari | ArduPilot setpoint takip | Pixhawk |
| E-stop | herhangi | RC/guvenlik zinciri | guc kesme | fiziksel kontaktor |

Yaris modunda yasaklar:

- `USV_USE_RC_OVERRIDE=1` ile otonomi yurutmek yasaktir.
- `SIM_GZ_ALLOW_MOTOR_COMMAND_JSON=1` bench bypass'i race-like sim ve race modda yasaktir.
- GUIDED setpoint gonderimi basarisiz olursa Pi RC override fallback yapmaz; HOLD/failsafe uygulanir.
- Manuel RC stick hareketi otonomiden ustundur; ancak gorev basladiktan sonra kabul edilen operator komutu yalniz E-stop'tur.

Simulasyonda da ayni sahiplik korunur:

- Pi MAVLink setpoint gonderir.
- ArduPilot SITL servo/PWM cikisi uretir.
- `sitl_gazebo_bridge.py` bu servo cikisini Gazebo hareketine cevirir.
- `motor_command.json` yalniz debug/telemetri aynasidir; fizik motorunun ana motor kaynagi degildir.

Bu nedenle yarista "Pixhawk mi yapacak, Pi mi override edecek?" sorusunun cevabi:

- P1'i Pixhawk `AUTO` yapar.
- P2/P3 kararini Pi verir, ama motor PWM'ini yine Pixhawk uretir.
- Pi override etmez; Pi yalniz setpoint verir.

## 2.2 Mevcut Pixhawk Param Baseline

`config/pixhawk_ida.param` mevcut teknede manuel surusle calisan Pixhawk baseline dosyasidir. Bu dosya otomasyon tarafinda referans alinir, ancak yaris oncesi guvenlik ve AUTO/GUIDED davranisi ayrica sertifiye edilmeden "race-ready" sayilmaz.

Yaris sertifikasyon belgeleri:

- `documents/race_cert_pixhawk_arming_failsafe.md` — arming/failsafe karari (Yol A param profili veya Yol B fiziksel E-stop)
- `documents/servo_mapping_water_test_plan.md` — SERVO1/3 su ustu + sim yaw sign sign-off
- `python3 host_scripts/compare_pixhawk_param_baseline.py` — baseline vs ArduRover reference diff + risk tablosu
- `documents/manual_operator_verification_checklist.md` — operatör sim/saha doğrulama checklist

Mevcut donanim eslesmesi:

- [x] Arac frame baseline'i `FRAME_CLASS=2`, `FRAME_TYPE=0` olarak kabul edilir.
- [x] Motor cikislari `SERVO1_FUNCTION=74` ve `SERVO3_FUNCTION=73` olarak mevcut manuel surus konfigurasyonunu temsil eder.
- [x] ESC PWM araligi `SERVO1/3_MIN=1100`, `SERVO1/3_TRIM=1500`, `SERVO1/3_MAX=1900` degerleriyle proje PWM sozlesmesiyle uyumludur.
- [x] RC mode secimi `MODE_CH=8` uzerindedir; `RC5_OPTION=0` oldugu icin CH5 Pi tarafinda yaris start esigi olarak kullanilabilir.
- [x] `RC7_OPTION=0` oldugu icin CH7 Pi tarafinda E-stop girdisi olarak izlenebilir; ancak fiziksel guc kesme zinciri Pixhawk param dosyasindan bagimsiz olarak dogrulanmalidir.
- [ ] `SERVO1_FUNCTION=74` / `SERVO3_FUNCTION=73` yon, sol-sag motor eslesmesi ve terslikleri su ustu testinde ileri, geri ve yerinde donus icin tekrar dogrulanir.

Yaris otomasyonu icin param riski:

- [ ] `ARMING_CHECK=0` ve `ARMING_REQUIRE=0` mevcut manuel-test kolayligi olarak kabul edilir; yaris oncesi ya guvenli pre-arm/arming politikasi etkinlestirilir ya da fiziksel guc kesme ve operator proseduruyle bilincli sekilde sertifiye edilir.
- [ ] `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=0`, `BATT_FS_LOW_ACT=0` ve `BATT_FS_CRT_ACT=0` Pixhawk native failsafe'lerinin kapali oldugunu gosterir; Pi watchdog, E-stop ve fiziksel kontaktor zinciri bu boslugu kapatacak sekilde saha testinde kanitlanir.
- [ ] `CRUISE_SPEED=2` ve `WP_SPEED=2` P1 `AUTO` hizini yaklasik 2 m/s seviyesine tasir; P1 icin hedeflenen yaris hizi mission speed komutlari veya param profiliyle netlestirilmeden gercek parkurda kullanilmaz.
- [ ] `WP_RADIUS=2` ve `TURN_RADIUS=0.9`, 1.60 m x 0.81 m govde icin zigzag donuslerinde su ustu overshoot testiyle dogrulanir.
- [ ] Pixhawk param baseline'i degistirilirse `OTOMASYON.md`, `AGENTS.md`, `GEMINI.md` ve sim SITL param profili ayni kontrol sahipligiyle guncellenir.

## 3. Gorev Baslatma ve Komut Kilidi

- Gorev otomatik baslamaz.
- Test modunda start yalniz dashboard Start butonu veya `POST /api/start_mission` ile verilir.
- Yarista start yalniz RC CH5 esigiyle kabul edilir; API start kapali kalir.
- Start on kosullari:
  - `ready_state=true`
  - `camera_ready=true`
  - `lidar_ready=true`
  - E-stop hattinin guvenli durumda olmasi
  - Mission ve hedef profilinin start oncesi dogrulanmis olmasi
- Gorev basladiktan sonra mission reload, retarget ve yeni operator komutlari reddedilir.
- Gorev sirasinda kabul edilen tek operator komutu E-stop'tur.
- Parkurlar arasi gecis kullanici girdisi olmadan otomatik yapilir.

## 4. Mission ve Parkur Profili

Yarista Mission Planner/Pixhawk mission listesi ana waypoint kaynagidir. Raspberry Pi start oncesi Pixhawk mission mirror ile waypoint listesini alir ve mission lifecycle alanlarina yazar. Race start icin beklenen kaynak `pixhawk_mission` olmalidir; farkli kaynaklar yarista reddedilmelidir.

Waypoint listesi tek basina parkur ayrimini garanti etmez. Bu nedenle start oncesi kilitlenen bir `mission_profile` sozlesmesi zorunludur:

- Parkur-1 zigzag waypoint araligi
- Parkur-2 engelli waypoint araligi
- Parkur-3 angajman waypoint'i
- Parkur-3 hedef rengi
- Mission upload kaynagi ve validation timestamp'i
- Profile schema version

`mission_profile` gorev baslamadan once dogrulanir ve kilitlenir. Gorev basladiktan sonra profil, hedef renk veya waypoint semasi degistirilemez.

Mevcut uyumluluk notu:

- Duz JSON waypoint formati `[[lat, lon], ...]` degisken uzunluklu NAV gorevini destekler.
- Duz format, Parkur-3 angajman noktasini tek basina ayirt edemez.
- Kamikaze angajman icin `mission_profile`, yapilandirilmis mission yukleme veya acik `engage_wp` sozlesmesi gerekir.
- Bu ayrim olmadan sistem Parkur-3 hedef rengini kilitlese bile angajman parkuru eksik kalir.

## 5. Parkur State Machine

### Parkur-1: Zigzag Nokta Takibi

- Yarista Pixhawk `AUTO` modda saf mission yurutur.
- Raspberry Pi direksiyon veya motor cikisina mudahale etmez.
- Pi yalniz saglik, log, mission ilerleme, guidance source ve fail-safe izlemesi yapar.
- P1'de kamera/lidar direksiyon karari uretmez.
- P1 tamamlaninca P2 hazirlik kontrolu yapilir.

### Parkur-1 -> Parkur-2 Gecisi

- Kamera ve lidar hazir degilse Parkur-2 gecisi bekletilir.
- Hazirlik beklerken arac emniyetli hiz siniri ve mevcut mission/hold davranisi icinde kalir.
- Hazirlik saglandiginda `GUIDED` mod setpoint yurutmeye gecilir.

### Parkur-2: Engelden Kacmali Waypoint Takibi

- Pi her kontrol dongusunde `v_target` ve `heading_target` uretir.
- Lidar birincil guvenlik sensorudur:
  - [x] Orta sektor engelinde kacinma zorunludur.
  - Sol/sag sektorler guvenli sapma secimi icin kullanilir.
  - Gecersiz lidar verisi algilanirsa gorev hatti degraded/hold davranisina gecer.
- Kamera turuncu sinir/kapi ve sari engel algisiyla yonelimi destekler.
- Kamera kapisi varsa kapi orta hatti tercih edilir.
- Tehdit varsa lidar kacinmasi kamera hedeflemesinden onceliklidir.
- En az iki kapi/engel gecisi ve son P2 gorev noktasina ulasma Parkur-2 kabul kriteridir.

### Parkur-2 -> Parkur-3 Gecisi

- Kullanici komutu beklenmez.
- P2 tamamlaninca state machine Parkur-3 angajman fazina gecer.
- Hedef renk start oncesi kilitli degilse Parkur-3 baslatilmaz; sistem HOLD/failsafe semantigine uygun pasif bekler.

### Parkur-3: Kamikaze Angajman

- Hedef renk start oncesi `target_state.json`, `mission_profile` veya gorev yukleme ekrani ile kilitlenir.
- Gorev basladiktan sonra hedef rengi degistirilemez.
- Kamera HSV isleme yalniz kilitli hedef rengini takip eder.
- [x] Yanlis hedef renkleri temas riski olusturdugunda kacinilacak nesne gibi ele alinir.
- `heading_target`, hedef merkez sapmasina gore guncellenir.
- `v_target`, hedef yakinlastikca kontrollu dusurulur.
- Bu teknede temas sensoru yoktur; P3 tamam karari temas sensorune dayanamaz.
- Temas sensoru olmadigi icin yazilim angajman tamam kararini dolayli kaynaklarla verir ve karar kaynagini loglar.
- Temas dogrulama su kaynaklarin birlesimiyle yapilir:
  - [x] hedef yakinligi (`gps_proximity` + `lidar_proximity`)
  - [x] goruntu alan orani (`vision_area`)
  - [x] ani yavaslama / hiz surekliligi (`lidar_collision`: v_target>0 + pos_change~0 + lidar_center<1.0m)
  - [x] temas sensoru yok; `p3_contact_confirmation_source` alaniyla dolayli karar kaynagi loglanir
- Yanlis hedef temas riski aktifken angajman tamam latch'i kurulmaz; once kacinma uygulanir.
- Basarisiz angajman timeout ve tek yeniden deneme sonrasi geri cekilme ve HOLD ile sonlanir.

## 6. Sensor Fuzyon Onceligi

Karar onceligi:

1. E-stop, RC override ve fiziksel emniyet zinciri
2. Health gate ve sensor freshness
3. Waypoint veya angajman hedefi ana navigasyon referansi
4. Yakin, taze ve dogrulanmis lidar/camera tehdidi varsa bounded avoidance duzeltmesi
5. Kamera kapi/sinir/engel/hedef algisi yalniz gorev referansini destekleyen sinirli bias
6. Hiz profili ve yumusaklama

Waypoint bacagi yeni degistiginde veya buyuk heading reacquire gerekiyorsa arac once hedef bearing'e yonelir. Bu pencerede map residue, tek frame kamera sinir/hedef algisi veya uzak engel waypoint yonelimini bastiramaz. Avoidance yalniz yakin ve dogrulanmis tehditte devreye girer; devreye girdiginde bile heading komutu sinirli duzeltme olarak kalir ve ters yone kilitlenme davranisi kabul edilmez.

Kamera cikti sozlesmesi (`camera_status.json`):

- gate algisi ve gate bearing
- sari engel algisi, bearing ve alan
- turuncu sinir/kapi algisi, bearing ve alan
- hedef renk algisi, bearing ve alan
- [x] yanlis hedef renk algisi, bearing, alan ve hedef sinifi
- pipeline freshness ve isleme metrikleri

Lidar cikti sozlesmesi:

- sol/orta/sag sektor mesafeleri
- minimum engel mesafesi
- lokal cost-map / engel haritasi
- kalite, freshness ve drop nedeni

## 7. Emniyet ve Fail-safe

- Manuel/RC override otonomiden daima onceliklidir.
- E-stop zinciri motor sinyalini degil, fiziksel guc kesme mantigini temsil eder.
- E-stop, role, RC override ve mission state semasi zayiflatilamaz.
- Telemetri heartbeat kaybinda once uyari, sonra hiz dusurme ve HOLD uygulanir.
- GPS, kamera veya lidar timeout olaylari INFO/WARN/ERROR seviyesinde acik loglanir.
- Parkur disi veya carpma riski algisinda hiz dusurme, guvenli sapma veya pasif bekleme uygulanir.
- Race modda Pi tarafindan RC override veya sim motor bypass bayraklari kullanilamaz.

## 8. Kayit ve Teslim Ciktilari

Gorev boyunca yerel dosya sisteminde kayit tutulur. Loglar sartnameye aykiri goruntu aktarimi icin kullanilamaz.

Zorunlu cikti gruplari:

- Kamera isleme kaydi: zaman etiketli, en az 1 Hz, hedef/engel/kapi overlay bilgisiyle.
- Lidar/cost-map kaydi: zaman etiketli, en az 1 Hz.
- Telemetri CSV: konum, hiz, yonelim, hiz setpoint, heading setpoint, PWM/cikis bilgileri ve cevresel sensor alanlari.
- Sistem loglari: `{component}.debug.log` ve `{component}.jsonl`.

Loglanacak kritik olaylar:

- mode degisimi
- mission load/start/stop
- mission lifecycle ve upload source
- target color lock
- RC override
- E-stop
- GPS fix kaybi
- kamera/lidar timeout
- failsafe gecisi
- waypoint ve gate gecisi
- angajman tamam/timeout

## 9. Simulasyon Sozlesmesi

Simulasyon, gercek sistemin yazilim davranisini temsil eder:

- ArduPilot SITL gercek Pixhawk MAVLink yolunu temsil eder.
- `sitl_gazebo_bridge.py`, SITL MAVLink ile Gazebo fizik/sensor durumunu baglar.
- `ros_to_tcp_cam.py`, ROS2 kamera topic'ini `cam.py` ile ayni TCP JPEG arayuzune cevirir.
- `usv_main.py` ayni state machine ile calisir.
- `USV_SIM=1` yalniz donanim yoklugu icin bypass saglar; gorev mantigi degismez.

Sim-to-real kabul kurali:

- Gazebo world, parkur ve tekne modeli gercek yarismayi temsil eden birincil dogrulama ortami olmalidir.
- Parkur geometrisi, duba renkleri, duba boyutlari, hedef yerlesimi, su alani, kamera acisi, lidar yerlesimi, tekne boyutu, agirlik merkezi, itki yonleri ve motor dinamikleri gercek tekneye karsilik gelecek sekilde modellenmelidir.
- Simulasyonda basarili olan gorev akisi, ayni repo kodlariyla gercek teknede ek is mantigi degisikligi gerektirmeden calismalidir.
- Sim ve gercek ayrimi yalniz donanim adaptoru seviyesinde kalir: seri port, kamera kaynagi, lidar kaynagi, USB depolama ve SITL/Pixhawk endpoint secimi.
- Otonomi karar kodunda `USV_SIM=1` icin parkur, hedef, kacinma, state transition veya fail-safe davranisini degistiren kisayol bulunamaz.
- Gazebo tarafindaki her parkur/world/tekne degisikligi, gercek sistem davranisini daha iyi temsil etmek icin yapilir; simde kolaylastirici fizik, duba veya sensor avantajlari kabul edilmez.
- Gercek tekneye geciste izin verilen ayarlar yalniz ortam/cihaz baglantisi ve kalibrasyon parametreleridir; mission state, MAVLink protokolu, hedef renk kilidi, post-start lock ve emniyet zinciri ayni kalir.

### Gercek Tekne ve Ultra Realistik Hareket Sozlesmesi

Gercek tekne olculeri:

- Boy: yaklasik `160 cm`
- En: yaklasik `81 cm`
- Su ustunde kalan yukseklik: yaklasik `35 cm`

Simulasyon model hedefi:

- Gazebo USV govdesi bu olculeri temsil etmelidir: `1.60-1.62 m x 0.81 m x 0.35 m`.
- Kütle, atalet, suya batma orani, lineer damping ve açisal damping gercek havuz/deniz testinden kalibre edilmelidir.
- Donus mekanigi iki thruster farkli itki mantigiyla olmalidir; simde dogrudan pose set etmek veya tekneyi yapay dondurmek yasaktir.
- Heading davranisi Pixhawk/ArduPilot kapali cevriminden gelmelidir; Pi yalniz heading/yaw hedefi verir.
- Sim bridge fizik kaynagi olarak ArduPilot SITL servo/PWM cikisini kullanmalidir; `motor_command.json` fizik icin kullanilmamalidir.
- Simde yaw sign, sol/sag motor tersligi, maksimum hiz ve maksimum yaw rate gercek tekne testleriyle kalibre edilir.

Kalibrasyon kabul metrikleri:

- Neutral PWM'de tekne ileri suruklenmemeli ve surekli donmemelidir.
- Sim ve gercek teknede ayni heading komutu ayni yone donmelidir.
- Sol/sag motor farki ile donus yoni ters olmamalidir.
- 1.0 m/s duz seyirde yaw salinimi kabul edilebilir kucuk bantta kalmalidir.
- 90 derece donus komutunda overshoot ve donus suresi gercek tekneye yakin olmalidir.
- P2 dar gecislerde tekne eni `0.81 m` hesaba katilarak lidar guvenlik mesafesi korunmalidir.

Bu kalibrasyon tamamlanmadan otomasyon mantigi dogru olsa bile yaris surusu guvenilir sayilmaz.

Sim senaryosu:

- P1: zigzag waypoint sirasi
- P2: sari engeller ve turuncu kapi/sinir dubalari
- P3: uc farkli renkte hedef duba, start oncesi kilitli hedef renk

Manuel surus, motion dogrulama ve mission progression testi operator tarafindan yapilir. Ajanlar otomatik surus testi veya mission baslatan `run_sim_stack.sh` turevlerini kendi kendine calistirmaz.

## 10. Test ve Dogrulama

Her kod degisikliginden sonra en az su dogrulamalar calistirilir:

```bash
python3 -m py_compile docker_workspace/src/*.py
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
python3 host_scripts/check_compliance_race.py
```

Log kontrol listesi:

- `logs/terminal.log`
- `logs/system/usv_main.debug.log`
- `logs/system/cam.jsonl`
- `logs/system/lidar_map.jsonl`
- `logs/system/telemetri_verisi.csv`
- gerekiyorsa `logs/simulation/*`

Kabul kriterleri:

- [x] P1 zigzag waypointleri saf Pixhawk `AUTO` ile tamamlanir.
- [ ] P2'de en az iki kapi/engel gecisi guvenli ilerler.
- [x] P3'te yalniz start oncesi secilen hedef renge angajman yapilir.
- [x] Yanlis hedef renklerine temas riski engel gibi ele alinir.
- [x] Race modda goruntu veya lidar harita stream'i disari acilmaz.
- [x] Start sonrasi mission reload ve retarget reddedilir.
- [x] Uyum betikleri basarili sonuc verir.
- [x] Race modda PWM sahipligi Pixhawk/ArduPilot katmanindadir; Pi RC override ana otonomi yolu degildir.
- [ ] Gercek tekne olculeriyle sim hareket/heading/PWM kalibrasyonu manuel su testiyle dogrulanir.
- [ ] WP1 -> WP2 gibi waypoint gecislerinde `target_bearing`, `heading_error`, `SET_POSITION_TARGET_GLOBAL_INT`, `SERVO_OUTPUT_RAW`, `yaw_cmd_norm` ve `observed_yaw_rate_dps` ayni zaman penceresinde tutarli isaret verir.

## 11. Acik Uyum Riskleri

- [ ] Duz waypoint listesi, P3 angajman noktasini ve parkur araliklarini tek basina ifade etmez.
- [ ] `mission_profile` veya yapilandirilmis mission sozlesmesi olmadan kamikaze parkuru eksik kalir.
- [x] `mission_upload_source=pixhawk_mission` yarista zorunlu hale getirilmezse Mission Planner/Pixhawk ana kaynak ilkesi zayiflar.
- [x] Hedef temas sensoru mevcut degilse P3 temas dogrulamasi kamera alani, yakinlik ve ani yavaslama gibi dolayli metriklere dayanir; bu karar loglarda acikca izlenebilir olmalidir.
- [ ] Temas sensoru olmadigi icin P3 yazilim "tamam" karari fiziksel temas yerine dolayli kanitlara dayanir; saha testinde hedefe gercek temas kamerayla ve telemetriyle manuel dogrulanmalidir.
- [ ] Gercek tekne kütlesi, trim, itici araligi, motor tersligi ve ArduPilot servo fonksiyonlari simle birebir kalibre edilmezse heading/PWM davranisi yaris sirasinda sapabilir.
- [ ] `SERVO1_FUNCTION=74` / `SERVO3_FUNCTION=73`, `SIM_GZ_SERVO1_MOTOR`, `SIM_GZ_SERVO3_MOTOR` ve `SIM_GZ_YAW_SIGN` su ustu ve sim manuel hareket testiyle ayni yonde dogrulanmadan ters donus riski tamamen kapanmis sayilmaz.
