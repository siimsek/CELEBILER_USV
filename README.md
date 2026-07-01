# CELEBILER USV - TEKNOFEST 2026 IDA

Bu depo, TEKNOFEST 2026 Insansiz Deniz Araci yarismasi icin gelistirilen otonomi, telemetri, simulasyon ve saha calisma altyapisini icerir.

README dosyasi proje hakkinda genel bilgi vermek icindir. Kod dosyasi, endpoint, port, algoritma, tuning, PWM veya dusuk seviye kontrol detaylari burada tutulmaz. Guncel teknik kurallar ve operasyon sozlesmesi icin proje dokumanlari ve dogrulama betikleri esas alinir.

---

## 1. Genel Sistem

Sistem; tekne uzerindeki otopilot, otonomi bilgisayari, kamera, lidar, RC kontrol, telemetri baglantisi, yer kontrol istasyonu ve simulasyon altyapisindan olusur.

Temel yaklasim:

- Gelistirme once simulasyonda dogrulanir.
- Simulasyon ve gercek sistem ayni ana gorev mantigini kullanir.
- Pixhawk/ArduPilot dusuk seviye arac kontrolunden sorumludur.
- Raspberry Pi ust seviye gorev, algilama, kayit ve telemetri islerini yurutur.
- Manuel kontrol ve acil durdurma otonomiden onceliklidir.
- Gorev operator komutu olmadan otomatik baslamaz.

---

## 2. Kullanilan Ana Parcalar

| Parca | Genel Amac |
|---|---|
| Pixhawk | Otopilot, arac modu, GNSS/IMU bilgisi ve motor cikislari |
| Raspberry Pi | Otonomi, sensor isleme, kayit ve telemetri yazilimlari |
| Kamera | Duba, kapi, hedef ve renk algilama |
| Lidar | Yakin cevre, mesafe ve engel bilgisi |
| STM32 | Yardimci sensor veya cevresel donanim okumalari |
| Crossfire RC | Manuel surus, yarista start tetigi ve uzaktan E-stop |
| 433 MHz telemetri | Mission yukleme ve MAVLink telemetri |
| YKI bilgisayari | Gorev hazirlama, izleme ve operator arayuzu |
| ESC ve motorlar | Teknenin itki sistemi |
| Fiziksel E-stop/kontaktor | Motor gucunu guvenli sekilde kesme |

---

## 3. Yazilim Gorev Gruplari

Yazilim yapisi belirli dosya adlariyla tarif edilmez. Sistem genel olarak su gorev gruplarindan olusur:

| Gorev Grubu | Genel Amac |
|---|---|
| Ana gorev yonetimi | Hazirlik, start/stop, parkur gecisleri ve gorev durumunu takip eder |
| Pixhawk/MAVLink baglantisi | Otopilot ile mod, konum, gorev, telemetri ve hedef haberlesmesini saglar |
| Kamera isleme | Goruntu uzerinden yaris nesneleri ve hedef bilgisi uretir |
| Lidar isleme | Yakin cevre ve engel bilgisini arac uzerinde uretir |
| Telemetri ve arayuz | Operator ekranina durum, saglik ve kayit bilgilerini sunar |
| Mission isleme | Yuklenen gorev bilgisini sistemin ortak gorev yapisina cevirir |
| Kayit ve loglama | Gorev olaylari, sensor durumu, telemetri ve hata bilgilerini saklar |
| Simulasyon kopruleri | Gercek donanim yerine kullanilan simulasyon parcalarini ayni akisla baglar |

Bu bolum mimariyi genel seviyede anlatir. Dosya isimleri, sinif yapilari, servis ayrimlari ve algoritmalar gelistirme sirasinda degisebilir.

---

## 4. Gorev Akisi

Genel gorev akisi:

1. Sistem acilir.
2. Pixhawk, Raspberry Pi ve sensorler hazirlanir.
3. Kamera, lidar, RC, telemetri ve E-stop saglik kontrolleri yapilir.
4. Mission YKI veya uygun gorev araci ile hazirlanir.
5. Mission Pixhawk/sistem tarafina yuklenir.
6. Sistem hazir durumdayken operator gorevi baslatir.
7. Parkur1, Parkur2 ve Parkur3 sirayla yurutulur.
8. Parkurlar arasi gecisler otonom olarak yapilir.
9. Gorev tamamlandiginda arac guvenli bekleme durumuna alinir.
10. Geri kazanim ve saha islemleri manuel yapilir.

Gorev basladiktan sonra acil durdurma haric yeni operator komutlari kabul edilmez.

---

## 5. Parkur Ozeti

### Parkur1

Waypoint tabanli seyir bolumudur. Yarista bu bolumde Pixhawk gorev yurutmesi ana roldedir. Raspberry Pi saglik, log ve gorev ilerlemesini izler.

### Parkur2

Kamera ve lidar bilgilerinin kullanildigi bolumdur. Sistem ust seviye hedef bilgisi uretir; arac kontrolu otopilot tarafinda uygulanir.

### Parkur3

Hedef rengi ve hedef konumu uzerinden angajmanin degerlendirildigi bolumdur. Yanlis hedef algisi emniyet amaciyla gorev durumuna yansitilir.

---

## 6. Haberlesme ve Kisitlar

Sistemde haberlesme kanallari gorevlerine gore ayrilir:

- RC linki manuel kontrol, start ve E-stop icin kullanilir.
- 433 MHz telemetri mission yukleme ve MAVLink izleme icin kullanilir.
- Pixhawk ile Raspberry Pi arasinda arac ici MAVLink haberlesmesi bulunur.
- Yardimci mikrodenetleyici ve sensorler kablolu baglantilarla sisteme dahil edilir.

Yaris seviyesinde temel kisitlar:

- 2.4 GHz ve 5 GHz tabanli yeni gorev/telemetri/goruntu aktarimi eklenmez.
- Hucresel modem, hotspot veya Wi-Fi tabanli gorev aktarimi kullanilmaz.
- Goruntu isleme ve sensor isleme arac uzerinde kalir.
- YKI yalniz arayuz, mission yukleme ve telemetri amaciyla kullanilir.
- Yaris modunda goruntu aktarimi acilmaz.

---

## 7. Emniyet ve Kontrol Sahipligi

Kontrol sahipligi genel olarak su sekildedir:

- Pixhawk/ArduPilot:
  - Dusuk seviye arac kontrolu
  - Arac modu
  - Motor/ESC cikislari
  - Manuel RC onceligi
  - Otopilot guvenlik davranislari

- Raspberry Pi:
  - Gorev durumu
  - Kamera/lidar isleme
  - Ust seviye hedef uretimi
  - Telemetri, kayit ve saglik izleme

Manuel kontrol ve E-stop her zaman otonomiden onceliklidir. Fiziksel guc kesme zinciri yazilimsal durdurma mantigiyla zayiflatilmaz.

PWM araligi: `PWM_NEUTRAL_US=1500`, min 1100, max 1900. RC override ana otonomi yolu degildir.

Pixhawk param baseline: `config/pixhawk_ida.param` mevcut manuel surus referansidir. Bu baseline'da `ARMING_CHECK=0`, `ARMING_REQUIRE=0` ve bazi Pixhawk native failsafe'leri kapali olabilir; yaris/otonom su testi oncesi fiziksel E-stop/kontaktor zinciri veya Pixhawk pre-arm/failsafe profiliyle dogrulanir. `SERVO1/3` motor eslesmesi ve `1100/1500/1900` PWM araligi su ustu test ile onaylanmadan degistirilmez.

Yaris konfigurasyonu: Race modunda `USV_RACE_P1_AUTO_WAYPOINTS` ortam degiskeni ile P1 parkurundaki kac waypoint'in Pixhawk AUTO'da calistirilacagi belirtilmelidir. Varsayilan 1'dir; bu birden fazla waypoint varsa P2'ye erken gecise neden olabilir.

---

## 8. Simulasyon ve Gercek Sistem

Proje simulasyon oncelikli gelistirilir. Simulasyonda gercek sistemle ayni ana gorev mantigi, ayni telemetri yaklasimi ve ayni otopilot haberlesme prensibi korunur.

Simulasyonda fiziksel parcalar yazilim karsiliklariyla temsil edilir:

| Gercek Sistem | Simulasyon Karsiligi |
|---|---|
| Pixhawk | ArduPilot SITL |
| Tekne ve parkur | Gazebo |
| Kamera | Simulasyon kamera akisi |
| Lidar | Simulasyon lidar akisi |
| MAVLink baglantisi | SITL MAVLink baglantilari |

Simulasyon dogrulamasi tamamlanmadan gercek donanim testi yapilmaz.

---

## 9. Kayitlar ve Izleme

Sistem gorev sirasinda yerel dosya sistemine kayit alir.

Genel kayit turleri:

- Telemetri verileri
- Gorev durumu
- Sensor hazirlik ve saglik bilgileri
- Kamera/lidar isleme sonuclari
- Servis loglari
- Simulasyon ve dogrulama loglari
- Hata, uyari ve E-stop olaylari

Loglar, gorev sonrasi analiz ve sistem dogrulama icin kullanilir. Yaris modunda kayit sistemi goruntu aktarimi amaciyla kullanilmaz.

---

## 10. Operasyon ve Dogrulama

Operasyon ve dogrulama akislari icin proje kurallari takip edilir:

- Ajan, gelistirme ve test kurallari icin `AGENTS.md`
- Sistem calisma ozeti icin `documents/rapor_calismasistemi.md`
- Yaris sartname uyumu icin `documents/ida_sartname.md`
- Kontrol sahipligi ve operasyon ozeti icin bu README
- Host/sim baslatma ve dogrulama akislarinda ilgili betikler

Otomatik gorev baslatan simulasyon veya hareket testleri operator kontrolu olmadan calistirilmaz. Kod ve dokuman degisikliklerinden sonra proje dogrulama betikleriyle uyum kontrolu yapilir.

---

## 11. Genel Dizin Yapisi

| Dizin/Dosya | Genel Icerik |
|---|---|
| `documents/` | Sartname ve calisma sistemi dokumanlari |
| `config/` | Sistem ve Pixhawk konfigurasyon dosyalari |
| `docker_workspace/` | Arac uzerinde calisan runtime yazilimlari |
| `host_scripts/` | Host baslatma, durdurma ve dogrulama araclari |
| `sim/` | Simulasyon ortami, kopruler ve sim konfigurasyonlari |
| `logs/` | Sistem, host ve simulasyon loglari |
| `AGENTS.md` | Proje ajan ve otomasyon kurallari |

---

## 12. Lisans ve Takim

MIT lisansi.

Akdeniz Universitesi - Celebiler IDA Takimi.
