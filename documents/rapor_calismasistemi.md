# Çalışma Sistemi

Bu doküman CELEBİLER İDA sisteminin genel çalışma yapısını, kullanılan ana parçaları ve görev sırasında hangi bileşenin ne amaçla kullanıldığını özetler. Detaylı kontrol algoritmaları, ileri seviye ayar değerleri ve tuning bilgileri bu dokümanda tutulmaz.

---

## 1. Genel Sistem Yapısı

Sistem, İDA üzerinde çalışan otonomi bilgisayarı, Pixhawk otopilotu, kamera, lidar, RC bağlantısı, telemetri bağlantısı ve yer kontrol istasyonundan oluşur.

Temel prensip:
- Görev planı YKİ tarafında hazırlanır.
- Pixhawk araç üzerindeki düşük seviye sürüş ve motor çıkışlarından sorumludur.
- Raspberry Pi algılama, görev durumu, kayıt, telemetri ve üst seviye otonomi yazılımlarını çalıştırır.
- Yarış sırasında görüntü işleme ve sensör işleme araç üzerinde yapılır.
- Görev başladıktan sonra acil durdurma dışında yeni operatör komutu kabul edilmez.

---

## 2. Kullanılan Ana Parçalar

| Parça | Kullanım Amacı |
|---|---|
| Pixhawk | Otopilot, araç modu, motor/ESC çıkışları, GNSS/IMU verileri |
| Raspberry Pi | Otonomi yazılımları, kamera/lidar işleme, kayıt ve telemetri arayüzleri |
| Kamera | Duba, kapı ve hedef algılama |
| Lidar | Yakın çevre ve engel algılama |
| STM32 | Yardımcı çevresel sensörlerin okunması |
| Crossfire RC | Manuel kontrol, görev başlatma ve uzaktan acil durdurma |
| 433 MHz telemetri modülü | Mission yükleme ve MAVLink telemetri |
| YKİ bilgisayarı | Mission Planner/QGroundControl ile görev hazırlama ve izleme |
| ESC ve motorlar | Teknenin itki sistemi |
| Fiziksel E-stop/kontaktör hattı | Motor gücünün güvenli şekilde kesilmesi |

---

## 3. Yazılım Bileşenleri

Yazılım tarafı belirli dosya adlarına veya eski modül ayrımlarına bağlı anlatılmaz. Sistem genel olarak şu görev gruplarından oluşur:

| Görev Grubu | Amaç |
|---|---|
| Ana görev yönetimi | Hazırlık, başlatma, durdurma, parkur geçişleri ve görev durumunun takibi |
| Pixhawk/MAVLink bağlantısı | Pixhawk ile mod, konum, görev, telemetri ve üst seviye hedef haberleşmesi |
| Kamera işleme | Duba, kapı, hedef rengi ve yanlış hedef bilgisinin araç üzerinde çıkarılması |
| Lidar işleme | Yakın çevre, engel ve mesafe bilgisinin araç üzerinde çıkarılması |
| Telemetri ve arayüz | YKİ/dashboard tarafına görev durumu, sağlık bilgisi ve kayıtların sunulması |
| Mission işleme | Yüklenen görev noktalarının sistemin kullandığı ortak görev yapısına çevrilmesi |
| Kayıt ve loglama | Görev olayları, sensör durumu, telemetri ve hata bilgilerinin yerel olarak saklanması |
| Simülasyon köprüleri | Gerçek donanım yerine kullanılan simülasyon parçalarını aynı yazılım akışına bağlamak |

Bu bölüm mimariyi genel seviyede tarif eder. Dosya isimleri, sınıf yapıları, algoritma detayları ve kontrol ayarları güncel uygulamaya göre değişebilir.

---

## 4. Haberleşme Yapısı

Sistemde haberleşme kanalları görevlerine göre ayrılmıştır:

| Link | Kullanım |
|---|---|
| Crossfire RC | Manuel sürüş, yarış start tetikleme, uzaktan E-stop |
| 433 MHz MAVLink telemetri | Mission yükleme, Pixhawk telemetrisi, YKİ izleme |
| Pixhawk - Raspberry Pi MAVLink | Araç içi durum, görev ve setpoint haberleşmesi |
| Raspberry Pi - STM32 USB seri | Yardımcı sensör verileri |

Kısıtlar:
- 2.4 GHz ve 5 GHz tabanlı görev/telemetri/görüntü aktarımı kullanılmaz.
- Hücresel modem veya hotspot kullanılmaz.
- Yarış modunda görüntü aktarımı yapılmaz.
- YKİ yalnızca mission yükleme, arayüz ve telemetri izleme için kullanılır.

---

## 5. Görev Akışı

Genel görev sırası:

1. İDA'ya güç verilir.
2. Pixhawk, Raspberry Pi ve sensörler başlar.
3. Kamera, lidar, RC, telemetri, depolama ve E-stop sağlık kontrolleri yapılır.
4. Görev noktaları YKİ üzerinde hazırlanır.
5. Mission Pixhawk'a yüklenir.
6. Sistem hazır durumdayken görev operatör komutuyla başlatılır.
7. Parkur1, Parkur2 ve Parkur3 sırasıyla tamamlanır.
8. Parkurlar arası geçişler otomatik yapılır.
9. Görev tamamlandığında araç güvenli bekleme durumuna alınır.
10. Geri kazanım manuel olarak yapılır.

Görev otomatik başlamaz. Test modunda dashboard/API start, yarış modunda RC start kullanılır.

---

## 6. Parkur Özeti

### Parkur1

Parkur1'de görev noktaları takip edilir. Yarış modunda bu bölüm Pixhawk AUTO mission yürütmesiyle yapılır. Raspberry Pi bu aşamada sağlık, durum ve görev ilerlemesini izler.

### Parkur2

Parkur2'de kamera ve lidar kullanılır. Kamera kapı/duba algısı için, lidar ise yakın çevre ve engel farkındalığı için kullanılır. Raspberry Pi üst seviye hedef üretir, Pixhawk araç kontrolünü uygular.

Waypoint geçişlerinde ana navigasyon referansı her zaman bir sonraki waypoint bearing'idir. Araç yeni waypoint bacağına geçtiğinde önce hedef yöne dönme davranışı doğrulanır; çok uzak, tek karelik veya kararsız kamera/lidar algıları bu yönelimi bastıramaz. Yakın ve doğrulanmış engel varsa kaçınma komutu ana heading üzerine sınırlı bir düzeltme olarak uygulanır.

### Parkur3

Parkur3'te kamera ile hedef rengi ve hedef konumu takip edilir. Hedefe yaklaşma ve angajman durumu araç üzerindeki yazılımlar tarafından değerlendirilir. Yanlış hedef algısı güvenlik amacıyla görev durumuna yansıtılır.

---

## 7. Emniyet ve Kontrol Sahipliği

Sistemde kontrol sahipliği şu şekilde ayrılır:

- Pixhawk:
  - Motor/ESC çıkışları
  - Araç modu
  - Düşük seviye sürüş kontrolü
  - Manuel RC önceliği
  - Otopilot güvenlik davranışları

- Raspberry Pi:
  - Görev durumu
  - Kamera/lidar işleme
  - Üst seviye hedef üretimi: Raspberry Pi setpoint üretir; Pixhawk bu setpoint’i thruster/ESC komutuna dönüştürür.
  - MAVLink GUIDED iletileri: `SET_POSITION_TARGET_GLOBAL_INT` üzerinden hız ve yön hedefi gönderilir.
  - Telemetri ve kayıt
  - Sağlık izleme

Manuel RC ve E-stop her zaman otonomiden önceliklidir. Fiziksel güç kesme zinciri yazılımsal durdurmanın yerine geçmez; motor gücü gerektiğinde fiziksel olarak kesilir.

Heading kontrol zinciri loglarla izlenebilir tutulur: Raspberry Pi `SET_POSITION_TARGET_GLOBAL_INT` ile hedef hız/yön gönderir, ArduPilot bu hedefi servo/PWM çıkışına çevirir, simülasyonda `sitl_gazebo_bridge.py` bu servo çıkışını Gazebo hareketine taşır. Bu zincirde `heading_error`, `target_bearing`, `SERVO_OUTPUT_RAW`, sol/sağ motor eşleşmesi ve gözlenen yaw rate aynı zaman penceresinde karşılaştırılır. WP1 -> WP2 geçişi gibi ters dönüş riski olan durumlarda başarı kriteri, en kısa heading hatası işareti ile gözlenen yaw rate işaretinin tutarlı olmasıdır.

---

## 8. Veri Kayıtları

Görev sırasında aşağıdaki veriler kaydedilir:

| Kayıt | İçerik |
|---|---|
| Kamera kaydı | İşlenmiş kamera görüntüsü, hedef/duba işaretlemeleri |
| Telemetri CSV | Konum, hız, yönelim, mod, görev durumu, setpoint bilgileri |
| Lidar/harita çıktısı | Yerel engel/harita görselleştirmesi |
| Servis logları | `cam`, `telemetry`, `lidar_map`, `usv_main` ve simülasyon servis logları |
| Mission state | Aktif görev, waypoint, parkur, sağlık ve güvenlik durumu |
| Kontrol korelasyon kaydı | GUIDED setpoint, ArduPilot servo çıkışı, motor eşleşmesi, yaw rate ve wrong-turn tanısı |

Kayıtlar yerel dosya sisteminde tutulur. Yarış modunda bu kayıt sistemi görüntü aktarımı amacıyla kullanılmaz.

---

## 9. Simülasyon ve Gerçek Sistem

Proje simülasyon öncelikli geliştirilir. Simülasyonda gerçek sistemle aynı ana yazılımlar, aynı mission/state yapısı ve aynı MAVLink akışı kullanılır.

Simülasyonda fiziksel parçaların karşılıkları:

| Gerçek Sistem | Simülasyon Karşılığı |
|---|---|
| Pixhawk | ArduPilot SITL |
| Tekne ve parkur | Gazebo |
| Kamera | ROS/Gazebo kamera akışı |
| Lidar | Gazebo lidar akışı |
| MAVLink bağlantısı | SITL TCP/UDP bağlantıları |

Simülasyon başarılı ve doğrulanmış olduğunda aynı ana yazılımlar gerçek İDA üzerinde çalıştırılır.

---

## 10. YKİ ve Görev Gösterimi

YKİ üzerinde temel olarak şu bilgiler izlenir:

- Araç konumu
- Araç modu
- Aktif görev/parkur
- Aktif waypoint
- Kamera/lidar hazır durumu
- RC ve telemetri bağlantı durumu
- E-stop durumu
- Görev olayları ve hata bilgileri

Otonomi kabiliyeti gösteriminde YKİ ekranı, dış kamera görüntüsü ve temel telemetri grafiklerinin senkron görünmesi hedeflenir.
