# Rapor Uyum Plani ve Adim Adim TODO

Bu dosya, `documents/rapor_calismasistemi.md` baz alinarak tum sistemi adim adim guncellemek icin ana calisma planidir.

## 1) Kaynak Onceligi
- 1. oncelik: `documents/rapor_calismasistemi.md`
- 2. oncelik: `documents/ida_sartname.md`
- 3. oncelik: proje kodu ve mevcut README

## 2) Calisma Kurali
- Her adim sonunda hem rapor hem sartnameye karsilik kontrolu yapilacak.
- Her adim bitince test/kanit eklenmeden tik atilmayacak.
- Belirsizlikte varsayimla ilerlemek yerine kullaniciya net soru sorulacak.
- Her adim icin "rapor maddesi -> kod/konfig -> test kaniti" zinciri zorunlu olacak.
- Kontrol-1: `host_scripts/check_compliance_static.py`
- Kontrol-2: `host_scripts/check_compliance_behavior.py`
- Bu plan, bir sonraki sohbette adim adim uygulanacak.

## 3) Master TODO (Adim Adim)

### A. Hazirlik ve Eslestirme
- [x] A1 - Mevcut kodu rapor bolumleriyle birebir eslestirme tablosu cikar.
- [x] A1.1 - Bolum 1.3 HEALTH_CHECK kalemlerini (MAVLink, RC RSSI, E-stop, kamera<=1.0s frame, lidar timeout, yazilabilir depolama) kod modulleriyle eslestir.
- [x] A1.2 - Bolum 1.8-1.12 esiklerini (`R_wp=2.5m`, `T_hold=2.0s`, `D_min=2.0m`, `timeout=180s`, `retry=60s`, heartbeat `5s/30s`) kod sabitleriyle tek tabloda eslestir.
- [x] A1.3 - Bolum 3.1-3.5 haberlesme topolojisi/frekanslarini (Crossfire, 433 MHz MAVLink, telemetri 5 Hz, olay anlik, lidar 10 Hz sinifi) servis bazinda eslestir.
- [x] A2 - Kritik uyumsuzluklari (davranis, parametre, API, emniyet) onceliklendir.
- [x] A2.1 - P0 emniyet uyumsuzluklari: fail-safe, E-stop, komut kisiti.
- [x] A2.2 - P1 gorev akisi uyumsuzluklari: parkur gecisleri, mod gecisleri, tamamlanma kriterleri.
- [x] A2.3 - P2 arayuz/API uyumsuzluklari: gereksiz endpointler ve rapor disi UI alanlari.
- [x] A3 - Test kapsamini ve kabul kriterlerini madde madde kilitle.
- [x] A3.1 - Her madde icin zorunlu kanit tipini sabitle: test cikisi + log + ekran/video kaniti.

### B. Mod Yonetimi (test/race)
- [x] B1 - Test/Yarisma modu tek noktadan yonetilecek yapiyi netlestir.
- [x] B1.1 - Mod bilgisini tek kaynakta topla (config/env + runtime state), tum servisler ayni kaynagi okusun.
- [x] B1.2 - Mission basladiginda mod degistirme kapisini kilitle; sadece E-stop istisnasi birak.
- [x] B2 - Varsayilan modu `test` olarak sabitle. (Karar netlesti: varsayilan `test` aktif kalacak.)
- [x] B2.1 - Acilis konfigunde explicit olarak `test` varsayilanini uygula; acik secim olmadan `race` acilmasin.
- [x] B3 - Mission aktifken mode degisikligi emniyet davranisini kilitle.
- [x] B4 - Dashboard/servislerde mode davranislarini rapor+sartname ile hizala.

### C. Uctan Uca Sistem Akisi (Rapor Bolum 1)
- [x] C1 - Boot, servis kalkisi, HEALTH_CHECK akisini rapora uygun uygula.
- [x] C1.1 - Boot sirasini netlestir: Pixhawk + RPi acilisi, sonra MAVLink/kamera/lidar/STM32/log servisleri.
- [x] C1.2 - READY durumuna gecis kosulunu bagimsiz kontrollerin tamamina bagla.
- [x] C1.3 - HEALTH_CHECK hata bayraklarini YKI tarafinda gorunur hale getir.
- [x] C2 - Gorev yukleme ve baslatma akisinda YKI odakli kurallari uygula.
- [x] C3 - Komut kisiti (gorev basladiktan sonra sadece acil durdurma) uygula.
- [x] C4 - Parkurlar arasi otomatik gecis davranisini kesinlestir.

### D. Parkur-1 (AUTO)
- [x] D1 - AUTO waypoint takibini rapordaki tamamlama olcutleriyle uyumlu hale getir (`R_wp`, `T_hold`).
- [x] D2 - Hiz profili (seyir/yaklasim) degerlerini rapora gore sabitle.

### E. Parkur-2 (GUIDED)
- [x] E1 - Kamera tabanli kapi yonelimi ve stabilite kontrolunu rapora gore uygula.
- [x] E2 - Lidar sektor (sol-orta-sag) ve `D_min` tabanli kacisma davranisini uygula.
- [x] E3 - Kapi gecisi dogrulama olay bayraklarini (`gate_gecildi`) netlestir.

### F. Parkur-3 (GUIDED)
- [x] F1 - HSV hedefleme ve merkezleme dongusunu rapora gore uygula.
- [x] F2 - Timeout (180s), retry (60s, 1 kez), geri cekilme akislarini uygula.
- [x] F3 - HOLD/pasif sonlandirma davranisini rapora gore sabitle.

### G. Fail-safe ve Emniyet
- [x] G1 - Heartbeat tabanli fail-safe esikleri (uyari/fail-safe) uygula.
- [x] G2 - Arac uzeri + uzaktan guc kesme mantigini yazilim tarafinda dogrula.
- [x] G3 - Race modda yasak goruntu aktarimi kurallarini tekrar test et.
- [ ] G3.1 - IDA -> YKI canli goruntu aktarimi olmadigini (analog/dijital) test kaydiyla kanitla. DOĞRULA: saha videosu + hakem goruntusu
- [x] G3.2 - Wi-Fi ve hucresel baglanti yasagi kontrollerini (konfig + saha) test checklistine bagla.

### H. Haberlesme ve Operasyon Kurallari
- [x] H1 - YKI-Pixhawk-RPi topolojisini raporla birebir hizala.
- [x] H1.1 - Link-A: Crossfire RC + E-stop, Link-B: 433 MHz MAVLink mission/telemetri, ic linkler: Pixhawk<->RPi MAVLink ve RPi<->STM32 USB.
- [x] H2 - Veri gonderim/alim frekanslari ve telemetri icerigini rapora gore sabitle.
- [x] H2.1 - Frekanslari sabitle: genel telemetri 5 Hz, kritik olaylar anlik, engel/gucvenlik bayraklari 5 Hz, lidar isleme 10 Hz sinifi.
- [x] H2.2 - Telemetri alanlarini sabitle: konum, kinematik, tutum, mod/state, gorev ilerleme, olay bayraklari, nav sagligi, link sagligi, emniyet, enerji.
- [x] H3 - Saha kisitlari (Wi-Fi kapali, yasak bantlar, hucresele yasak) kontrol listesi olustur.
- [x] H3.1 - Yasak bantlar (2.4-2.8 GHz, 5.15-5.85 GHz) ve dahili Wi-Fi kapali durumunu saha oncesi checklistine ekle.
- [ ] H3.2 - Hucresel modem kullanilmadigini ve kanal seciminin saha frekans planina uygun yapildigini kanitla. DOĞRULA: saha frekans tutanagi

### I. Veri Kayit ve Teslim (Rapor 1.16 + Sartname 6)
- [x] I1 - Dosya-1 (islenmis sensor videolari) format/frekans/overlay kriterlerini uygula.
- [x] I2 - Dosya-2 (telemetri CSV) zorunlu alanlari ve en az 1 Hz kaydi garanti et.
- [x] I3 - Dosya-3 (lokal harita/costmap) cikti akislarini netlestir.
- [x] I4 - Zaman damgasi onceligi (GNSS, fallback monotonic) mekanizmasini uygula.

### J. Dashboard ve API Sadelestirme
- [x] J1 - Mission kontrol, durum gosterimi, telemetri alanlarini rapor odakli sadelestir.
- [x] J1.1 - Dashboard alanlarini Bolum 3.4 veri gruplariyla sinirla (mod/state, aktif parkur, gate sayaci, fail-safe ve link sagligi dahil).
- [x] J2 - Gereksiz/rapora aykiri endpoint ve butonlari kaldir.
- [x] J2.1 - Mission sonrasi manuel komut uretebilen rapor disi endpoint/butonlari kaldir veya kilitle.
- [x] J3 - UI tarafinda test/race gorunurluk kurallarini netlestir.
- [x] J3.1 - Race modda goruntu aktarimi ve rapor disi debug panellerini gizle/engelle.

### K. Dogrulama, Kanit ve Kapanis
- [x] K1 - Her bolum icin test senaryolari calistir ve log/kanit topla.
- [x] K1.1 - Bolum 1, 2 ve 3 icin ayri test paketleri calistir; kanitlari tarih/saat damgali arsivle.
- [x] K2 - Rapor + sartname uyum checklistini finalden once bastan sona tekrar kontrol et.
- [x] K2.1 - Son kontrolde ozellikle esik/frekans degerlerini (2.5m, 2.0s, 2.0m, 180s, 60s, 5s/30s, 5 Hz) tekrar dogrula.
- [x] K3 - Kalan riskler ve acik sorulari tek listede toplayip kapatma karari ver.
- [x] K3.1 - Acik riskler icin sahip + hedef tarih + etki seviyesi ata.

### L. Otonomi Gosterim Videosu (Rapor 1.15)
- [ ] L1 - Video akisini 3 parcali duzende hazirla: YKI ekrani + senkron grafikler + dis kamera. DOĞRULA: saha cekim kaydi
- [ ] L2 - Video kriterlerini dogrula: min 720p, sure 2-5 dk, 4 nokta mission, guvenlik anahtariyla motor/guc kesme kaniti. DOĞRULA: final teslim videosu

### M. Algoritma ve Haberlesme Dokumani Uyumu (Rapor Bolum 2 ve 3)
- [x] M1 - Parkur1/2/3 akis diyagramlari ile state-machine kodunu birebir eslestir.
- [x] M2 - Ortak esikler tablosunu tek konfig kaynagina bagla; daginik sabitleri temizle.
- [x] M3 - Haberlesme bolumu (3.1-3.8) ile telemetri publisher/API davranisini birebir uyumlu hale getir.

## 4) Gereksizleri Silme Listesi (Surekli Bakim)
- [x] R1 - Kullanilmayan script/dosya/endpointleri tespit et.
- [x] R2 - Gecici log/rapor/artifact dosyalarini `.gitignore` ile filtrele.
- [x] R3 - Kodda dead-path ve eski davranislari temizle (testlerle dogrulayarak).
