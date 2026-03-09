# Çelebiler USV - Otonom Sistem İnovasyon ve Geliştirme Fikirleri

Bu belge, Teknofest veya benzeri İnsansız Deniz Aracı (İDA) yarışmalarında hakem heyetinden ekstra "Özgünlük ve İnovasyon" puanı getirebilecek, sistemin otonomi zekasını standart bir araçtan çıkarıp "gelişmiş/akıllı" bir robota dönüştürecek vizyon fikirlerini içerir.

## 🌟 1. Sensör Füzyonu (Kamera + Lidar Hayalet Hedef Koruması)
**Sorun:** Deniz yüzeyindeki güneş yansımaları veya dalga köpükleri kameranın anlık olarak "Burada kırmızı şamandıra var" demesine (False-Positive) yol açabilir.
**Çözüm:** Kamera bir hedefin açısını tespit ettiğinde, Lidar verisinde o doğrultuda fiziksel bir kütle olup olmadığı doğrulanır. Lidar doğrulamıyorsa sistem bu tespiti "hayalet/yansıma" olarak etiketleyip görmezden gelir. Bu sayede vizyon hatalarından kaynaklı yanlış manevraların önüne geçilir.

## 🚤 2. Dinamik Hız Profillemesi (Dynamic Speed Profiling / Trajectory Planning)
**Sorun:** Otonom tekneler dar dönüşlerde sabit hızla gitmeye çalışırsa merkezkaç kuvveti nedeniyle dışarı savrulur (overshoot).
**Çözüm:** Hız, "Heading Error" (rotaya olan açısal hata) şiddetine göre dinamik olarak hesaplanır. Hedef tam karşıdaysa motorlar tam gaz beslenir. Hedef yan taraflardaysa (örneğin ufukta 30 derece sapma varsa), sistem viraja girdiğini anlayıp otomatik olarak otonom hızı düşürür, tekne burnunu hedefe sokar ve düzlükte tekrar ivmelenir.

## 🌬️ 3. Akıntı ve Rüzgar Düzeltme Asistanı (Integral Windage Control / Crab Steering)
**Sorun:** Tekne düz bir rotaya kilitlenip ilerlerken yandan esen rüzgar veya su akıntısı tekneyi yörüngesinden paralel olarak dışarı sürükler.
**Çözüm:** Yönelim kontrolcüsüne PID-Integral (I) terimi eklenerek zaman içindeki "sürekli sapma" birikimi tespit edilir. Rüzgar tespiti yapıldığında tekne hedefe dümdüz gitmek yerine hafifçe rüzgara doğru açı vererek (Yengeç Yürüyüşü) sürüklenmeyi kendi kendine kompanze eder.

## 🔋 4. Termal ve Enerji Hayatta Kalma Modu (Eco/Limp Mode)
**Sorun:** Yarışma parkurunda veya denizin ortasında batarya voltajının aniden çökmesi veya Raspberry Pi CPU'sunun ısınarak throttling'e girmesi.
**Çözüm:** Pixhawk'tan gelen batarya verisi veya Pi'nin sıcaklık verisi eşik değerin altına/üstüne çıktığında, sistem otonom olarak "ECO" moda geçer. Maksimum PWM sınırları (örneğin 1900'den 1650'ye) mekanik olarak kilitlenir. Sistem sadece parkuru bitirebilecek kadar enerji tüketimine odaklanır.

## 🦮 5. Lidar Kendi-Gövdesini Gizleme (Lidar Self-Masking)
**Sorun:** Direkte yer alan Lidar'ın (özellikle arka açılarda) teknenin kendi pruvasını, gövdesini veya pervanelerini engel gibi algılayıp haritaya işlemesi.
**Çözüm:** Montaj geometrisi bilindiği için, doğrudan ROS mesajı veya `lidar_map.py` seviyesinde bir "Ignore Zone (Ölü Bölge)" filtresi oluşturulur. Örneğin 170 ile -170 derece arası 1 metreden yakın ölçümler "tekne gövdesi" olarak sınıflandırılıp otonom sistemden / haritadan filtrelenir.

## 🛡️ 6. Etrafı Savunmalı Sanal Çit Koruması (Geo-Fence & Safe-Halt Virtual Anchor)
**Sorun:** Kopan veya zayıflayan telemetri sinyali sonucunda teknenin dalgalarla yarışma dışı alanlara sürüklenmesi veya kıyıya çarpması.
**Çözüm:** "Watchdog" zamanlayıcısı bağlantının tamamen koptuğunu anlarsa motorları kapatarak sistemi durdurur. Fakat sürüklenme devam ediyorsa sistem bunu GPS ile fark eder ve sanal çitten çıkmamak için olduğu yerde küçük rölanti motor hamleleriyle (Sanal Demir Atma - Virtual Anchor) pozisyonunu koruyup bağlantının tekrar gelmesini veya manuel kurtarma ekibini bekler.

## 🚨 7. Lidar Tabanlı Acil Çarpışma Refleksi (Reactive Collision Avoidance Reflex)
**Sorun:** Otonom sistem GPS rotasında ilerlerken (veya kamera bir şamandıraya kilitlenmişken) aniden kör noktadan hareketli bir tekne, martı veya yüzen bir dalga kıran çıkabilir.
**Çözüm:** `usv_main.py` içerisine asenkron çalışan bir "Refleks (Dodge)" katmanı kurulur. O andaki aktif otonom görev ne olursa olsun, Lidar'ın `D_MIN_M` mesafesinden çok daha yakınında (örn. 0.8 metre) bir engel belirirse, refleks katmanı ana otonomi algoritmasını ezer (override). Derhal gazı sıfırlar ve motorlara tam kapasite ters itki (Hard Reverse / Acil Fren) vererek fiziki teması imkansız hale getirir.

## 📐 8. IMU Tabanlı Ufuk Çizgisi Sabitlemesi (Horizon-Locked Targeting)
**Sorun:** Deniz dalgalı olduğunda tekne şiddetli yalpalar (Roll/Pitch). Kamera fiziksel olarak direğe sabit olduğu için şamandıralar kadrajda sürekli yukarı aşağı ve sağa sola kaçar, objenin takibini sarsıntılı hale getirir.
**Çözüm:** Pixhawk'tan gelen IMU (Cayroko) verilerinden `Roll` ve `Pitch` sıvı açısı okunur. Python tarafında, kameranın bounding box piksel hesaplamalarına bu açı dahil edilir. Tekne 10 derece sağa yatsa bile sanal hedef matriksi zıt yönde 10 derece döndürülür (Homography/Perspective Shift). Gövde ne kadar sallanırsa sallansın sistemdeki koordinat okuması dümdüz suya paralelmiş gibi stabil kalır.

## 📡 9. Akıllı Bant Genişliği ve Telemetri Kısılması (Bandwidth-Aware Dynamic Telemetry)
**Sorun:** Yarışma alanlarındaki RF kirliliği veya uzun mesafelerde Telemetri modeminin bant genişliğinin daralması, paket kayıpları (Lag) yaşatır.
**Çözüm:** İDA, `telemetry.py` üzerinden gönderdiği MAVLink ve JSON paketlerinin "Age" veya "RSSI" (Sinyal Gücü) gecikmesini sürekli dinler. Sinyal kalitesi düştüğünde sistem otomatik "Dar Bant (Narrow Band)" moduna geçer. Kamera durumu veya Lidar haritası gibi ağır frekanslı raporları durdurur. Sadece "Konum, Batarya, Otonom Durum" gibi en hayati paketleri 1 Hz ile kodlayarak bağlantının kopmasını engeller.

## ☀️ 10. Kamera Gündüz/Gece Güneş Körü Adaptasyonu (Dynamic Exposure & HSV Tuning)
**Sorun:** Güneşin tam tekneye karşıdan vurduğu parkur açılarında veya bulut gölgesi geçtiğinde kamera paneli aniden kararır veya parlar. Sabit tanımlanmış Kırmızı/Yeşil (HSV) eşik değerleri (Thresholds) objeyi seçemez hale gelir.
**Çözüm:** `cam.py` içerisine bir "Luminance (Parlaklık)" analizörü eklenir. Eğer çerçevenin genel histogramı aniden çok parlak / çok karanlık bölgeye kayarsa (Güneş Parlaması), sistem Raspberry Pi Camera ayarlarına dinamik olarak Exposure (Pozlama) kısma/açma sinyali gönderir. Algokod bu sürede HSV filtre değerlerini göreceli olarak kaydırarak güneşte de loş havada da otonomiyi körlükten korur.

## 🌊 11. Karar Ağaçlı Dalga Patern Kestirimi (Sea-State Estimation & Surf Mode)
**Sorun:** Rüzgarlı havada 30-40 cm dalgalar oluştuğunda teknenin mevcut sabit gaz-limitleri burun daldırmalara veya motor boşa çıkmalarına neden olabilir.
**Çözüm:** Pixhawk'ın Z ekseni ivme ölçer (`Z-Accel` / Heave) verisi 5 veya 10 saniyelik bir Hızlı Fourier Dönüşümü (FFT) zaman aralığına sokulur. Okyanus veya göldeki mevcut "Dalga periyodu" tahmin edilir. Deniz çok bozuksa tekne otomatik olarak "Sörf Profili"ne geçer: Dalga tepesine tırmanırken ekstra tork verir, çukura düşerken pruvanın gömülmemesi için gazı hafif keserek daha yumuşak (Dolphin) stili ilerler.

## ⚠️ 12. Asimetrik Motor Hasar Toleransı (Thrust Vectoring Fallback)
**Sorun:** Skid-steer (çift diferansiyel) çalışan teknede; motorlardan birine yosun/plastik poşet dolanırsa veya sağ ESC yanarsa, tekne sadece sol motorla çalışır ve olduğu yerde daireler çizmeye başlar. Görev çöker.
**Çözüm:** MAVLink IMU Compass (`Yaw`) ile algoritmanın hedeflenen (Heading Target) açısı sürekli kıyaslanır. Otonomi "Ben 4 saniyedir tekne sağa dönsün diye sol motora gaz veriyorum ama compass 1 derece bile dönmüyor" diyerek sol motorun mekanik olarak ÖLDÜĞÜNÜ anlar. Asimetrik Hasar Moduna geçer. Kalan tek motoru sürekli açık tutmak yerine nabız şeklinde ateşleyerek (Yaw-Pulsing/Zigzag) hedefe tek motorla kırık dökük de olsa ulaştırır. Dünyadaki otonom sistemlerde en çok aranılan kritik toleranstır.

## 💦 13. Sintine / Gemi İçi Su Acil Durum Tespiti (Hull Breach Failsafe)
**Sorun:** Kötü izolasyon kaynaklı teknenin ağır ağır su alması. Denizin ortasındaki yalıtım hatası sistemi içten içe kızartarak İDA'yı veya pahalı beyin kartlarını komple kaybetmeye neden olur.
**Çözüm:** Raspberry Pi'nin boş bir GPIO pinine bağlanacak 2 Dolarlık basit bir sıvı/nem (Water Leak) sensöründen değer okunur. Şiddetli su görüldüğü anda "HULL BREACH (GÖVDE DELİNDİ)" acil durum protokolü girer. Sistem otonom görevi REDDEDER. Pompalar tetiklenir ve araç can havliyle, gücü en aza indirgeyerek suya batmadan önce "RTL (Return to Launch/Kıyıya Dön)" manevrası atar.

## 🎛️ 14. Bilişsel Sağlık İndeksi Algoritması (Autonomy Health Scoring / Trust Bar)
**Sorun:** Arayüz üzerinden aracı denetleyen Operatörün elinde genelde sadece "Lidar Bağlı / Kamera Açık" gibi ikili (Binary) seçenekler vardır. Otonominin hata yapıp yapmayacağını sezgisel olarak bilemez.
**Çözüm:** Yalnızca donanım bağlılıklarına bakmayan, ortamı da hesaba katan ağırlıklı bir "Sağlık Skoru (Trust Factor = %0-100)" algoritması oluşturulur. Örneğin: `(Uydu Sayısı) * 0.30 + (Kamera Işık Stabilitesi) * 0.20 + (Lidar Nokta Sayısı) * 0.25 + (RC Ping) * 0.25`. Eğer Trust Factor %85 ise arayüzde yeşil yanar, pilot arkasına yaslanır. Yansıyan güneş veya zayıf uydu nedeniyle skor %40'lara düşerse İDA pilotu arayüzden "DİKKAT! Sensör Güveni Düştü, Otonomi Kararsızlaşabilir, RC Kumandayı Eline Al" şeklinde erkenden uyarır.
