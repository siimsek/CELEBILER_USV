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
