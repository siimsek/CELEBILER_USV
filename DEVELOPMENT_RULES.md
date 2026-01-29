# ⚠️ DEVELOPMENT RULES & CODING STANDARDS

Bu proje şu anda **DEBUG & TEST** modundadır. Aşağıdaki kurallar her kod değişikliğinde dikkate alınmalıdır.

## 1. GENEL KODLAMA KURALLARI
1.  **Sessizlik Yasak:** Her kod çalıştığında konsola ne yaptığını basmalı (`print`).
2.  **Port Temizliği Şart:** Her Python scripti (`if __name__ == "__main__":` bloğunda), başlamadan önce kendi portunu `fuser -k` ile temizlemelidir. "Address already in use" hatası kabul edilemez.
3.  **Hata Toleransı (Retry):** Kamera kopsa, Pixhawk yanıt vermese veya Lidar dursa bile kod **çökmemeli**, `try-except` bloklarıyla sürekli yeniden bağlanmayı denemelidir.

## 2. TEST AŞAMASI GEREKSİNİMLERİ (Şu Anki Faz)
* **Görselleştirme:** Tüm sensör verileri Web Arayüzü üzerinden izlenebilir olmalıdır. `cv2.imshow` kullanılmamalıdır (Docker GUI sorunu), yerine Flask Streaming kullanılmalıdır.
* **Web Port Haritası:**
    * **8080:** Ana Dashboard (Telemetri verileri, durum özeti).
    * **5000:** İşlenmiş Kamera Görüntüsü (AI algılamaları çizilmiş halde).
    * **5001:** Lidar Haritası (Kuş bakışı 2D harita).
    * **8888:** Ham Kamera Yayını (Host -> Docker arası iç hat).
* **Loglama:** Tüm kritik veriler `/root/workspace/logs/` altına hem `.log` (konsol çıktısı) hem de `.csv` (veri seti) olarak kaydedilmelidir.

## 3. BAŞLATMA VE DURDURMA PROSEDÜRLERİ
* **ASLA** `docker start` elle yazılmamalıdır.
* **Başlatmak için:** Host terminalinde `start` komutu (veya `system_start.sh`) kullanılmalıdır. Bu script IP kontrolü ve donanım hazırlığı yapar.
* **Durdurmak için:** Host terminalinde `stop` komutu kullanılmalıdır. Bu komut hem Docker'ı hem de kamera sürecini (`rpicam-vid`) temizler.
* **Güncellemek için:** Kod değişikliğinden sonra mutlaka `guncelle` komutu kullanılmalıdır.

## 4. REFERANS BELGELER (KLASÖR İÇİNDE)
Kodlama yaparken aşağıdaki dokümanlar sürekli referans alınmalıdır:
1.  `2026_İnsansız_Deniz_Araci_Şartnamesi_TR.pdf`: Yarışma kuralları ve puanlama için.
2.  `RPLidar_S2E_User_Manual.pdf`: Lidar protokolü ve hata kodları için.
3.  `Pixhawk_Wiring.pdf`: Pin bağlantıları ve portlar için.

## 5. ACİL DURUM VE GÜVENLİK
* Otonom kod (`fusion_main.py`), Lidar verisi 3 saniyeden fazla kesilirse veya Engel Algılandı bayrağı kalkarsa **MOTORLARI DURDURMALIDIR (PWM 1500)**.
* Sistem başlatılırken motorlar her zaman **DISARMED** modunda başlamalıdır.