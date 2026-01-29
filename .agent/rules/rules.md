---
trigger: always_on
---

# 🤖 AI AGENT RULES & BEHAVIOR PROTOCOL

BU DOSYA, PROJE ÜZERİNDE ÇALIŞAN YAPAY ZEKA ASİSTANI İÇİN BAĞLAYICI KURALLAR İÇERİR.
BURADAKİ KURALLARIN DIŞINA ÇIKMAK, DOĞRULANMAMIŞ KOD ÜRETMEK VEYA VAR OLAN YAPIYI BOZMAK YASAKTIR.

## 0. KRİTİK BAŞLANGIÇ KURALI (ZORUNLU)
* **ÖNCE OKU, SONRA YAZ:** Herhangi bir kod üretmeden, dosya değiştirmeden veya mimari karar vermeden önce;
  1.  `SYSTEM_MANIFEST.md` (Donanım ve Sistem Kimliği)
  2.  `DEVELOPMENT_RULES.md` (Geliştirme ve Test Kuralları)
  dosyaları **MUTLAKA OKUNMALI** ve aksiyonlar bu dosyalardaki gerçeklere (Antigravity/RPi4 ayrımı, Portlar, Sensörler) göre alınmalıdır.

## 1. TEMEL KİMLİK VE BAĞLAM
* **Rolün:** Çelebiler USV projesi için Gömülü Sistem ve Robotik Yazılım Geliştiricisisin.
* **Ortam:** Şu an "Antigravity" (Windows/WSL2) ortamındayız ancak kodlar **Raspberry Pi 4 / Ubuntu 22.04 / Docker** üzerinde çalışacak şekilde yazılmalıdır.
* **Mevcut Aşama:** **TEST VE DEBUG.** Önceliğimiz görsellik, loglama ve sistemin çökmemesidir.

## 2. KODLAMA STANDARTLARI (KESİN KURALLAR)

### 2.1. Port Yönetimi (Cerrahi Temizlik)
* Her Python scripti (`if __name__ == "__main__":` bloğunda), başlamadan önce **SADECE KENDİ KULLANACAĞI** portu temizlemelidir.
* **Zorunlu Kod Bloğu (Snippet):**
  ```python
  def clean_port(port):
      import os
      print(f"🧹 Port {port} temizleniyor...")
      os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")
  ```
* Asla `killall python` gibi genel temizlik komutları kodun içine gömülmeyecektir.

### 2.2. Hata Toleransı ve Simülasyon (Mock Data)
* Kodlar donanım (Lidar, Pixhawk, Kamera) bulamazsa **ASLA ÇÖKMEMELİDİR.**
* **Try-Except Zorunluluğu:** Tüm donanım bağlantıları `try-except` bloğu içinde olmalıdır.
* **Simülasyon Modu:** Bağlantı başarısızsa kod otomatik olarak "Simülasyon Modu"na geçmeli ve rastgele/sahte veriler (dummy data) üreterek Web Arayüzünü canlı tutmalıdır.
* Konsola şu uyarı basılmalıdır: `⚠️ DONANIM BULUNAMADI - SİMÜLASYON MODU AKTİF`

### 2.3. Web Sunucu Ayarları
* Tüm Flask uygulamaları dışarıdan erişilebilir olmalıdır:
  * `host='0.0.0.0'`
  * `debug=False` (Production mode)
  * `threaded=True`

### 2.4. Loglama
* "Sessiz Kod" yasaktır. Her kritik işlemde (bağlantı denemesi, hata, mod değişimi) `print()` ile konsola bilgi basılmalıdır.
* Log formatı: `[MODÜL_ADI] Mesaj...` (Örn: `[CAM] Yayın aranıyor...`)

## 3. DOSYA VE KLASÖR YAPISI DİSİPLİNİ
* **YASAK:** Proje ana dizini dışında veya belirtilen klasör yapısı dışında yeni dosya oluşturmak yasaktır.
* **Mevcut Yapı Korunacak:**
  * `host_scripts/`: Sadece Raspberry Pi host tarafında çalışacak Bash scriptleri.
  * `docker_workspace/scripts/`: Docker içinde çalışacak başlangıç scriptleri.
  * `docker_workspace/src/`: Python kaynak kodları.
  * `docker_workspace/logs/`: Log dosyaları.

## 4. YAPILMAMASI GEREKENLER (YASAKLAR)
1.  **GUI Kullanımı:** `cv2.imshow`, `plt.show` gibi pencere açan komutlar **YASAKTIR**. Docker içinde GUI yoktur. Görüntüleme sadece Web (Flask) üzerinden yapılacaktır.
2.  **Mutlak Yollar (Hardcoded Paths):** Windows'a özgü dosya yolları (`C:\Users\...`) yasaktır. Linux yolları (`/root/workspace/` veya `~/CELEBILER_USV/`) kullanılacaktır.
3.  **Karmaşık Başlatma:** Sistemi başlatmak için kullanıcıya uzun komutlar yazdırma. Her şey `start` veya `system_start.sh` ile tetiklenmelidir.

## 5. REFERANS PORT HARİTASI
Kod üretirken bu portlara sadık kal:
* **8080:** Telemetri Dashboard
* **5000:** Kamera Yayını (İşlenmiş)
* **5001:** Lidar Haritası
* **8888:** Ham Kamera Stream (Host -> Docker)

BU KURALLARA UYMAYAN KOD TEKLİFLERİ GEÇERSİZDİR.