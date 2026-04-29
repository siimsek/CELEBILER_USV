# CELEBILER_USV MOOS Entegrasyon TODO

Guncelleme: 2026-04-21
Durum: statik entegrasyon + refactor tamamlandi, dinamik/sim test operatorunde.

## 1. Analiz
- [x] mevcut kod tabanini oku
- [x] mevcut otonomi akisinin cikarimi
- [x] mevcut perception -> decision -> control zincirinin cikarimi
- [x] mevcut state, IPC ve telemetry sozlesmelerinin cikarimi
- [x] sim ve real farklarinin statik tespiti
- [x] kirilgan/hatali alanlarin statik tespiti
- [x] sartname celisme riski olan alanlarin statik tespiti
- [x] `moos_referans` icerigini incele
- [x] referans bazli MOOS yaklasimini netlestir

## 2. Mimari Karar
- [x] MOOS entegrasyonunun dogal yerlestirimini belirle
- [x] yeni modul yapisi tasarla (`moos_decision_layer.py`)
- [x] sim/real ortak mantigi koruyan yaklasimi belirle
- [x] perception/autonomy/control/telemetry sinirlarini netlestir
- [x] race/test kisitlarini mimariye yansit

## 3. Entegrasyon
- [x] MOOS karar katmanini sisteme entegre et (opsiyonel, guvenli fallback)
- [x] perception ciktilari ile MOOS bagini kur
- [x] telemetri/nav state ile MOOS bagini kur
- [x] MOOS kararini kontrol zincirine bagla (policy-gated)
- [x] state/export/telemetry gorunurlugunu koru
- [x] entegrasyonu mevcut akisla uyumlu yerlestir

## 4. Iyilestirme
- [x] bariz I/O yuk sorunu iyilestirmesi (telemetry link-state force-write azaltildi)
- [x] karar katmani modulerlestirme (USV core fallback korunarak)
- [ ] navigation mantiginda ek optimizasyon turu
- [ ] motor kontrol mantiginda ek optimizasyon turu
- [ ] failsafe zincirinde ikinci sertlestirme turu
- [ ] gereksiz karmasikligi azaltma ikinci tur
- [x] bakim kolayligi: karar katmanini izole modulde toplama
- [ ] performans/stabilite ikinci tur statik inceleme

## 5. Simulasyon Uyum
- [x] simle uyumlu ortak mimari korundu
- [x] sim/real ayrismasi giris/adaptasyon seviyesinde tutuldu
- [x] sim-first kullanim bicimi korundu
- [x] dinamik simulasyon testi operatorunde birakildi
- [x] bu turda statik entegrasyon kalitesine odaklanildi

## 6. Yarisma Uyumu
- [x] sartnameye aykiri yeni veri akis/komut yolu eklenmedi
- [x] race modunda nav AUTO safligi korunacak policy-gate eklendi
- [x] otonominin onboard kalmasi korundu
- [x] mission akis kilitleri korunarak entegrasyon yapildi
- [x] static/behavior script ciktilari ile son dogrulama
- [x] `check_compliance_race.py` ile 8 kabul kriteri gecildi (statik/script tabanli)

## 7. Son Kontrol
- [x] sistem daha entegre hale getirildi (MOOS katmani + state/export baglantisi)
- [x] sistemde hedefli optimizasyon yapildi (telemetry write-throttle)
- [x] belirgin riskli alanlardan biri iyilestirildi (link-state I/O churn)
- [ ] nav ve motor zinciri ikinci gecis inceleme
- [x] sim-real ortak mimari korunumu saglandi
- [x] checklist dosyasi olusturuldu ve guncellendi
- [x] yeni bulgulara gore ikinci iterasyon TODO revizyonu

## Notlar
- `MOOS_REFERANS` dizini talepta buyuk harfle geciyor; repoda mevcut yol `moos_referans`.
- MOOS karar kontrolu varsayilan olarak kapali:
  - `MOOS_DECISION_ENABLED=1`
  - `MOOS_DECISION_CONTROL_ENABLED=1`
  ayarlanmadan aktif override yapmaz.
