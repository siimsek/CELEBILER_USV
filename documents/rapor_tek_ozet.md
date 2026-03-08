# Rapor Tek Ozet

Bu dosya, daginik raporlarin onemli kisimlarini tek yerde toplar.

## Sabit Referanslar

- `documents/ida_sartname.md` (degistirilmedi)
- `documents/rapor_calismasistemi.md` (degistirilmedi)

## Uyum Sonucu (Kod/Davranis)

- Kontrol-1 (statik): `15/15` PASS
- Kontrol-2 (davranis): `10/10` PASS
- Not: Bu skorlar kod/davranis kapsamidir; saha kaniti gerektiren maddeler bu skorlara dahil degildir.

## Kod Tarafinda Kapanan Kritik Basliklar

- Emniyet: fail-safe, E-stop, komut kilidi
- Gorev akisi: otomatik parkur gecisi, READY kapisi
- Arayuz/API: manuel parkur gecis endpointlerinin kapanmasi, race gorunurluk kurallari
- Esik/frekans: `R_wp=2.5m`, `T_hold=2.0s`, `D_min=2.0m`, `timeout=180s`, `retry=60s`, heartbeat `5s/30s`, telemetri `5Hz`

## Saha Kaniti Bekleyen Maddeler

| Ref | Konu | Gerekli Kanit | Kapanis Kosulu |
|---|---|---|---|
| SK-G3.1 | IDA -> YKI canli goruntu aktarimi yasagi | Saha videosu + hakem goruntusu | Gorev sirasinda canli aktarim olmadigi kayitta net gorunmeli |
| SK-H3.2 | Hucresel modem yasagi + saha frekans planina uygun kanal secimi | Saha frekans tutanagi + cihaz envanteri | Tutanakta kanal/onay bilgisi ve modem yok kaydi bulunmali |
| SK-L1 | Otonomi videosu 3 bolmeli duzen | Saha cekim kaydi | Tek videoda YKI + senkron grafik + dis kamera eszamanli gorunmeli |
| SK-L2 | Video teslim kriterleri | Final teslim videosu + metadata | Min 720p, 2-5 dk, 4 nokta mission akisi ve guc kesme kaniti birlikte saglanmali |

## Acik Riskler

- USB yazma ortami race sahada bagli degilse READY bloklanabilir.
- SK-G3.1 kaniti eksik kalirsa goruntu aktarim yasagi dogrulanamaz.
- SK-H3.2 kaniti eksik kalirsa frekans/iletisim uygunlugu belgesiz kalir.
- SK-L1/SK-L2 kaniti eksik kalirsa video teslim kriterlerinde red riski dogar.

## Operasyon Notu

- Bu dosyada yer alan saha maddeleri kanit baglanmadan `[tamamlandi]` kabul edilmez.
- Gerekirse compliance raporlari script ile yeniden uretilir:
  - `host_scripts/check_compliance_static.py`
  - `host_scripts/check_compliance_behavior.py`
