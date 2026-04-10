# CELEBILER_USV Agent Rules

Bu dosya bağlayıcı kural dosyasıdır. Ajan, alt ajan ve otomasyon işleri bu sırayla uymak zorundadır.

## 1. Belge Önceliği
1. `/documents/ida_sartname.md`
2. `/documents/rapor_calismasistemi.md`
3. `host_scripts/` içindeki start/stop/doğrulama akışları
4. Bu dosya

Üst sıradaki belge alt sıradaki kural ile çelişirse üst sıradaki geçerlidir.

## 2. Rol Ayrımı
- `SDA` yalnızca tanı, şartname uyum analizi ve plan üretir.
- `SDA`, `autonomy/control/mission` çekirdek kodunu doğrudan değiştirmez.
- `SDA`, onaylı planını `/memories/session/ida-sim-diagnosis.md` içine yazar.
- `Implementation Agent` yalnızca onaylı ve şartnameye uyumlu planı uygular.
- Kod değişikliği minimum diff ile yapılır ve manuel düzenleme için `apply_patch` kullanılır.

## 3. Zorunlu İş Akışı
- Her görev başında `vexp run_pipeline` çağrılır.
- `vexp` erişilemiyorsa blokaj loglanır; ardından yerel dosya incelemesine kontrollü geçilir.
- Kod değişikliği öncesi en az şu dosyalar okunur:
  - `/documents/ida_sartname.md`
  - `/documents/rapor_calismasistemi.md`
  - ilgili `host_scripts/*`
- Görevle ilgisiz değişiklikler geri alınmaz.
- E-stop, röle, RC override ve mission state şeması zayıflatılamaz.

## 4. Haberleşme ve Donanım Yasakları
- `2.4–2.8 GHz` ve `5.15–5.85 GHz` bandında çalışan yeni haberleşme yolu eklenmez.
- Hücresel modem, hotspot, Wi-Fi tabanlı görev/telemetri/görüntü aktarımı eklenmez.
- Otonomi, görüntü işleme ve sensör işleme gemi dışına taşınmaz.
- YKİ yalnızca arayüz, mission yükleme ve telemetri amaçlı kullanılır.
- Yarış modunda görüntü aktarımı açılmaz.

## 5. Kontrol ve Emniyet Kuralları
- Manuel/RC override, otonomiden daima önceliklidir.
- Görev başladıktan sonra acil durdurma dışında yeni operatör komutu kabul edilmez.
- E-stop zinciri fiziksel güç kesme mantığını zayıflatacak şekilde değiştirilemez.
- Fail-safe davranışları `HOLD`, hız düşürme ve güvenli pasif bekleme mantığıyla uyumlu kalır.
- Parkurlar arası geçiş kullanıcı girdisi olmadan otomatik olmalıdır.

## 6. Mod Kuralları
- Geliştirme ve dashboard doğrulaması `test` modunda yapılır.
- Yarış mantığı `race` modunda onboard odaklı ve görüntü aktarımı kapalı çalışır.
- Simülasyon işleri `USV_SIM=1` ile ayrıştırılır.

## 7. Zorunlu Loglama Standardı
- Simülasyonda bütün çekirdek servisler merkezi log init kullanır.
- Simülasyonda fonksiyon trace varsayılan olarak açıktır.
- Her servis en az şu dosyaları üretir:
  - `{component}.debug.log`
  - `{component}.jsonl`
- Fonksiyon trace en az şu olayları kapsar:
  - giriş
  - çıkış
  - hata
  - süre bilgisi
- Şu olaylar `INFO/WARN/ERROR` seviyesinde açıkça loglanır:
  - mode değişimi
  - mission load/start/stop
  - RC override
  - E-stop
  - GPS fix kaybı
  - lidar timeout
  - kamera timeout
  - failsafe geçişi
  - waypoint geçişi
- Loglar yalnızca yerel dosya sisteminde tutulur; şartnameye aykırı görüntü aktarımı için kullanılamaz.
- Log rotasyonu kapatılamaz.
- Simülasyonda üretilen yeni log dosyaları dashboard log görüntüleyicisinde görünür olmalıdır.

## 8. Veri ve Şema Kuralları
- Servisler arası mission/state paylaşımı mevcut dosya şemaları bozulmadan sürdürülür.
- `docker_workspace/mission.json`, `sim/control/*.json` ve benzeri ortak durum dosyalarında geriye dönük uyum korunur.
- Geçersiz sensör verisi doğrudan görev hattına verilmez; önce guard/filter uygulanır.

## 9. Dizin Sorumlulukları
- `docker_workspace/src/`: çalışma zamanı servisleri ve kontrol mantığı
- `sim/bin/`: simülasyon orkestrasyonu
- `sim/bridges/`: simülasyon haberleşme köprüleri
- `sim/control/`: simülasyon kontrol ve paylaşılan durum dosyaları
- `host_scripts/`: host başlatma, durdurma ve doğrulama

## 10. Doğrulama Zorunluluğu
- Her değişiklik sonrası en az aşağıdakiler çalıştırılır:
```bash
python3 -m py_compile docker_workspace/src/*.py
python3 host_scripts/check_compliance_static.py
python3 host_scripts/check_compliance_behavior.py
./sim/bin/run_sim_stack.sh
```
- Simülasyon çalıştırılamıyorsa neden açıkça not edilir.

## 11. vexp Kuralı
- Kod keşfi için birincil araç `run_pipeline`dır.
- Alt ajanlara görev devredilecekse `run_pipeline` çıktısı bağlam olarak aktarılır.
- `run_pipeline` yoksa veya daemon kapalıysa bu durum loglanmadan rastgele keşfe geçilmez.


## vexp <!-- vexp v1.3.11 -->

**MANDATORY: use `run_pipeline` — do NOT grep or glob the codebase.**
vexp returns pre-indexed, graph-ranked context in a single call.

### Workflow
1. `run_pipeline` with your task description — ALWAYS FIRST (replaces all other tools)
2. Make targeted changes based on the context returned
3. `run_pipeline` again only if you need more context

### Available MCP tools
- `run_pipeline` — **PRIMARY TOOL**. Runs capsule + impact + memory in 1 call.
  Auto-detects intent. Includes file content. Example: `run_pipeline({ "task": "fix auth bug" })`
- `get_context_capsule` — lightweight, for simple questions only
- `get_impact_graph` — impact analysis of a specific symbol
- `search_logic_flow` — execution paths between functions
- `get_skeleton` — compact file structure
- `index_status` — indexing status
- `get_session_context` — recall observations from sessions
- `search_memory` — cross-session search
- `save_observation` — persist insights (prefer run_pipeline's observation param)

### Agentic search
- Do NOT use built-in file search, grep, or codebase indexing — always call `run_pipeline` first
- If you spawn sub-agents or background tasks, pass them the context from `run_pipeline`
  rather than letting them search the codebase independently

### Smart Features
Intent auto-detection, hybrid ranking, session memory, auto-expanding budget.

### Multi-Repo
`run_pipeline` auto-queries all indexed repos. Use `repos: ["alias"]` to scope. Run `index_status` to see aliases.
<!-- /vexp -->