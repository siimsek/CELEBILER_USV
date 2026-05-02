#!/usr/bin/env python3
"""
Sim SITL GPS + Mode teşhis scripti.

Kullanım:
    # Sim stack çalışırken (run_sim_stack.sh başlattıktan sonra):
    python3 host_scripts/diagnose_sim_gps.py [--duration 15] [--endpoint tcp:127.0.0.1:5760]

Amaç:
    - SITL GPS fix durumunu doğrulamak (GPS_RAW_INT, fix_type, satellites_visible)
    - GLOBAL_POSITION_INT içeriğini kontrol etmek (lat/lon 0 ise GPS zinciri kırık)
    - HEARTBEAT ile vehicle_mode ve arm durumunu okumak
    - STATUSTEXT / SYS_STATUS üzerinden pre-arm hatalarını izlemek

Çıktı logs/host/diagnose_sim_gps.log dosyasına yazılır.
"""
import argparse
import os
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("[HATA] pymavlink yüklü değil. 'pip install pymavlink' ile kurun.", file=sys.stderr)
    sys.exit(2)

MODE_MAPPING = {
    0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD",
    5: "LOITER", 6: "FOLLOW", 7: "SIMPLE", 10: "AUTO",
    11: "RTL", 12: "SMART_RTL", 15: "GUIDED", 16: "INITIALIZING",
}

FIX_TYPE_MAPPING = {
    0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX",
    4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED", 7: "STATIC", 8: "PPP",
}


def main():
    parser = argparse.ArgumentParser(description="Sim SITL GPS ve mode teşhisi")
    parser.add_argument("--endpoint", default="udpin:127.0.0.1:14552",
                        help="MAVLink endpoint (default: udpin:127.0.0.1:14552 - Mission Planner portu). "
                             "tcp:5760 kullanma (usv_main zaten bagli). "
                             "udpin:14552 = SITL'in Mission Planner icin actigi UDP portu dinle.")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="Dinleme süresi saniye (default: 15)")
    parser.add_argument("--check-params", action="store_true",
                        help="SITL parametrelerini kontrol et (SIM_GPS1_ENABLE, GPS_TYPE vb.)")
    args = parser.parse_args()

    log_dir = Path(__file__).resolve().parent.parent / "logs" / "host"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "diagnose_sim_gps.log"

    def out(msg):
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        print(line)
        with open(log_file, "a") as f:
            f.write(line + "\n")

    out(f"[INFO] Bağlanılıyor: {args.endpoint}")
    try:
        m = mavutil.mavlink_connection(args.endpoint)
        m.wait_heartbeat(timeout=10)
    except Exception as exc:
        out(f"[HATA] MAVLink bağlantı başarısız: {exc}")
        sys.exit(1)

    out(f"[OK] Heartbeat alındı. system={m.target_system} comp={m.target_component}")

    if args.check_params:
        out("")
        out("--- Parametre Kontrol (SIM_GPS1_*, GPS_TYPE, ARMING_CHECK) ---")
        param_names = [
            "GPS_TYPE", "GPS_TYPE2", "ARMING_CHECK",
            "SIM_GPS1_ENABLE", "SIM_GPS1_TYPE", "SIM_GPS1_NUMSATS", "SIM_GPS1_HZ",
            "AHRS_EKF_TYPE", "EK3_ENABLE",
        ]
        received = {}
        for pname in param_names:
            m.mav.param_request_read_send(m.target_system, m.target_component,
                                          pname.encode("ascii"), -1)
        deadline_p = time.monotonic() + 5.0
        while time.monotonic() < deadline_p and len(received) < len(param_names):
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors="ignore")
            pid = pid.strip("\x00 ")
            if pid in param_names and pid not in received:
                received[pid] = msg.param_value
                out(f"  {pid:25s} = {msg.param_value}")
        missing = [p for p in param_names if p not in received]
        if missing:
            out(f"[UYARI] Parametreler alınamadı: {missing}")
        out("")

    counters = Counter()
    gps_samples = []
    gpi_samples = []
    mode_samples = []
    statustexts = []
    sys_status_samples = []

    deadline = time.monotonic() + args.duration
    out(f"[INFO] {args.duration:.1f}s boyunca mesajlar toplanıyor...")

    while time.monotonic() < deadline:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        mtype = msg.get_type()
        counters[mtype] += 1

        if mtype == "GPS_RAW_INT":
            gps_samples.append({
                "fix_type": msg.fix_type,
                "fix_name": FIX_TYPE_MAPPING.get(msg.fix_type, f"UNK_{msg.fix_type}"),
                "sats": msg.satellites_visible,
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt_mm": msg.alt,
                "eph": msg.eph,
            })
        elif mtype == "GLOBAL_POSITION_INT":
            gpi_samples.append({
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt_mm": msg.alt,
                "hdg_cdeg": msg.hdg,
            })
        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode_samples.append({
                "custom_mode": msg.custom_mode,
                "mode_name": MODE_MAPPING.get(msg.custom_mode, f"UNK_{msg.custom_mode}"),
                "armed": armed,
                "system_status": msg.system_status,
            })
        elif mtype == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode(errors="ignore")
            statustexts.append({"sev": msg.severity, "text": text.strip()})
        elif mtype == "SYS_STATUS":
            sys_status_samples.append({
                "sensors_present": msg.onboard_control_sensors_present,
                "sensors_enabled": msg.onboard_control_sensors_enabled,
                "sensors_health": msg.onboard_control_sensors_health,
                "battery_v_mV": msg.voltage_battery,
            })

    out("")
    out("=" * 60)
    out("  TEŞHİS RAPORU")
    out("=" * 60)

    out(f"[INFO] Mesaj tipi sayımı (top 10):")
    for mtype, cnt in counters.most_common(10):
        out(f"  {mtype:30s}: {cnt}")

    out("")
    out("--- GPS_RAW_INT Analiz ---")
    if not gps_samples:
        out("[SORUN] GPS_RAW_INT HİÇ ALINMADI! GPS zinciri tamamen kırık.")
        out("[ÖNERİ] sitl_sim_gps.parm içinde GPS_TYPE=100 (SITL GPS) olmalı.")
    else:
        last = gps_samples[-1]
        fix_counts = Counter(s["fix_name"] for s in gps_samples)
        out(f"  Sample count: {len(gps_samples)}")
        out(f"  Fix type dağılımı: {dict(fix_counts)}")
        out(f"  Son örnek: fix={last['fix_name']} sats={last['sats']} lat={last['lat']:.7f} lon={last['lon']:.7f} eph={last['eph']}")
        if last["fix_type"] < 3:
            out("[SORUN] Son GPS fix_type < 3 (3D_FIX yok). EKF konum veremez.")
        if last["sats"] < 6:
            out(f"[SORUN] Son GPS satellite sayısı ({last['sats']}) < 6. GUIDED pre-arm başarısız olabilir.")

    out("")
    out("--- GLOBAL_POSITION_INT Analiz ---")
    if not gpi_samples:
        out("[SORUN] GLOBAL_POSITION_INT HİÇ ALINMADI!")
    else:
        last = gpi_samples[-1]
        zero_count = sum(1 for s in gpi_samples if s["lat"] == 0.0 and s["lon"] == 0.0)
        out(f"  Sample count: {len(gpi_samples)} (sıfır lat/lon: {zero_count})")
        out(f"  Son örnek: lat={last['lat']:.7f} lon={last['lon']:.7f} hdg={last['hdg_cdeg']/100:.1f}°")
        if zero_count > 0:
            out(f"[SORUN] {zero_count} örnekte lat/lon=0. EKF konum üretmiyor.")

    out("")
    out("--- HEARTBEAT (Mode/Arm) Analiz ---")
    if not mode_samples:
        out("[SORUN] HEARTBEAT alınmadı!")
    else:
        last = mode_samples[-1]
        mode_counts = Counter(s["mode_name"] for s in mode_samples)
        armed_counts = Counter(s["armed"] for s in mode_samples)
        out(f"  Sample count: {len(mode_samples)}")
        out(f"  Mode dağılımı: {dict(mode_counts)}")
        out(f"  Armed dağılımı: {dict(armed_counts)}")
        out(f"  Son örnek: mode={last['mode_name']} armed={last['armed']} sys_status={last['system_status']}")
        if last["mode_name"] == "MANUAL":
            out("[SORUN] Araç MANUAL moddan çıkmadı. GUIDED/AUTO'ya geçilemedi.")

    out("")
    out("--- STATUSTEXT (Pre-arm ve Uyarılar) ---")
    if not statustexts:
        out("  Hiç STATUSTEXT alınmadı.")
    else:
        unique_texts = {}
        for st in statustexts:
            key = (st["sev"], st["text"])
            unique_texts[key] = unique_texts.get(key, 0) + 1
        for (sev, text), cnt in sorted(unique_texts.items(), key=lambda x: x[0][0]):
            out(f"  sev={sev} ({cnt}x): {text}")

    out("")
    out("--- SYS_STATUS Sensör Sağlığı ---")
    if sys_status_samples:
        last = sys_status_samples[-1]
        # GPS bit: 0x20 (MAV_SYS_STATUS_SENSOR_GPS)
        gps_present = bool(last["sensors_present"] & 0x20)
        gps_enabled = bool(last["sensors_enabled"] & 0x20)
        gps_health = bool(last["sensors_health"] & 0x20)
        out(f"  GPS sensor: present={gps_present} enabled={gps_enabled} health={gps_health}")
        out(f"  Batarya: {last['battery_v_mV']} mV")

    out("")
    out("=" * 60)
    out("  SONUÇ")
    out("=" * 60)
    has_gps = bool(gps_samples) and gps_samples[-1]["fix_type"] >= 3
    has_gpi = bool(gpi_samples) and gpi_samples[-1]["lat"] != 0.0
    has_guided = any(s["mode_name"] in ("GUIDED", "AUTO") for s in mode_samples)

    if not has_gps:
        out("[1] GPS zinciri KIRIK → GPS_TYPE parametresini revize et (JSON SITL için GPS_TYPE=100 öner).")
    if not has_gpi:
        out("[2] GLOBAL_POSITION_INT konum üretmiyor → EKF fix eksik.")
    if not has_guided:
        out("[3] Araç MANUAL moddan çıkmadı → GPS düzelince otomatik çözülmeli.")
    if has_gps and has_gpi and has_guided:
        out("[OK] Tüm zincir sağlıklı. Hareket sorunu başka yerde.")

    out(f"\nLog: {log_file}")


if __name__ == "__main__":
    main()
