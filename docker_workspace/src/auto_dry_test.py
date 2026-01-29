import time
import sys
import glob
from pymavlink import mavutil

# --- AYARLAR ---
# Raspberry Pi ile Pixhawk arasındaki USB Bağlantı Hızı
BAUD_RATE = 115200 

def find_pixhawk():
    """
    Sistemdeki tüm USB portlarını tarar ve Pixhawk'ı (MAVLink cihazını) bulur.
    """
    print("🔍 Sistem taranıyor (Pixhawk aranıyor)...")
    
    # Potansiyel port listesi (/dev/ttyACM* ve /dev/ttyUSB*)
    potential_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    
    if not potential_ports:
        print("❌ Hiçbir USB cihazı bulunamadı!")
        return None

    for port in potential_ports:
        try:
            print(f"   Testing: {port}...", end=" ")
            # Bağlantıyı dene
            connection = mavutil.mavlink_connection(port, baud=BAUD_RATE)
            
            # Kalp atışı (Heartbeat) bekle - 1 saniye içinde cevap gelmeli
            msg = connection.wait_heartbeat(timeout=1)
            
            if msg:
                print(f"✅ BULUNDU! (Sistem ID: {connection.target_system})")
                return connection
            else:
                print("❌ (Cevap yok)")
                connection.close()
        except Exception as e:
            print(f"❌ Hata: {e}")
            
    return None

def set_servo(master, channel, pwm):
    """
    Pixhawk motor çıkışlarına PWM sinyali gönderir.
    channel: 1 (Sol), 3 (Sağ) - Genelde böyledir
    pwm: 1100 (Geri), 1500 (Dur), 1900 (İleri)
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        channel, pwm, 0, 0, 0, 0, 0
    )

# --- ANA PROGRAM ---
master = find_pixhawk()

if not master:
    print("\n🛑 KRİTİK HATA: Pixhawk bulunamadı! Kabloyu kontrol et.")
    sys.exit()

try:
    print("\n🚀 TEST BAŞLIYOR! (Gözün Mission Planner'da olsun)")
    print("-" * 50)
    
    # 1. Modu GUIDED yap
    print("🛠️  Mod Değiştiriliyor: GUIDED")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()['GUIDED'])
    time.sleep(1)
    
    # 2. ARM Yap (Sanal)
    print("💪 ARM Sinyali Gönderiliyor (Motor Kilidi Açılıyor)...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("✅ ARMED! (Mission Planner'da 'ARMED' yazmalı)")
    time.sleep(2)

    # 3. İLERİ GİT TESTİ
    print("\n🌊 DURUM: İLERİ GİT (PWM 1650)")
    print("   👉 ch1_out (Sol) ve ch3_out (Sağ) YÜKSELMELİ (Grafik yukarı çıkar)")
    set_servo(master, 1, 1650) # Sol Motor İleri
    set_servo(master, 3, 1650) # Sağ Motor İleri
    time.sleep(4)

    # 4. SAĞA DÖN TESTİ (Tank Dönüşü)
    print("\n↪️  DURUM: SAĞA DÖN (Sol İleri, Sağ Dur/Geri)")
    print("   👉 ch1_out (Sol) Yüksek, ch3_out (Sağ) Düşük/1500 olmalı")
    set_servo(master, 1, 1650) # Sol İleri
    set_servo(master, 3, 1500) # Sağ Dur (veya 1400 ile geri döner)
    time.sleep(4)

    # 5. DUR
    print("\n🛑 DURUM: MOTORLARI DURDUR (PWM 1500)")
    set_servo(master, 1, 1500)
    set_servo(master, 3, 1500)
    time.sleep(2)

    # 6. Kapanış
    print("\n🔒 SİSTEM KAPATILIYOR (DISARM)")
    master.arducopter_disarm()
    print("✅ Test Başarıyla Tamamlandı.")

except KeyboardInterrupt:
    print("\n🛑 ACİL DURDURMA!")
    set_servo(master, 1, 1500)
    set_servo(master, 3, 1500)
    master.arducopter_disarm()
