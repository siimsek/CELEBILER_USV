import time
import sys
import glob
from pymavlink import mavutil

# --- AYARLAR ---
BAUD_RATE = 115200 

def find_pixhawk():
    print("🔍 Pixhawk aranıyor...")
    potential_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    
    for port in potential_ports:
        try:
            master = mavutil.mavlink_connection(port, baud=BAUD_RATE)
            master.wait_heartbeat(timeout=1)
            print(f"✅ BULUNDU! ({port})")
            return master
        except:
            pass
    return None

def set_rc_channel_pwm(master, ch1=65535, ch2=65535, ch3=65535, ch4=65535):
    """
    Sanal Kumanda Sinyali Gönderir.
    65535 = Kontrol etme (Boş bırak)
    1100-1900 = PWM Değeri
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        ch1, ch2, ch3, ch4, 
        65535, 65535, 65535, 65535) # Kanal 5-8 boş

# --- ANA PROGRAM ---
master = find_pixhawk()
if not master:
    print("❌ Pixhawk Yok!")
    sys.exit()

try:
    print("\n🎮 SANAL KUMANDA TESTİ BAŞLIYOR")
    print("-" * 50)
    
    # 1. Modu MANUAL yap (Override testi için en iyisi MANUAL'dir)
    print("🛠️  Mod: MANUAL yapılıyor...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()['MANUAL'])
    time.sleep(1)
    
    # 2. ARM Yap
    print("💪 ARM Sinyali...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("✅ ARMED! (Motorlar aktif)")
    time.sleep(1)

    # 3. İLERİ GİT (Kanal 3 genelde gazdır, Kanal 1 yöndür)
    # Rover'da Skid Steering'de Throttle (Ch3) ve Steering (Ch1) karıştırılır.
    # İkisini de ileri itelim.
    print("\n🌊 HAREKET: İLERİ GİT (3 Saniye)")
    print("   👉 Mission Planner: ch1_out ve ch3_out YÜKSELMELİ")
    
    start_time = time.time()
    while time.time() - start_time < 3:
        # Kanal 1 (Roll/Steer) = 1500 (Düz)
        # Kanal 3 (Throttle)   = 1700 (İleri Gaz)
        set_rc_channel_pwm(master, ch1=1500, ch3=1700)
        time.sleep(0.2) # Sürekli sinyal göndermek gerekir

    # 4. DUR
    print("\n🛑 DURUYOR...")
    set_rc_channel_pwm(master, ch1=1500, ch3=1500)
    time.sleep(1)

    # 5. DISARM
    print("🔒 DISARM")
    master.arducopter_disarm()

except KeyboardInterrupt:
    set_rc_channel_pwm(master, ch1=1500, ch3=1500)
    master.arducopter_disarm()
