import time
import sys
import math
from pymavlink import mavutil

# --- AYARLAR ---
TARGET_HEADING = 0  # Hedefimiz KUZEY (0 derece)
SPEED_PWM = 1650    # İleri Gaz Şiddeti (1500=Dur, 1900=Tam Gaz)
TURN_SPEED = 200    # Dönüş sertliği (PWM farkı)
CONNECTION_STRING = '/dev/ttyACM1' # Veya ACM0, senin portun hangisiyse
BAUD_RATE = 115200

class IdaRobot:
    def __init__(self):
        print(f"🔌 Bağlanılıyor: {CONNECTION_STRING}...")
        self.master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        self.master.wait_heartbeat()
        print("✅ Pixhawk Bağlandı!")
        self.current_heading = 0

    def get_heading(self):
        """Pusula verisini (Heading) çeker"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            # hdg verisi cdeg (santiderece) gelir, 100'e bölmeliyiz
            self.current_heading = msg.hdg / 100.0
        return self.current_heading

    def set_motor(self, left_pwm, right_pwm):
        """Sanal Kumanda ile Motorları Sürer"""
        # Kanal 1: Steering (Yön), Kanal 3: Throttle (Gaz)
        # Skid Steering (Tank) mantığında bu ikisi karıştırılır.
        # Basitlik için Kanal 1 ve 3'ü direkt override ediyoruz.
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            int(left_pwm),  # Kanal 1 (Sol Motor Varsayalım)
            0,              # Kanal 2 (Boş)
            int(right_pwm), # Kanal 3 (Sağ Motor Varsayalım)
            0, 0, 0, 0, 0)

    def run(self):
        try:
            print("🛠️  Mod: MANUAL (Güvenlik İçin)")
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.master.mode_mapping()['MANUAL'])
            time.sleep(1)

            print("💪 ARM Yapılıyor...")
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            print("🚀 OTONOM SÜRÜŞ BAŞLADI! (Hedef: KUZEY)")

            while True:
                # 1. Nereye Bakıyorum?
                heading = self.get_heading()
                
                # 2. Hata Ne Kadar? (0 ile 360 arası fark)
                error = TARGET_HEADING - heading
                
                # Açıyı -180 ile +180 arasına sıkıştır (En kısa dönüş yolu)
                if error > 180: error -= 360
                if error < -180: error += 360

                # 3. Karar Ver (PD Kontrolün P'si)
                # Düz gitmek için temel gaz: SPEED_PWM
                # Dönmek için motorlara fark ekle
                
                # Basit Mantık:
                # Hata Pozitif (+) -> Hedef Sağımda -> Sağa Dön (Solu artır, Sağı azalt)
                # Hata Negatif (-) -> Hedef Solumda -> Sola Dön (Sağı artır, Solu azalt)
                
                correction = error * 2 # Düzeltme katsayısı (P Gain)
                
                # PWM Sınırla (1100 - 1900 arası)
                left_motor = SPEED_PWM + correction
                right_motor = SPEED_PWM - correction
                
                # Güvenlik Sınırları
                left_motor = max(1100, min(1900, left_motor))
                right_motor = max(1100, min(1900, right_motor))

                # 4. Uygula
                self.set_motor(left_motor, right_motor)
                
                # Bilgi Bas
                print(f"🧭 Yön: {heading:.1f}° | Hata: {error:.1f}° | Motorlar: L{int(left_motor)} R{int(right_motor)}")
                
                time.sleep(0.1) # 10Hz döngü hızı

        except KeyboardInterrupt:
            print("\n🛑 ACİL DURDURMA!")
            self.set_motor(1500, 1500) # Motorları durdur
            self.master.arducopter_disarm()

if __name__ == "__main__":
    bot = IdaRobot()
    bot.run()
