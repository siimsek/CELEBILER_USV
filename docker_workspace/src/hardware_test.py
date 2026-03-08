import time
import sys
import glob
from pymavlink import mavutil
import argparse

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
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        ch1, ch2, ch3, ch4, 65535, 65535, 65535, 65535)

def set_servo(master, channel, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        channel, pwm, 0, 0, 0, 0, 0
    )

def test_rc_override():
    master = find_pixhawk()
    if not master: return
    try:
        master.mav.set_mode_send(
            master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            master.mode_mapping()['MANUAL'])
        time.sleep(1)
        master.arducopter_arm()
        master.motors_armed_wait()
        
        print("🌊 HAREKET: İLERİ GİT")
        start_time = time.time()
        while time.time() - start_time < 3:
            set_rc_channel_pwm(master, ch1=1500, ch3=1700)
            time.sleep(0.2)
            
        print("🛑 DURUYOR...")
        set_rc_channel_pwm(master, ch1=1500, ch3=1500)
        time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        set_rc_channel_pwm(master, ch1=1500, ch3=1500)
        master.arducopter_disarm()

def test_servo_direct():
    master = find_pixhawk()
    if not master: return
    try:
        master.mav.set_mode_send(
            master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            master.mode_mapping()['GUIDED'])
        time.sleep(1)
        master.arducopter_arm()
        master.motors_armed_wait()
        
        print("🌊 İLERİ")
        set_servo(master, 1, 1650)
        set_servo(master, 3, 1650)
        time.sleep(4)
        
        print("↪️ SAĞA DÖN")
        set_servo(master, 1, 1650)
        set_servo(master, 3, 1500)
        time.sleep(4)
        
        print("🛑 DUR")
        set_servo(master, 1, 1500)
        set_servo(master, 3, 1500)
    except KeyboardInterrupt:
        pass
    finally:
        set_servo(master, 1, 1500)
        set_servo(master, 3, 1500)
        master.arducopter_disarm()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "rc":
        test_rc_override()
    elif len(sys.argv) > 1 and sys.argv[1] == "servo":
        test_servo_direct()
    else:
        print("Kullanım: python3 hardware_test.py [rc|servo]")
