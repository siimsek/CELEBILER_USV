import cv2
import time
import numpy as np
import os
import threading
from flask import Flask, Response

# --- AYARLAR ---
STREAM_URL = "tcp://127.0.0.1:8888" # Host'tan gelen yayın
SAVE_PATH = "/root/workspace/logs/"
WEB_PORT = 5000

# Flask Uygulaması
app = Flask(__name__)

# Global Değişkenler (Threadler arası veri paylaşımı için)
output_frame = None
lock = threading.Lock()

# --- MOD SEÇİMİ ---
TEST_MODE = True
SIMULATION_MODE = False

# --- RENK ARALIKLARI ---
COLOR_RANGES = {
    "SARI_ENGEL": {"lower": np.array([20, 100, 100]), "upper": np.array([35, 255, 255]), "color": (0, 255, 255)},
    "YESIL_SANCAK": {"lower": np.array([40, 50, 50]), "upper": np.array([90, 255, 255]), "color": (0, 255, 0)}
}
RED_LOWER1 = np.array([0, 70, 50]); RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 70, 50]); RED_UPPER2 = np.array([180, 255, 255])

def process_vision(frame):
    """Görüntü İşleme ve Çizim Fonksiyonu"""
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    detected_objects = []

    # Kırmızı Tespiti
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    mask_red = cv2.bitwise_or(mask1, mask2)
    detected_objects.append(("KIRMIZI_ISKELE", mask_red, (0, 0, 255)))

    # Diğer Renkler
    for name, params in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, params["lower"], params["upper"])
        detected_objects.append((name, mask, params["color"]))

    for name, mask, color in detected_objects:
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if TEST_MODE:
                    cv2.circle(frame, (int(x), int(y)), int(radius), color, 2)
                    cv2.putText(frame, f"{name}", (int(x)-20, int(y)-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame

def get_simulated_frame():
    """Kamera yoksa sahte bir kare üretir"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    
    # Hareketli kutular için basit mantık
    t = time.time()
    x1 = int(640 + 300 * np.sin(t))
    y1 = int(360 + 200 * np.cos(t))
    
    x2 = int(640 + 300 * np.sin(t + 2))
    y2 = int(360 + 200 * np.cos(t + 2))
    
    # Kırmızı Engel
    cv2.rectangle(frame, (x1, y1), (x1+100, y1+100), (0, 0, 255), -1)
    cv2.putText(frame, "SIM_ENGEL", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    # Yeşil Sancak
    cv2.rectangle(frame, (x2, y2), (x2+80, y2+150), (0, 255, 0), -1)
    cv2.putText(frame, "SIM_SANCAK", (x2, y2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Uyarı Metni
    cv2.putText(frame, "⚠️ KAMERA BAGLANTISI YOK - SIMULASYON MODU", (300, 360), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
    
    return frame

def camera_thread():
    """Kamerayı okuyan ve işleyen ana döngü"""
    global output_frame, SIMULATION_MODE
    
    print(f"📷 [CAM] Yayın aranıyor: {STREAM_URL}...")
    
    cap = None
    try:
        cap = cv2.VideoCapture(STREAM_URL)
        if not cap.isOpened():
            raise Exception("Kamera açılamadı")
    except Exception as e:
        print(f"⚠️ [CAM] Bağlantı Hatası: {e}")
        SIMULATION_MODE = True
        print("⚠️ DONANIM YOK - SIMULASYON MODUNDA ÇALIŞIYOR")
    
    # Kayıt Ayarları
    if not os.path.exists(SAVE_PATH): os.makedirs(SAVE_PATH)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    out = cv2.VideoWriter(f"{SAVE_PATH}USV_{timestamp_str}.avi", fourcc, 30.0, (1280, 720))

    print(f"✅ [CAM] Kamera Modu: {'SİMÜLASYON' if SIMULATION_MODE else 'CANLI'}")

    while True:
        if SIMULATION_MODE:
            frame = get_simulated_frame()
            time.sleep(0.05) # ~20 FPS simulation
        else:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ [CAM] Sinyal koptu, yeniden bağlanılıyor...")
                cap.release()
                time.sleep(1)
                try:
                    cap = cv2.VideoCapture(STREAM_URL)
                    if not cap.isOpened(): raise Exception("Fail")
                except:
                    print("⚠️ [CAM] Yeniden bağlanamadı -> Simülasyona geçiliyor")
                    SIMULATION_MODE = True
                continue

        # Görüntüyü İşle
        # Simülasyon değilse işle, simülasyonda zaten çizdik
        processed_frame = process_vision(frame) if not SIMULATION_MODE else frame

        # Bilgi Bas (Overlay)
        t_now = time.strftime("%H:%M:%S")
        cv2.rectangle(processed_frame, (0, 0), (1280, 40), (0, 0, 0), -1)
        mode_str = "SIMULATION" if SIMULATION_MODE else "LIVE"
        cv2.putText(processed_frame, f"REC: {t_now} | EGE USV WEB VIEW | MODE: {mode_str}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Kaydet
        out.write(processed_frame)

        # Web Yayını İçin Kopyala
        with lock:
            output_frame = processed_frame.copy()

def generate():
    """Web tarayıcısına kare kare resim gönderir"""
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                time.sleep(0.1)
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encodedImage) + b'\r\n')
        time.sleep(0.03) # 30 FPS limiti

def clean_port(port):
    print(f"🧹 Port {port} temizleniyor...")
    # Robust kill (bazen fuser yetmez)
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")
    os.system(f"lsof -t -i:{port} | xargs kill -9 > /dev/null 2>&1")
    time.sleep(0.5)

@app.route("/")
def video_feed():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    clean_port(5000)
    # Önce kamera işini arka planda başlat
    t = threading.Thread(target=camera_thread)
    t.daemon = True
    t.start()
    
    # Web sunucusunu başlat (Burası kodu bloklar)
    print(f"🌍 WEB ARAYÜZÜ BAŞLATILIYOR: http://0.0.0.0:{WEB_PORT}")
    app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)
