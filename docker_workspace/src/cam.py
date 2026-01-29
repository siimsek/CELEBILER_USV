import cv2
import time
import numpy as np
import os
import threading
import socket
from flask import Flask, Response

# --- AYARLAR ---
HOST = '127.0.0.1'
PORT = 8888
SAVE_PATH = "/root/workspace/logs/"
WEB_PORT = 5000

# Flask Uygulaması
app = Flask(__name__)

# Global Değişkenler (Threadler arası veri paylaşımı için)
output_frame = np.zeros((360, 640, 3), dtype=np.uint8)
cv2.putText(output_frame, "SYSTEM STARTING...", (100, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
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
    """Kamera yoksa sahte bir kare üretir (360p)"""
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    
    # Hareketli kutular
    t = time.time()
    x1 = int(320 + 150 * np.sin(t))
    y1 = int(180 + 100 * np.cos(t))
    x2 = int(320 + 150 * np.sin(t + 2))
    y2 = int(180 + 100 * np.cos(t + 2))
    
    cv2.rectangle(frame, (x1, y1), (x1+50, y1+50), (0, 0, 255), -1)
    cv2.putText(frame, "SIM_ENGEL", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.rectangle(frame, (x2, y2), (x2+40, y2+75), (0, 255, 0), -1)
    cv2.putText(frame, "SIM_SANCAK", (x2, y2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(frame, "⚠️ NO SIGNAL - SIMULATION", (150, 180), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return frame

def camera_thread():
    """Kamerayı okuyan ve işleyen ana döngü (Socket Based)"""
    global output_frame, SIMULATION_MODE
    
    print(f"📷 [CAM] Yayın aranıyor: {HOST}:{PORT} (Socket Mode)...")
    
    # Kayıt Ayarları
    if not os.path.exists(SAVE_PATH): os.makedirs(SAVE_PATH)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    out = cv2.VideoWriter(f"{SAVE_PATH}USV_{timestamp_str}.avi", fourcc, 30.0, (1280, 720))

    while True:
        # Eğer simülasyon modundaysak ve bağlantı denenmiyorsa
        if SIMULATION_MODE:
            frame = get_simulated_frame()
            with lock:
                output_frame = frame
            
            # Arada bir tekrar bağlanmayı dene
            if int(time.time()) % 5 == 0:
                 SIMULATION_MODE = False # Loop başa dönsün ve bağlanmayı denesin
            else:
                time.sleep(0.05)
                continue

        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(2) # Hızlı fail, simülasyona dön
            client_socket.connect((HOST, PORT))
            connection = client_socket.makefile('rb')
            
            print("✅ [CAM] Kamera Bağlantısı Sağlandı!")
            SIMULATION_MODE = False
            
            stream_bytes = b''
            while True:
                data = connection.read(4096)
                if not data: 
                    break
                stream_bytes += data
                
                first = stream_bytes.find(b'\xff\xd8')
                last = stream_bytes.find(b'\xff\xd9')
                
                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last + 2]
                    stream_bytes = stream_bytes[last + 2:]
                    
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # 1. BOYUT KÜÇÜLTME (PERFORMANS İÇİN KRİTİK)
                        # Processing 720p on CPU is slow. 360p is 4x faster.
                        frame = cv2.resize(frame, (640, 360))

                        # İşleme
                        processed_frame = process_vision(frame)
                        
                        # Bilgi Bas
                        t_now = time.strftime("%H:%M:%S")
                        cv2.rectangle(processed_frame, (0, 0), (640, 35), (0, 0, 0), -1)
                        cv2.putText(processed_frame, f"REC: {t_now} | LIVE | 360p", (10, 25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        
                        out.write(processed_frame)
                        with lock:
                            output_frame = processed_frame.copy()
                            
        except Exception as e:
            # Bağlantı hatası olursa simülasyona geç
            if not SIMULATION_MODE:
                print(f"⚠️ [CAM] Bağlantı Koptu/Hata: {e}")
                print("⚠️ DONANIM YOK - SIMULASYON MODUNDA ÇALIŞIYOR")
            SIMULATION_MODE = True
            time.sleep(1)

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
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")
    os.system(f"lsof -t -i:{port} | xargs kill -9 > /dev/null 2>&1")
    time.sleep(0.5)

@app.route("/")
def video_feed():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    clean_port(WEB_PORT)
    t = threading.Thread(target=camera_thread)
    t.daemon = True
    t.start()
    
    print(f"🌍 WEB ARAYÜZÜ BAŞLATILIYOR: http://0.0.0.0:{WEB_PORT}")
    app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)
