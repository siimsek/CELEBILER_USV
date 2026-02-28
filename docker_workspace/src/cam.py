import cv2
import time
import socket
import numpy as np
import threading
import os
import sys
from flask import Flask, Response
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Gereksiz logları kapat

# --- YARIŞMA MODU (IDA 3.7 - YKİ'ye görüntü aktarımı yasak) ---
# ANCAK: Kamera İDA üzerinde engelden kaçınma ve hedef tespiti için çalışmaya devam eder.
# Sadece Flask web yayını (port 5000) kapatılır.
RACE_MODE = os.environ.get('USV_MODE') == 'race'
if RACE_MODE:
    print("🏁 [cam.py] YARIŞMA MODU — Web yayını KAPALI, onboard işleme AKTİF")

# --- AYARLAR ---
HOST = '127.0.0.1'
PORT = 8888
WEB_PORT = 5000
SAVE_PATH = "/root/workspace/logs/video/"

# --- RENK TANIMLAMALARI ---
COLOR_RANGES = {
    "SARI_ENGEL": {
        "lower": np.array([20, 100, 100]),
        "upper": np.array([35, 255, 255]),
        "color": (0, 255, 255)
    },
    "YESIL_SANCAK": {
        "lower": np.array([40, 80, 80]),
        "upper": np.array([85, 255, 255]),
        "color": (0, 255, 0)
    },
    "KIRMIZI_SANCAK": {
        "lower": np.array([0, 150, 100]),
        "upper": np.array([10, 255, 255]),
        "lower2": np.array([170, 150, 100]),
        "upper2": np.array([180, 255, 255]),
        "color": (0, 0, 255)
    }
}

def clean_port(port):
    """Portu kullanan süreci (PID) bulur ve öldürür."""
    import os
    print(f"🧹 Port {port} temizleniyor...")
    # Fuser ile zorla (-k -9) öldür
    os.system(f"fuser -k -9 {port}/tcp > /dev/null 2>&1")
    time.sleep(1) # Portun boşa düşmesi için bekle

class VideoCamera:
    """Basit, Kanıtlanmış Kamera Okuyucu"""
    def __init__(self):
        self.frame = None
        self.connected = False
        self.stopped = False
        print(f"📡 Kamera Sistemi Başlatılıyor (Host: {HOST}:{PORT})...")
        
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        """Tek bir thread: Soket'ten oku -> self.frame'e yaz. Hepsi bu."""
        while not self.stopped:
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                client_socket.settimeout(5)
                client_socket.connect((HOST, PORT))
                connection = client_socket.makefile('rb')
                self.connected = True
                print("✅ Kamera Bağlantısı Sağlandı!")

                stream_bytes = b''
                while not self.stopped:
                    data = connection.read(4096)
                    if not data: break
                    stream_bytes += data
                    
                    first = stream_bytes.find(b'\xff\xd8')
                    last = stream_bytes.find(b'\xff\xd9')

                    if first != -1 and last != -1:
                        jpg = stream_bytes[first:last + 2]
                        stream_bytes = stream_bytes[last + 2:]
                        image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if image is not None:
                            # ÇÖZÜNÜRLÜK: 720p (Orijinal gelen görüntü zaten 720p ise resize'a gerek yok)
                            h, w = image.shape[:2]
                            if (w, h) != (1280, 720):
                                image = cv2.resize(image, (1280, 720))
                            self.frame = image
                            
            except Exception as e:
                self.connected = False
                print(f"⚠️ Kamera Bağlantısı Yok: {e}")
                time.sleep(2)

    def process_frame(self, frame):
        """Görüntü İşleme (Renk Tespiti)"""
        if self.frame is None: return frame

        # OPTIMIZASYON: İşleme (Renk Tespiti) küçük resimde yapılır (0.5x)
        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        blurred = cv2.GaussianBlur(small_frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        for name, params in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, params["lower"], params["upper"])
            if "lower2" in params:
                mask2 = cv2.inRange(hsv, params["lower2"], params["upper2"])
                mask = cv2.bitwise_or(mask, mask2)

            # Morfolojik işlemler (daha küçük kernel)
            kernel = np.ones((3,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Küçük resimde 1000/4 = 250 alan eşiği
                if area > 250:
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Koordinatları orijinal boyuta (2x) çevir
                    x, y, w, h = x*2, y*2, w*2, h*2
                    
                    cv2.rectangle(frame, (x, y), (x + w, y + h), params["color"], 2)
                    cv2.putText(frame, name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, params["color"], 2)

        # HUD
        h, w = frame.shape[:2]
        timestamp = time.strftime("%H:%M:%S")
        cv2.rectangle(frame, (0, 0), (w, 35), (0, 0, 0), -1)
        cv2.putText(frame, f"REC: {timestamp} | FPS: 30 | LIVE", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Merkez Nişangah
        cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (200, 200, 200), 2)
        cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (200, 200, 200), 2)
        
        return frame

    def get_frame(self):
        if not self.connected or self.frame is None:
            # Simülasyon Karesi (720p)
            sim = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(sim, "KAMERA BEKLENIYOR...", (450, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            ret, jpeg = cv2.imencode('.jpg', sim)
            return jpeg.tobytes()
        
        processed = self.process_frame(self.frame.copy())
        ret, jpeg = cv2.imencode('.jpg', processed, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        return jpeg.tobytes()

# Flask App
app = Flask(__name__)
camera_stream = None

@app.route('/')
def video_feed():
    def generate():
        while True:
            frame = camera_stream.get_frame()
            if frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033) # ~30 FPS limiti
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    clean_port(WEB_PORT)
    camera_stream = VideoCamera().start()

    if RACE_MODE:
        # YARIŞMA MODU: Kamera çalışıyor (onboard engel tespiti için)
        # ama web yayını YOK (Şartname 3.7: YKİ'ye görüntü aktarımı yasak)
        print("📷 [cam.py] Kamera onboard çalışıyor — Web yayını KAPALI")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("📷 [cam.py] Kapatılıyor...")
    else:
        # TEST MODU: Tam web yayını
        print(f"🌍 WEB ARAYÜZÜ BAŞLATILIYOR: http://0.0.0.0:{WEB_PORT}")
        app.run(host='0.0.0.0', port=WEB_PORT, threaded=True)

