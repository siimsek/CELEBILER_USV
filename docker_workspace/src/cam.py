import cv2
import time
import socket
import numpy as np
import threading
from flask import Flask, Response

# --- AYARLAR ---
HOST = '127.0.0.1'
PORT = 8888

# --- GELİŞMİŞ RENK TANIMLAMALARI (SIKI FİLTRELER) ---
# Siyahın algılanmaması için 'V' (Parlaklık) ve 'S' (Doygunluk) alt limitleri yükseltildi.
# OpenCV HSV Aralıkları: H: 0-179, S: 0-255, V: 0-255
COLOR_RANGES = {
    "SARI (ENGEL)": {
        # Sarı genelde parlaktır, V en az 100 olsun
        "lower": np.array([20, 100, 100]),
        "upper": np.array([35, 255, 255]),
        "color": (0, 255, 255) # BGR: Sarı
    },
    "MAVI (HEDEF)": {
        # Koyu lacivert ile siyah karışmasın diye V limitini 80 yaptık
        "lower": np.array([100, 150, 80]),
        "upper": np.array([140, 255, 255]),
        "color": (255, 0, 0)   # BGR: Mavi
    },
    "YESIL (HEDEF)": {
        # SIYAH SORUNUNU ÇÖZEN AYAR BURADA
        # V (Parlaklık) en az 80, S (Doygunluk) en az 80 olmalı. 
        # Böylece siyah veya gri tonları yeşil sanılmaz.
        "lower": np.array([40, 80, 80]),
        "upper": np.array([85, 255, 255]),
        "color": (0, 255, 0)   # BGR: Yeşil
    },
    "KIRMIZI (YASAK)": {
        "lower": np.array([0, 150, 100]),
        "upper": np.array([10, 255, 255]),
        "lower2": np.array([170, 150, 100]),
        "upper2": np.array([180, 255, 255]),
        "color": (0, 0, 255)   # BGR: Kırmızı
    }
}

class VideoCamera(object):
    def __init__(self):
        self.frame = None
        self.stopped = False
        self.connected = False
        print(f"📡 EGE İDA Görüntü Sistemi Başlatılıyor...")

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
                            self.frame = image
            except Exception:
                self.connected = False
                time.sleep(2)

    def process_frame(self, frame):
        # 1. Ön İşleme: Hafif Bulanıklaştırma (Gürültüyü azaltır)
        blurred = cv2.GaussianBlur(frame, (9, 9), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 2. Renk Taraması
        for name, params in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, params["lower"], params["upper"])
            if "lower2" in params:
                mask2 = cv2.inRange(hsv, params["lower2"], params["upper2"])
                mask = cv2.bitwise_or(mask, mask2)

            # Morfolojik Açılış (Erosion + Dilation): Beyaz noktacıkları (gürültü) siler
            # Bu işlem hatalı tespitleri ciddi oranda düşürür
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                # 1000 pikselden küçük alanları görmezden gel (Küçük lekeleri ele)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), params["color"], 2)
                    
                    # Yazı Arka Planı (Okunabilirlik için)
                    label_size, baseline = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(frame, (x, y - label_size[1] - 10), (x + label_size[0], y), params["color"], -1)
                    cv2.putText(frame, name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 3. HUD ve Bilgi Ekranı (Şartname Gereği Zaman Etiketi)
        h, w = frame.shape[:2]
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        
        # Üst Siyah Bar
        cv2.rectangle(frame, (0, 0), (w, 35), (0, 0, 0), -1)
        cv2.putText(frame, f"REC: {timestamp}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, "EGE IDA AI SYSTEM", (w - 200, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Merkez Nişangah
        cv2.line(frame, (w//2 - 15, h//2), (w//2 + 15, h//2), (200, 200, 200), 2)
        cv2.line(frame, (w//2, h//2 - 15), (w//2, h//2 + 15), (200, 200, 200), 2)
        
        return frame

    def get_frame(self):
        if not self.connected or self.frame is None: return None
        processed = self.process_frame(self.frame.copy())
        ret, jpeg = cv2.imencode('.jpg', processed, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        return jpeg.tobytes()

app = Flask(__name__)
camera_stream = VideoCamera().start()

@app.route('/')
def video_feed():
    def generate():
        while True:
            frame = camera_stream.get_frame()
            if frame is not None:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                time.sleep(0.1)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 0.0.0.0 ile dışarıya açıyoruz
    app.run(host='0.0.0.0', port=5000, threaded=True)
