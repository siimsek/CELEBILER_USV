#!/bin/bash
# Dosya Yeri: CELEBILER_USV/host_scripts/system_start.sh

CONTAINER_NAME="ege_ros"
echo "=== CELEBILER USV BASLATILIYOR ==="

# 1. IP Kontrolü
if ! ip addr show eth0 | grep "192.168.11.5" > /dev/null; then
    echo "[HOST] IP atanıyor: 192.168.11.5..."
    sudo ip addr add 192.168.11.5/24 dev eth0
else
    echo "[HOST] IP ayarı zaten tamam."
fi

# 2. Kamera Yayını (720p TCP)
pkill rpicam-vid || true
echo "[HOST] Kamera yayını başlatılıyor..."
rpicam-vid -t 0 --codec mjpeg --inline --listen -o tcp://0.0.0.0:8888 --width 1280 --height 720 --framerate 30 > /dev/null 2>&1 &

# 3. Docker Başlatma
if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    docker start $CONTAINER_NAME
fi

# 4. İçeriye Emir Ver (Internal Scripti Çalıştır)
echo "[HOST] Docker içi otonom sistem tetikleniyor..."
docker exec -it $CONTAINER_NAME /bin/bash -c "/root/workspace/scripts/internal_start.sh"

# Çıkışta temizlik
pkill rpicam-vid
echo "[HOST] Sistem kapatıldı."
