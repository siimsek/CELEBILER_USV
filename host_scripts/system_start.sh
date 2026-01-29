#!/bin/bash
# Dosya Yeri: CELEBILER_USV/host_scripts/system_start.sh

CONTAINER_NAME="ege_ros"
LOG_FILE="/home/admin/CELEBILER_USV/system_boot.log"

# Renk Kodları (Terminalde havalı dursun diye)
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

clear
echo -e "${CYAN}=================================================${NC}"
echo -e "${CYAN}   🚀 ÇELEBİLER USV - YER İSTASYONU BAŞLATICI   ${NC}"
echo -e "${CYAN}=================================================${NC}"
echo "Log Dosyası: $LOG_FILE"
echo "Başlatılıyor..."

# 1. IP KONTROLÜ
HOST_IP=$(hostname -I | awk '{print $1}')
TARGET_IP="192.168.11.5"

if [[ "$HOST_IP" != *"$TARGET_IP"* ]]; then
    echo -e "${YELLOW}[AĞ]${NC} IP ($TARGET_IP) atanıyor..."
    sudo ip addr add 192.168.11.5/24 dev eth0
    HOST_IP="192.168.11.5"
    sleep 1
else
    echo -e "${GREEN}[AĞ]${NC} IP Adresi Doğru: $HOST_IP"
fi

# 2. KAMERA BAŞLATMA (HOST)
echo -e "${GREEN}[KAMERA]${NC} Port 8888 temizleniyor ve yayın başlatılıyor..."

# Sadece 8888 portunu kullanan işlemi öldür (Surgical Kill)
sudo fuser -k 8888/tcp > /dev/null 2>&1
pkill rpicam-vid || true # Garanti olsun diye ismen de öldür

# Kamerayı başlat
rpicam-vid -t 0 --codec mjpeg --inline --listen -o tcp://0.0.0.0:8888 --width 1280 --height 720 --framerate 30 > /dev/null 2>&1 &
sleep 2

# 3. DOCKER BAŞLATMA
echo -e "${GREEN}[DOCKER]${NC} Konteyner ($CONTAINER_NAME) Kontrol Ediliyor..."

if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
        docker start $CONTAINER_NAME > /dev/null
        echo -e "${GREEN}[DOCKER]${NC} Konteyner Uyandırıldı."
    else
        echo -e "${RED}[HATA]${NC} Konteyner bulunamadı! Lütfen kurulum yapın."
        exit 1
    fi
fi

# 4. İÇ SCRİPTİ TETİKLEME
echo -e "${GREEN}[SİSTEM]${NC} Otonom Pilot ve Web Sunucular Başlatılıyor..."
docker exec -d $CONTAINER_NAME /bin/bash -c "/root/workspace/scripts/internal_start.sh"

# 5. FİNAL BİLGİ TABLOSU (İŞTE BURASI)
sleep 2 # Servislerin açılmasına biraz izin ver

echo ""
echo -e "${CYAN}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║           ✅ SİSTEM BAŞARIYLA AÇILDI!              ║${NC}"
echo -e "${CYAN}╠════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} 🌍 ${YELLOW}ANA DASHBOARD:${NC}   http://$HOST_IP:8080        ${CYAN}║${NC} (Telemetri + Hepsi)"
echo -e "${CYAN}║${NC} 📷 ${YELLOW}KAMERA YAYINI:${NC}   http://$HOST_IP:5000   ${CYAN}║${NC} (Sadece Görüntü)"
echo -e "${CYAN}║${NC} 🗺️  ${YELLOW}LIDAR HARİTA:${NC}    http://$HOST_IP:5001   ${CYAN}║${NC} (Sadece Harita)"
echo -e "${CYAN}╠════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} 📡 ${YELLOW}Lidar IP:${NC}        192.168.11.2           ${CYAN}║${NC}"
echo -e "${CYAN}║${NC} 🔌 ${YELLOW}Host IP:${NC}         $HOST_IP           ${CYAN}║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Not: Kapatmak için terminali kapatabilirsin, sistem arkada çalışır.${NC}"
echo -e "${YELLOW}Tamamen durdurmak için: 'docker stop ege_ros' yaz.${NC}"
