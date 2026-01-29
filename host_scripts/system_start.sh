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

# 1. TEMİZLİK
sudo fuser -k 8888/tcp > /dev/null 2>&1


# 2. KAMERA BAŞLATMA (HOST) (Sıra değişti, IP ayarı aşağıda)
echo -e "${GREEN}[KAMERA]${NC} Port 8888 temizleniyor ve yayın başlatılıyor..."
sudo fuser -k 8888/tcp > /dev/null 2>&1
pkill rpicam-vid || true
rpicam-vid -t 0 --codec mjpeg --inline --listen -o tcp://0.0.0.0:8888 --width 1280 --height 720 --framerate 30 > /dev/null 2>&1 &
sleep 2

# 3. IP YAPILANDIRMASI (ZORUNLU 192.168.11.5 EKLEME)
# Mevcut IP'leri al
CURRENT_IPS=$(hostname -I)
TARGET_IP="192.168.11.5"

if [[ "$CURRENT_IPS" != *"$TARGET_IP"* ]]; then
    echo -e "${YELLOW}[AĞ]${NC} Lidar ağı için $TARGET_IP atanıyor..."
    sudo ip addr add 192.168.11.5/24 dev eth0 > /dev/null 2>&1 || echo "Uyarı: eth0 bulunamadı veya IP eklenemedi."
    sleep 1
    CURRENT_IPS=$(hostname -I) # Güncelle
else
    echo -e "${GREEN}[AĞ]${NC} Lidar IP ($TARGET_IP) mevcut."
fi

# 4. DOCKER BAŞLATMA
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

# 5. İÇ SCRİPTİ TETİKLEME
echo -e "${GREEN}[SİSTEM]${NC} Otonom Pilot ve Web Sunucular Başlatılıyor..."
docker exec -d $CONTAINER_NAME /bin/bash -c "/root/workspace/scripts/internal_start.sh"

# 6. FİNAL BİLGİ TABLOSU
sleep 3
echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║              ✅ SİSTEM BAŞARIYLA AÇILDI!                     ║${NC}"
echo -e "${CYAN}╠══════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} 🌍 ERİŞİM ADRESLERİ (Ağınıza uygun olanı seçin):            ${CYAN}║${NC}"

# Tüm IP'leri listele
for ip in $CURRENT_IPS; do
    # 127.0.0.1 hariç, 172. (docker) hariç tutmaya çalışalım ama hostname -I zaten temiz verir genelde
    if [[ $ip != "127."* ]]; then
        echo -e "${CYAN}║${NC}    👉 http://$ip:8080 (Dashboard)                     ${CYAN}║${NC}"
    fi
done

echo -e "${CYAN}╠══════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} 📷 KAMERA: Port 5000  |  🗺️  HARİTA: Port 5001              ${CYAN}║${NC}" 
echo -e "${CYAN}╠══════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} 📡 Lidar IP: 192.168.11.2                                 ${CYAN}║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Not: Kapatmak için terminali kapatabilirsin, sistem arkada çalışır.${NC}"
echo -e "${YELLOW}Tamamen durdurmak için: 'docker stop ege_ros' yaz.${NC}"
