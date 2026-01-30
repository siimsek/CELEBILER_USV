#!/bin/bash
# Dosya Yeri: CELEBILER_USV/host_scripts/system_start.sh

CONTAINER_NAME="ege_ros"
# --- LOG YÖNETİMİ ---
# 1. Ana Log Klasörü
LOG_DIR="$HOME/CELEBILER_USV/logs"
mkdir -p "$LOG_DIR"

# 2. Docker Loglarını Buraya Bağla (Symlink)
# Docker içindeki /root/workspace/logs -> Host'ta ./docker_workspace/logs dizinine düşer (Bind Mount varsayımıyla)
DOCKER_LOG_SOURCE="$HOME/CELEBILER_USV/docker_workspace/logs"
if [ -d "$DOCKER_LOG_SOURCE" ]; then
    # Eğer link yoksa veya yanlışsa düzelt (-sfn zorlar)
    ln -sfn "$DOCKER_LOG_SOURCE" "$LOG_DIR/docker"
fi

# 3. Host Log Dosyaları
LOG_FILE="$LOG_DIR/system_boot.log"

# Renk Kodları
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

clear
echo -e "${CYAN}=================================================${NC}"
echo -e "${CYAN}   🚀 ÇELEBİLER USV - YER İSTASYONU BAŞLATICI   ${NC}"
echo -e "${CYAN}=================================================${NC}"
echo "Log Merkezi: $LOG_DIR"
echo "Başlatılıyor..."

# 1. TEMİZLİK
sudo fuser -k 8888/tcp > /dev/null 2>&1

# 2. KAMERA BAŞLATMA (HOST)
echo -e "${GREEN}[KAMERA]${NC} Port 8888 temizleniyor ve yayın başlatılıyor..."
sudo fuser -k 8888/tcp > /dev/null 2>&1
# Sert temizlik
sudo pkill -9 rpicam-vid || true
# Host logu ana log klasörüne
sudo nice -n -20 rpicam-vid -t 0 --codec mjpeg --inline --listen -o tcp://0.0.0.0:8888 --width 1280 --height 720 --framerate 25 --quality 95 > "$LOG_DIR/cam_host.log" 2>&1 &
sleep 2

# 3. IP YAPILANDIRMASI (ZORUNLU 192.168.11.5 EKLEME)
# Mevcut IP'leri al
CURRENT_IPS=$(hostname -I)
TARGET_IP="192.168.11.5"

# Ethernet Arayüzünü Otomatik Bul (eth0, end0, enp3s0 vb.)
ETH_IFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^(e|en|eth)' | grep -v 'lo' | head -n 1)

if [[ -z "$ETH_IFACE" ]]; then
    echo -e "${RED}[AĞ HATASI]${NC} Kablolu Ethernet kartı (eth/end) bulunamadı!"
    echo "       Lütfen Ethernet kablosunu kontrol edin."
else
    if [[ "$CURRENT_IPS" != *"$TARGET_IP"* ]]; then
        echo -e "${YELLOW}[AĞ]${NC} Lidar ağı ($ETH_IFACE) için $TARGET_IP atanıyor..."
        sudo ip addr add 192.168.11.5/24 dev $ETH_IFACE > /dev/null 2>&1 || echo "Uyarı: IP zaten var olabilir."
        sleep 1
        CURRENT_IPS=$(hostname -I) # Güncelle
    else
        echo -e "${GREEN}[AĞ]${NC} Lidar IP ($TARGET_IP) zaten $ETH_IFACE üzerinde aktif."
    fi
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

# --- YENİ EKLENTİ: BAĞLANTI KONTROLÜ (12 Saniye İzle) ---
echo -e "\n🔎 [KONTROL] Donanım Bağlantıları Bekleniyor (12sn)..."
end=$((SECONDS+12))
tail -n 0 -f "$LOG_DIR/docker/telemetry.log" | while read line; do
    if [[ "$line" == *"✅"* ]] || [[ "$line" == *"🎮"* ]] || [[ "$line" == *"💓"* ]]; then
        echo "   -> $line"
    fi
    if [ $SECONDS -ge $end ]; then
        pkill -P $$ tail
        break
    fi
done || true

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
