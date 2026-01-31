#!/bin/bash
# Dosya Yeri: CELEBILER_USV/host_scripts/system_start.sh

CONTAINER_NAME="ege_ros"

# Renk Kodları
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

clear
echo -e "${CYAN}=================================================${NC}"
echo -e "${CYAN}   🚀 ÇELEBİLER USV - YER İSTASYONU BAŞLATICI   ${NC}"
echo -e "${CYAN}=================================================${NC}"

LOG_DIR="$HOME/CELEBILER_USV/logs"
echo "🧹 [HOST] Eski loglar temizleniyor..."
sudo rm -f "$LOG_DIR"/*.log
sudo rm -f "$LOG_DIR"/docker/*.log
mkdir -p "$LOG_DIR/docker"

# --- 0. AĞ YAPILANDIRMASI (ÖNCELİKLİ) ---
# Modemin resetlenmesi durumunda Lidar IP'sini (192.168.11.5) kaybetmemek için en başta yapıyoruz.
TARGET_IP="192.168.11.5"
CURRENT_IPS=$(hostname -I)

# Ethernet Arayüzünü Bul (eth0, end0, enp3s0 vb.)
ETH_IFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^(e|en|eth)' | grep -v 'lo' | head -n 1)

if [[ -z "$ETH_IFACE" ]]; then
    echo -e "${RED}[AĞ HATASI]${NC} Ethernet kartı bulunamadı! Kabloyu kontrol edin."
else
    # IP var mı kontrol et
    if [[ "$CURRENT_IPS" != *"$TARGET_IP"* ]]; then
        echo -e "${YELLOW}[AĞ]${NC} Lidar ağı ($ETH_IFACE) için $TARGET_IP atanıyor..."
        sudo ip addr add 192.168.11.5/24 dev $ETH_IFACE > /dev/null 2>&1 || true
        sleep 1
        CURRENT_IPS=$(hostname -I) # Güncelle
    else
        echo -e "${GREEN}[AĞ]${NC} Lidar IP ($TARGET_IP) zaten $ETH_IFACE üzerinde aktif."
    fi
fi

# --- 1. DONANIM ÖN KONTROLÜ (Pre-Flight Check) ---
echo -e "\n🛡️  [GÜVENLİK] Donanım Bağlantıları Kontrol Ediliyor..."

# A) LIDAR KONTROLÜ
if ping -c 1 -W 1 192.168.11.2 > /dev/null; then
    echo "   ✅ Lidar (192.168.11.2) - BAĞLI"
else
    echo -e "   ❌ [HATA] Lidar Bulunamadı! (IP: 192.168.11.2 Erişim Yok)"
    echo "      -> Modem veya Switch resetlenmiş olabilir."
    echo "      -> Statik IP ataması ($TARGET_IP) yapıldı ama Lidar cevap vermiyor."
    exit 1
fi

# B) SERİ PORT KONTROLÜ (Pixhawk + STM32)
port_count=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | wc -l)
if [ "$port_count" -ge 2 ]; then
    echo "   ✅ Seri Portlar ($port_count Aygıt) - TAMAM"
else
    echo -e "   ❌ [HATA] Eksik Seri Cihaz! (Bulunan: $port_count, Beklenen: 2+)"
    echo "      -> Pixhawk ve STM32 USB kablolarını kontrol et."
    echo "      -> Bulunanlar: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null)"
    exit 1
fi

# C) KAMERA KONTROLÜ
if rpicam-vid --list-cameras > /dev/null 2>&1; then
    echo "   ✅ Kamera Modülü - BAĞLI"
else
    echo -e "   ❌ [HATA] Kamera Algılanmadı! (rpicam-vid listeleme başarısız)"
    echo "      -> Şerit kabloyu (csi/dsi) kontrol et."
    exit 1
fi

echo "   🚀 Tüm Sistemler Hazır. Başlatılıyor..."
echo "-----------------------------------------------------------"

# --- 2. LOG ve DİZİN AYARLARI ---
LOG_DIR="$HOME/CELEBILER_USV/logs"
mkdir -p "$LOG_DIR"
DOCKER_LOG_SOURCE="$HOME/CELEBILER_USV/docker_workspace/logs"
if [ -d "$DOCKER_LOG_SOURCE" ]; then
    ln -sfn "$DOCKER_LOG_SOURCE" "$LOG_DIR/docker"
fi
LOG_FILE="$LOG_DIR/system_boot.log"

echo "Log Merkezi: $LOG_DIR"
echo "Başlatılıyor..."

# 3. TEMİZLİK & KAMERA BAŞLATMA (HOST)
sudo fuser -k 8888/tcp > /dev/null 2>&1
echo -e "${GREEN}[KAMERA]${NC} Port 8888 temizleniyor ve yayın başlatılıyor..."
sudo pkill -9 rpicam-vid || true
sudo nice -n -20 rpicam-vid -t 0 --codec mjpeg --inline --listen -o tcp://0.0.0.0:8888 --width 1280 --height 720 --framerate 25 --quality 95 > "$LOG_DIR/cam_host.log" 2>&1 &
sleep 2

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

# 6. BAĞLANTI KONTROLÜ (5 Saniye İzle)
echo -e "\n🔎 [KONTROL] Donanım Bağlantıları Bekleniyor (5sn)..."
end=$((SECONDS+5))
tail -n 0 -f "$LOG_DIR/docker/telemetry.log" | while read line; do
    if [[ "$line" == *"✅"* ]] || [[ "$line" == *"🎮"* ]] || [[ "$line" == *"💓"* ]]; then
        echo "   -> $line"
    fi
    if [ $SECONDS -ge $end ]; then
        pkill -P $$ tail
        break
    fi
done || true

# 7. FİNAL BİLGİ TABLOSU
sleep 3
echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║              ✅ SİSTEM BAŞARIYLA AÇILDI!                     ║${NC}"
echo -e "${CYAN}╠══════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} 🌍 ERİŞİM ADRESLERİ (Ağınıza uygun olanı seçin):            ${CYAN}║${NC}"

for ip in $CURRENT_IPS; do
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
